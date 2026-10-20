# ARC compact store: run sheet (verified end to end on the laptop, 12 Oct 2026)

**Quickest way:** `./arc_up.sh check`, then `./arc_up.sh map` (first time), then `./arc_up.sh nav` or
`./arc_up.sh nav --perception`. The launcher opens every terminal below in order; the manual steps follow.

Every terminal starts with:
```bash
source /opt/ros/jazzy/setup.bash && source ~/slam_ws/install/setup.bash
```

## 0. Clean start
```bash
pkill -f "gz sim"; pkill -f parameter_bridge; pkill -f rtabmap; pkill -f icp_odometry; pkill -f robot_state_publisher; pkill -f rviz2; pkill -f controller_server; pkill -f lifecycle_manager; pkill -f dijkstra_planner; pkill -f planned_path_to_teb; pkill -f person_assistant_bt; pkill -f arc_motion_gate; pkill -f shopping_manager; pkill -f item_markers; ros2 daemon stop
```

## 1. Simulation and robot
| T | Command | Expect |
|---|---|---|
| T1 | `ros2 launch arc_store_sim store.launch.py world:=arc_store.sdf` | compact store; `world:=` is required (launch default is the old store) |
| T2 | `export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true` | `Robot initialized` |
| T3 | `ros2 run ros_gz_sim create -name arc -file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x 0.0 -y 0.0 -z 0.01 -Y 0.0 && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/slam_ws/src/arc_store_sim/config/arc_store_burger_bridge.yaml -p use_sim_time:=true` | robot at the origin (x 0, y 0, yaw 0 is required); bridge lines |
| check | `timeout 4 ros2 topic hz /scan; ros2 topic echo --once /odom --field pose.pose.position` | /scan ~5 Hz, odom near 0,0 |

## 2. RTAB-Map with lidar (ICP) odometry
Wheel odometry alone slips (about 4° per 90° turn, much more on contact) and made RTAB-Map lose
localization during navigation. Lidar odometry uses the wheels only as a first guess.

T5a, lidar odometry:
```bash
ros2 run rtabmap_odom icp_odometry --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.5 --Icp/Iterations 10 --Icp/Epsilon 0.001 --Icp/OutlierRatio 0.7 --Icp/CorrespondenceRatio 0.2 --Odom/ScanKeyFrameThr 0.4 --OdomF2M/ScanSubtractRadius 0.05 --OdomF2M/ScanMaxSize 15000 --Reg/Force3DoF true --ros-args -p use_sim_time:=true -p frame_id:=base_footprint -p odom_frame_id:=icp_odom -p guess_frame_id:=odom -p publish_tf:=true -r scan:=/scan -r odom:=/odom_icp
```
T5b, mapping (`-d` = new database; omit it to reuse the saved map):
```bash
ros2 run rtabmap_slam rtabmap -d --Reg/Strategy 1 --Reg/Force3DoF true --Grid/Sensor 0 --Grid/RangeMin 0.2 --RGBD/NeighborLinkRefining true --Optimizer/GravitySigma 0 --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.5 --ros-args -r __ns:=/rtabmap -p use_sim_time:=true -p frame_id:=base_footprint -p subscribe_depth:=false -p subscribe_rgb:=false -p subscribe_scan:=true -p approx_sync:=true -r scan:=/scan -r odom:=/odom_icp
```
Optional viewer:
```bash
ros2 run rtabmap_viz rtabmap_viz --ros-args -r __ns:=/rtabmap -p use_sim_time:=true -p frame_id:=base_footprint -p subscribe_depth:=false -p subscribe_rgb:=false -p subscribe_scan:=true -p subscribe_odom_info:=false -p approx_sync:=true -r scan:=/scan -r odom:=/odom_icp
```
Check: `timeout 4 ros2 topic hz /odom_icp; timeout 5 ros2 run tf2_ros tf2_echo map base_footprint` (about 5 Hz; near 0,0).
TF chain: map → icp_odom → odom → base_footprint.

RViz: `rviz2 -d ~/arc_compact_store_kit/run/arc_nav.rviz --ros-args -p use_sim_time:=true`

## 3. Map the store, then lock the map
```bash
python3 ~/arc_compact_store_kit/ros_tools/auto_map.py
```
- 11-waypoint route, steering by the map pose; expect `11 reached, 0 skipped` and a small `map err`.
- No teleop or BT stack during this run.

Then lock the map:
```bash
ros2 service call /rtabmap/set_mode_localization std_srvs/srv/Empty
```

## 4. Navigation (each in its own terminal, in this order)
| | Command |
|---|---|
| B1 | `ros2 run nav2_controller controller_server --ros-args --params-file $HOME/slam_ws/src/arc_planner/config/teb_controller.yaml -r cmd_vel:=/cmd_vel_nav` |
| B2 | `ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_controller -p use_sim_time:=true -p autostart:=true -p node_names:="['controller_server']"` |
| B3 | `ros2 run arc_planner dijkstra_planner --ros-args -p use_sim_time:=true` |
| B4 | `ros2 run arc_planner planned_path_to_teb --ros-args -p use_sim_time:=true` |
| B5 | `ros2 launch arc_behavior_tree arc_bt_stack.launch.py` |
| B6 | `ros2 run arc_behavior_tree person_state_input` → choose **3** (LEAD) or **1** (FOLLOW); with 2 or 4 nothing moves |
| B8 | `python3 ~/arc_compact_store_kit/ros_tools/item_markers.py` (store labels in RViz) |
| B7 | `ros2 run arc_planner shopping_manager` (last) |

Verify at any stop:
```bash
python3 ~/arc_compact_store_kit/ros_tools/check_frames.py --model arc --samples 1
```
PASS means the map pose is within 0.15 m / 5° of the Gazebo truth.

## Item layout
`python3 ~/arc_compact_store_kit/assign_items.py --write` places all 29 items by aisle:
- **Fresh:** produce, dairy, tea & drinks.
- **Aisle 1:** gondola 1, west face.
- **Aisle 2:** gondola 1 east face, gondola 2 west face.
- **Aisle 3:** gondola 2 east face, gondola 3.

It backs up the previous file to `~/slam_ws/arc_backups/`. Restart `shopping_manager` after it runs.

## Notes
- `src/arc_planner_claude` carries `COLCON_IGNORE` (duplicate `arc_planner` package). Build with `colcon build --symlink-install`.
- Do not run `arc_backups/compact_store_20261012_133112/RESTORE.sh`: it predates the goal-frame guard in `dijkstra_planner.py`.
