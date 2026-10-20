- 10:45 NOTE: A2 attempt 2 (10:16-10:41) stopped: Gazebo bridge lost multicast ('Network is unreachable'), sensors froze, robot drove on last command into west wall. Fix: GZ_IP=127.0.0.1 + ROS discovery LOCALHOST (kit 2026-10-15.6)

## 2026-10-15 10:53 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_57200119b040
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=-0.00 y=0.08 yaw=-80.3
[OK] Gazebo true pose x=-0.01 y=0.07 yaw=-80.4 (source gzcli)
[OK] map vs truth 0.007 m, 0.2 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] spawn/remove test box works
```

## 2026-10-15 10:53 - smoke

- Command: `python3 arc_tests/mission_runner.py --out results --label smoke --pairs 1 --seed 5 `
- Gazebo real-time factor at start: 1.0053469376880941
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=228b29f9a313 size=6.1M modified=2026-10-15 10:52
- Snapshot: `results/run_logs/20261015_105310_smoke`
- Observations: (add with ./run.sh note "...")
- Finished 10:54 (smoke, label `smoke`, cumulative for this label):
  - missions completed 1/1; legs reached 1/1 (1 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 0; legs with clearance <= 0: 0
  - leg failures: none
  - median localisation error 0.022 m over 344 samples

## 2026-10-15 10:54 - a2

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp --lists 30 --size-range 3-12 --strategies as_entered --seed 7 `
- Gazebo real-time factor at start: 0.99995150235213592
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=228b29f9a313 size=6.1M modified=2026-10-15 10:52
- Snapshot: `results/run_logs/20261015_105451_a2`
- Observations: (add with ./run.sh note "...")
- Finished 11:17 (a2, label `teb_icp`, cumulative for this label):
  - missions completed 3/4; legs reached 16/17 (16 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 6; legs with clearance <= 0: 0
  - leg failures: teb_icp_s7_003 leg 2 (sim_link_lost)
  - median localisation error 0.030 m over 4420 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_s7_003 leg 2 - no Gazebo pose for 15 s
- 11:24 NOTE: A2 stop at 11:16 (mission 4) was a false alarm: gz pose service not answering (Host unreachable), sim fine, robot at goal. Kit 2026-10-15.7 stops only on frozen /clock.

## 2026-10-15 11:24 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_57200119b040
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status visible
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=3.47 y=-0.37 yaw=3.9
[OK] Gazebo true pose x=3.47 y=-0.41 yaw=3.7 (source gzcli)
[OK] map vs truth 0.035 m, 0.2 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] spawn/remove test box works
```

## 2026-10-15 11:24 - a2

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp --lists 30 --size-range 3-12 --strategies as_entered --seed 7 --start-at 3`
- Gazebo real-time factor at start: 1.0016331628720629
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=228b29f9a313 size=6.1M modified=2026-10-15 10:52
- Snapshot: `results/run_logs/20261015_112438_a2`
- Observations: (add with ./run.sh note "...")
- Finished 13:50 (a2, label `teb_icp`, cumulative for this label):
  - missions completed 29/30; legs reached 143/144 (143 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 97; legs with clearance <= 0: 0
  - leg failures: teb_icp_s7_021 leg 1 (controller_aborted)
  - median localisation error 0.029 m over 21591 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_s7_003 leg 2 - no Gazebo pose for 15 s

## 2026-10-15 16:55 - c1 ordering benchmark (offline)

- Command: `ordering_benchmark.py `
- Map: results/map.npz (md5 66187cab5ca5)
- Output: ordering_benchmark.csv

## 2026-10-15 17:17 - a3-teb

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pairs --pairs 20 --seed 5 `
- Gazebo real-time factor at start: 1.0007438028314546
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=228b29f9a313 size=6.1M modified=2026-10-15 10:52
- Snapshot: `results/run_logs/20261015_171749_a3-teb`
- Observations: (add with ./run.sh note "...")
- Finished 17:42 (a3-teb, label `teb_icp_pairs`, cumulative for this label):
  - missions completed 10/12; legs reached 10/12 (10 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 15; legs with clearance <= 0: 0
  - leg failures: teb_icp_pairs_s5_010 leg 0 (controller_aborted), teb_icp_pairs_s5_011 leg 0 (sim_link_lost)
  - median localisation error 0.029 m over 1697 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_pairs_s5_011 leg 0 - /clock frozen for 15 s
- 17:50 NOTE: A3-teb stopped at pair 12 (17:41): Wi-Fi reconnect broke ROS 2 discovery (/clock unseen, Gazebo fine RTF 1.0). Fix: Fast DDS pinned to loopback+SHM (kit 2026-10-15.8), Wi-Fi off during runs.
- 18:11 NOTE: 18:09 re-mapped (map err <=0.04 m all waypoints, lock 0.010 m), Fast DDS loopback profile active, Wi-Fi off

## 2026-10-15 18:11 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_55ff2e69c140
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=0.00 y=0.08 yaw=-85.0
[OK] Gazebo true pose x=-0.01 y=0.07 yaw=-85.1 (source gzcli)
[OK] map vs truth 0.007 m, 0.0 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] spawn/remove test box works
```

## 2026-10-15 18:11 - a3-teb

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pairs --pairs 20 --seed 5 --start-at 11`
- Gazebo real-time factor at start: 0.99988276374595075
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=997a2e62da06 size=6.2M modified=2026-10-15 18:09
- Snapshot: `results/run_logs/20261015_181129_a3-teb`
- Observations: (add with ./run.sh note "...")
- Finished 18:32 (a3-teb, label `teb_icp_pairs`, cumulative for this label):
  - missions completed 19/20; legs reached 19/20 (19 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 17; legs with clearance <= 0: 0
  - leg failures: teb_icp_pairs_s5_010 leg 0 (controller_aborted)
  - median localisation error 0.020 m over 6202 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_pairs_s5_011 leg 0 - /clock frozen for 15 s

## 2026-10-15 18:36 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_55ff2e69c140
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=3.46 y=-0.41 yaw=0.5
[OK] Gazebo true pose x=3.46 y=-0.44 yaw=-0.2 (source gzcli)
[OK] map vs truth 0.030 m, 0.7 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] spawn/remove test box works
```

## 2026-10-15 18:36 - a3-rpp

- Command: `python3 arc_tests/mission_runner.py --out results --label rpp_icp_pairs --pairs 20 --seed 5 `
- Gazebo real-time factor at start: 1.001554161670372
- Controller: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=997a2e62da06 size=6.2M modified=2026-10-15 18:09
- Snapshot: `results/run_logs/20261015_183606_a3-rpp`
- Observations: (add with ./run.sh note "...")
- Finished 19:05 (a3-rpp, label `rpp_icp_pairs`, cumulative for this label):
  - missions completed 14/20; legs reached 14/20 (14 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 24; legs with clearance <= 0: 0
  - leg failures: rpp_icp_pairs_s5_000 leg 0 (controller_aborted), rpp_icp_pairs_s5_003 leg 0 (controller_aborted), rpp_icp_pairs_s5_004 leg 0 (controller_aborted), rpp_icp_pairs_s5_005 leg 0 (controller_aborted), rpp_icp_pairs_s5_014 leg 0 (controller_aborted), rpp_icp_pairs_s5_015 leg 0 (controller_aborted)
  - median localisation error 0.018 m over 4443 samples
- 19:10 NOTE: A3 done: TEB 19/20, RPP 14/20 (RPP aborts mostly code 106 no-valid-control = collision check in narrow aisles; TEB code 105 progress check). B4 log before 19:07 overwritten by a duplicate nav start (kit bug, fixed in 2026-10-15.10); aborts still in events.csv

## 2026-10-15 19:10 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_55ff2e69c140
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=3.38 y=-0.17 yaw=-0.5
[OK] Gazebo true pose x=3.38 y=-0.20 yaw=-1.3 (source gzcli)
[OK] map vs truth 0.034 m, 0.8 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] spawn/remove test box works
```

## 2026-10-15 19:10 - a4

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_boxes --pairs 20 --seed 9 --pair-min-dist 2.5 --obstacles 1-3 `
- Gazebo real-time factor at start: 1.000075005625422
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=997a2e62da06 size=6.2M modified=2026-10-15 18:09
- Snapshot: `results/run_logs/20261015_191035_a4`
- Observations: (add with ./run.sh note "...")
- Finished 19:31 (a4, label `teb_icp_boxes`, cumulative for this label):
  - missions completed 0/14; legs reached 0/14 (0 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 42; legs with clearance <= 0: 4
  - leg failures: teb_icp_boxes_s9_000 leg 0 (controller_aborted), teb_icp_boxes_s9_001 leg 0 (controller_aborted), teb_icp_boxes_s9_002 leg 0 (controller_aborted), teb_icp_boxes_s9_003 leg 0 (controller_aborted), teb_icp_boxes_s9_004 leg 0 (controller_aborted), teb_icp_boxes_s9_005 leg 0 (controller_aborted), teb_icp_boxes_s9_006 leg 0 (controller_aborted), teb_icp_boxes_s9_007 leg 0 (controller_aborted), teb_icp_
  - median localisation error 0.018 m over 2469 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_boxes_s9_014 leg -1 - map pose 3.15 m from truth for 10 s
- 19:45 VOID: label `teb_icp_boxes` set aside (samples.csv 4551, legs.csv 29, missions.csv 14, events.csv 259) -> results/void/teb_icp_boxes_20261015_194510. Reason: 12 test boxes left in Gazebo (removal not verified - kit bug), later legs drove into blocked aisles; map pose lost 3.15 m at pair14
- 20:26 NOTE: A4 rerun on fresh map 19:45 session (post-lock check 0.052 m / 0.8 deg; pre-lock 0.05 m check not met after 3 revisits). Kit 2026-10-15.11 with verified box removal.

## 2026-10-15 20:26 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_5fdc600643e0
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: False
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 180x161 @ 0.050 m
[OK] TF map -> base_footprint x=-0.05 y=-0.03 yaw=47.7
[OK] Gazebo true pose x=-0.04 y=0.02 yaw=47.0 (source gzcli)
[OK] map vs truth 0.051 m, 0.7 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-15 20:26 - a4

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_boxes --pairs 20 --seed 9 --pair-min-dist 2.5 --obstacles 1-3 `
- Gazebo real-time factor at start: 0.99994275327737492
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: unknown
- Map: rtabmap.db md5=8276dbc0d2a2 size=11M modified=2026-10-15 19:55
- Snapshot: `results/run_logs/20261015_202644_a4`
- Observations: (add with ./run.sh note "...")
- Finished 20:35 (a4, label `teb_icp_boxes`, cumulative for this label):
  - missions completed 2/5; legs reached 2/5 (2 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 9; legs with clearance <= 0: 3
  - leg failures: teb_icp_boxes_s9_000 leg 0 (controller_aborted), teb_icp_boxes_s9_002 leg 0 (controller_aborted), teb_icp_boxes_s9_003 leg 0 (controller_aborted)
  - median localisation error 0.091 m over 1712 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_boxes_s9_005 leg -1 - map pose 2.30 m from truth for 10 s
- 21:29 NOTE: A4 pair04 (20:34-20:35): ICP odometry failed permanently ~20:35:04 during the 3-box leg, near box contact (clearance -0.023 m); libpointmatcher 'limit out of bounds tr .../0.2', ~297 failures/min, never recovered; 1 failure before that all session. Likely wheel slip against a static box -> guess off > 0.2 m. pair01 false arrival (stale path) fixed in kit 2026-10-15.12; rerunning from pair01 on a fresh map.
- 21:36 NOTE: 21:29 bring-up: auto_map stopped at wp9; map err 0.07 -> 0.31 m on the aisle 2-3 north->south drive (wp4->wp5), odom err 0.36 m; likely ICP sliding along the aisle (not confirmed). Log saved as run_logs/auto_map_FAILED_2129.csv. Retrying.

## 2026-10-15 21:43 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_60743db095f0
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.08 yaw=-80.2
[OK] Gazebo true pose x=-0.01 y=0.07 yaw=-80.1 (source gzcli)
[OK] map vs truth 0.014 m, 0.1 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-15 21:44 - a4

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_boxes --pairs 20 --seed 9 --pair-min-dist 2.5 --obstacles 1-3 --start-at 1`
- Gazebo real-time factor at start: 0.999745314881034
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=dda81aea8f6e size=6.1M modified=2026-10-15 21:42
- Snapshot: `results/run_logs/20261015_214354_a4`
- Observations: (add with ./run.sh note "...")
- Finished 22:11 (a4, label `teb_icp_boxes`, cumulative for this label):
  - missions completed 4/15; legs reached 4/15 (4 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 31; legs with clearance <= 0: 13
  - leg failures: teb_icp_boxes_s9_000 leg 0 (controller_aborted), teb_icp_boxes_s9_001 leg 0 (controller_aborted), teb_icp_boxes_s9_003 leg 0 (controller_aborted), teb_icp_boxes_s9_004 leg 0 (controller_aborted), teb_icp_boxes_s9_005 leg 0 (controller_aborted), teb_icp_boxes_s9_007 leg 0 (controller_aborted), teb_icp_boxes_s9_008 leg 0 (controller_aborted), teb_icp_boxes_s9_009 leg 0 (controller_aborted), teb_icp_
  - median localisation error 0.043 m over 6444 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_boxes_s9_014 leg 0 - map pose 1.93 m from truth for 10 s
- 23:07 NOTE: A4 21:36 session: map 0.013 m; pairs 01-14 rerun; stopped pair14 (3 boxes) - ICP loss after contact (2nd time). pair11 = no-box/no-start trial. Leftover box found before pair11 leg 0 (pair11 leg -1 possibly contaminated). Runner showed controller=unknown (param query timeout); probe confirmed TEB.

## 2026-10-15 23:10 - probe

```
[FAIL] node dijkstra not running - start B3
[FAIL] node controller_server not running - start B1
[FAIL] node planned_path_to_teb not running - start B4
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_55753b917ab0
[FAIL] node motion_gate not running - start B5
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[WARN] controller plugin None
[FAIL] goal topic /goal_pose - the Dijkstra planner is not subscribed
[FAIL] /navigation_goal_reached no publisher (B4)
[WARN] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 177x159 @ 0.050 m
[OK] TF map -> base_footprint x=1.81 y=2.63 yaw=5.4
[OK] Gazebo true pose x=1.82 y=2.90 yaw=5.3 (source gzcli)
[WARN] map vs truth 0.270 m, 0.1 deg (outside 0.15 m / 5 deg)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[FAIL] BT mode /arc/behavior_mode None - the gate holds the robot; type 3 in B6
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-15 23:18 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_646e350f20c0
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 178x160 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.06 yaw=-80.0
[OK] Gazebo true pose x=-0.01 y=0.05 yaw=-80.0 (source gzcli)
[OK] map vs truth 0.007 m, 0.0 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-15 23:18 - c2

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_c2 --sizes 5,8,12 --per-size 10 --strategies as_entered,nn_dijkstra,exact --seed 21 `
- Gazebo real-time factor at start: 1.0007500621715977
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=7ddc64d8076e size=6.2M modified=2026-10-15 23:17
- Snapshot: `results/run_logs/20261015_231844_c2`
- Observations: (add with ./run.sh note "...")
- Finished 08:01 (c2, label `teb_icp_c2`, cumulative for this label):
  - missions completed 47/90; legs reached 476/543 (476 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 611; legs with clearance <= 0: 0
  - leg failures: teb_icp_c2_s21_002 leg 2 (controller_aborted), teb_icp_c2_s21_002 leg 3 (controller_aborted), teb_icp_c2_s21_010 leg 4 (controller_aborted), teb_icp_c2_s21_012 leg 1 (controller_aborted), teb_icp_c2_s21_014 leg 2 (controller_aborted), teb_icp_c2_s21_016 leg 3 (controller_aborted), teb_icp_c2_s21_017 leg 2 (controller_aborted), teb_icp_c2_s21_019 leg 0 (controller_aborted), teb_icp_c2_s21_019 leg 2
  - median localisation error 0.029 m over 35079 samples

## 2026-10-16 09:42 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_649f0f64f9e0
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=-0.00 y=0.07 yaw=-81.2
[OK] Gazebo true pose x=-0.01 y=0.06 yaw=-81.5 (source gzcli)
[OK] map vs truth 0.012 m, 0.4 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-16 09:42 - a4

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_boxes --pairs 20 --seed 9 --pair-min-dist 2.5 --obstacles 1-3 --start-at 15`
- Gazebo real-time factor at start: 0.99991900656046862
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=be044a43ef3d size=6.1M modified=2026-10-16 09:41
- Snapshot: `results/run_logs/20261016_094208_a4`
- Observations: (add with ./run.sh note "...")
- Finished 09:52 (a4, label `teb_icp_boxes`, cumulative for this label):
  - missions completed 5/20; legs reached 5/20 (5 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 44; legs with clearance <= 0: 17
  - leg failures: teb_icp_boxes_s9_000 leg 0 (controller_aborted), teb_icp_boxes_s9_001 leg 0 (controller_aborted), teb_icp_boxes_s9_003 leg 0 (controller_aborted), teb_icp_boxes_s9_004 leg 0 (controller_aborted), teb_icp_boxes_s9_005 leg 0 (controller_aborted), teb_icp_boxes_s9_007 leg 0 (controller_aborted), teb_icp_boxes_s9_008 leg 0 (controller_aborted), teb_icp_boxes_s9_009 leg 0 (controller_aborted), teb_icp_
  - median localisation error 0.038 m over 8382 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_boxes_s9_014 leg 0 - map pose 1.93 m from truth for 10 s

## 2026-10-16 10:00 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_610ec57089e0
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry wheel (/odom); icp_odometry running: False
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=-0.00 y=0.08 yaw=-81.7
[OK] Gazebo true pose x=-0.01 y=0.08 yaw=-81.9 (source gzcli)
[OK] map vs truth 0.003 m, 0.2 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-16 10:00 - b1

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_wheel --lists 10 --size-range 3-12 --strategies as_entered --seed 7 `
- Gazebo real-time factor at start: 0.99890470099535855
- Controller: teb_local_planner/TebController; RTAB-Map odometry: wheel; BT mode: LEAD
- Map: rtabmap.db md5=ed29f30fdf16 size=6.3M modified=2026-10-16 09:59
- Snapshot: `results/run_logs/20261016_100010_b1`
- Observations: (add with ./run.sh note "...")
- Finished 10:18 (b1, label `teb_wheel`, cumulative for this label):
  - missions completed 3/4; legs reached 14/15 (14 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 7; legs with clearance <= 0: 0
  - leg failures: teb_wheel_s7_003 leg 0 (localisation_lost)
  - median localisation error 0.026 m over 3890 samples
  - STOPPED (localisation or Gazebo link lost): teb_wheel_s7_003 leg 0 - map pose 1.34 m from truth for 10 s
- 10:48 NOTE: B1 list 4 (s7_003): RTAB-Map running but correction frozen ~80 s from sim_t 1392; 3 localizations rejected as wrong loop closure (graph error ratio > 3) - wheel drift 1-2 m makes correct matches look inconsistent; map err grew to 1.34 m. Resuming B1 from list 4 on a fresh wheel map.

## 2026-10-16 11:13 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_60cc3e69e060
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry wheel (/odom); icp_odometry running: False
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 180x159 @ 0.050 m
[OK] TF map -> base_footprint x=-0.00 y=0.08 yaw=-81.2
[OK] Gazebo true pose x=-0.01 y=0.07 yaw=-81.2 (source gzcli)
[OK] map vs truth 0.007 m, 0.1 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-16 11:14 - b1

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_wheel --lists 10 --size-range 3-12 --strategies as_entered --seed 7 --start-at 3`
- Gazebo real-time factor at start: 1.0015817479754903
- Controller: teb_local_planner/TebController; RTAB-Map odometry: wheel; BT mode: LEAD
- Map: rtabmap.db md5=bacf24c9d53d size=6.2M modified=2026-10-16 11:12
- Snapshot: `results/run_logs/20261016_111400_b1`
- Observations: (add with ./run.sh note "...")
- Finished 11:26 (b1, label `teb_wheel`, cumulative for this label):
  - missions completed 4/5; legs reached 22/23 (22 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 10; legs with clearance <= 0: 1
  - leg failures: teb_wheel_s7_004 leg 4 (localisation_lost)
  - median localisation error 0.023 m over 6556 samples
  - STOPPED (localisation or Gazebo link lost): teb_wheel_s7_004 leg 4 - map pose 1.16 m from truth for 10 s

## 2026-10-16 12:33 - footprint check (offline)

```
Cart: standard 180 L trolley 1000 x 610 mm. Map: /home/akshada-raghuvanshi/arc_test_kit/results/map.npz
footprint                       radius  start  stops   snap  longer  unreachable
kit grid (C1/C2)                  0.15     ok    9/9    0.0    0.0%  -
ARC planner, TurtleBot tests      0.20     ok    9/9    0.0    1.6%  -
cart, straight (lenient)          0.35     ok    9/9    0.0    9.7%  -
cart, turning (strict)            0.59     ok    9/9   0.85    5.1%  -
wrote /home/akshada-raghuvanshi/arc_test_kit/results/footprint_check.csv
```

## 2026-10-16 12:33 - footprint check (offline)

```
Cart: standard 180 L trolley 1000 x 610 mm. Map: /home/akshada-raghuvanshi/arc_test_kit/results/map.npz
footprint                       radius  start  stops   snap  longer  unreachable
kit grid (C1/C2)                  0.15     ok    9/9    0.0    0.0%  -
ARC planner, TurtleBot tests      0.20     ok    9/9    0.0    1.6%  -
cart, straight (lenient)          0.35     ok    9/9    0.0    9.7%  -
cart, turning (strict)            0.59     ok    9/9   0.85    5.1%  -
wrote /home/akshada-raghuvanshi/arc_test_kit/results/footprint_check.csv
```

## 2026-10-16 22:04 - probe

```
[FAIL] node dijkstra not running - start B3
[FAIL] node controller_server not running - start B1
[FAIL] node planned_path_to_teb not running - start B4
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_59426c759360
[FAIL] node motion_gate not running - start B5
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[WARN] controller plugin None
[FAIL] goal topic /goal_pose - the Dijkstra planner is not subscribed
[FAIL] /navigation_goal_reached no publisher (B4)
[WARN] FollowPath action status publisher present, no goals yet
[FAIL] map /rtabmap/map no map received in 10 s
[FAIL] TF map -> base_footprint not available
```

## 2026-10-16 22:16 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_55a5ed8ce820
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.07 yaw=-79.4
[OK] Gazebo true pose x=-0.01 y=0.06 yaw=-79.4 (source gzcli)
[OK] map vs truth 0.013 m, 0.0 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-16 22:17 - a2-pc

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pc --lists 30 --size-range 3-12 --strategies as_entered --seed 7 `
- Gazebo real-time factor at start: 1.0015917797359453
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=cb874ac4528c size=6.0M modified=2026-10-16 22:15
- Snapshot: `results/run_logs/20261016_221658_a2-pc`
- Observations: (add with ./run.sh note "...")
- Finished 22:21 (a2-pc, label `teb_icp_pc`, cumulative for this label):
  - missions completed 0/0; legs reached 3/3 (3 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 0; legs with clearance <= 0: 0
  - leg failures: none
  - median localisation error 0.022 m over 741 samples

## 2026-10-16 22:21 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_55a5ed8ce820
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[WARN] controller plugin None
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status visible
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=3.04 y=-1.07 yaw=-140.9
[OK] Gazebo true pose x=2.63 y=-1.29 yaw=-177.6 (source gzcli)
[OK] map vs truth 0.066 m, 0.3 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-16 22:21 - a2-final

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pc_final --lists 30 --size-range 3-12 --strategies exact --seed 7 `
- Gazebo real-time factor at start: 1.0005495518413487
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=cb874ac4528c size=6.0M modified=2026-10-16 22:15
- Snapshot: `results/run_logs/20261016_222125_a2-final`
- Observations: (add with ./run.sh note "...")
- 22:23 VOID: label `teb_icp_pc` set aside (samples.csv 841, legs.csv 4, events.csv 15) -> results/void/teb_icp_pc_20261016_222347. Reason: aborted at list 1, clean restart 16 Oct 22:2x
- 22:23 VOID: label `teb_icp_pc_final` set aside (samples.csv 365, legs.csv 2, events.csv 9) -> results/void/teb_icp_pc_final_20261016_222351. Reason: clean restart

## 2026-10-16 22:33 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_5fcc0375df20
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.07 yaw=-76.4
[OK] Gazebo true pose x=-0.01 y=0.06 yaw=-76.7 (source gzcli)
[OK] map vs truth 0.004 m, 0.2 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-16 22:33 - a2-pc

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pc --lists 30 --size-range 3-12 --strategies as_entered --seed 7 `
- Gazebo real-time factor at start: 0.999577678430863
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=fd5c4a68cfb1 size=6.0M modified=2026-10-16 22:31
- Snapshot: `results/run_logs/20261016_223325_a2-pc`
- Observations: (add with ./run.sh note "...")
- Finished 23:00 (a2-pc, label `teb_icp_pc`, cumulative for this label):
  - missions completed 4/5; legs reached 20/21 (20 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 2; legs with clearance <= 0: 1
  - leg failures: teb_icp_pc_s7_004 leg 2 (localisation_lost)
  - median localisation error 0.068 m over 5336 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_pc_s7_004 leg 2 - map pose 2.32 m from truth for 10 s

## 2026-10-16 23:01 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[FAIL] node planned_path_to_teb not running - start B4
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_5fcc0375df20
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status visible
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=-0.71 y=-1.10 yaw=179.3
[OK] Gazebo true pose x=2.94 y=-0.85 yaw=-180.0 (source gzcli)
[WARN] map vs truth 4.082 m, 43.6 deg (outside 0.15 m / 5 deg)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```
- 09:46 NOTE: a2-pc stopped 16 Oct 23:00 at list 5 leg 2 (tea_coffee): static contact -0.055 m, then localisation lost (2.32 m), ICP never recovered; B4 found dead afterwards. List 5 kept as failed; resuming from list 6 with a fresh map 17 Oct.
- 12:52 NOTE: 17 Oct: two mapping drives stopped by the mid-drive guard (0.16 m, 0.26 m at aisle north ends). Guard relaxed 0.15 -> 0.30 m for the drive only (MAP_GUARD_M); final map acceptance unchanged (end 0.05 m, locked 0.15 m/5 deg). Bring-up now retried up to 3x.

## 2026-10-17 13:03 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[FAIL] node rtabmap not running - start T5b
[OK] node motion_gate /arc_motion_gate
[WARN] RTAB-Map odometry unknown (None); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.08 yaw=-78.4
[OK] Gazebo true pose x=-0.01 y=0.07 yaw=-78.5 (source gzcli)
[OK] map vs truth 0.010 m, 0.1 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-17 13:12 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_59b8a2211b80
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x160 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.07 yaw=-82.1
[OK] Gazebo true pose x=-0.01 y=0.06 yaw=-82.5 (source gzcli)
[OK] map vs truth 0.009 m, 0.3 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-17 13:12 - a2-pc

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pc --lists 30 --size-range 3-12 --strategies as_entered --seed 7 --start-at 6`
- Gazebo real-time factor at start: 0.999538712884004
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=26892b345b55 size=6.2M modified=2026-10-17 13:09
- Snapshot: `results/run_logs/20261017_131215_a2-pc`
- Observations: (add with ./run.sh note "...")
- Finished 15:27 (a2-pc, label `teb_icp_pc`, cumulative for this label):
  - missions completed 28/29; legs reached 137/138 (137 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 4; legs with clearance <= 0: 1
  - leg failures: teb_icp_pc_s7_004 leg 2 (localisation_lost)
  - median localisation error 0.025 m over 23626 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_pc_s7_004 leg 2 - map pose 2.32 m from truth for 10 s

## 2026-10-17 15:38 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_607aacb2e4e0
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x159 @ 0.050 m
[OK] TF map -> base_footprint x=0.00 y=0.06 yaw=-82.6
[OK] Gazebo true pose x=-0.01 y=0.06 yaw=-82.9 (source gzcli)
[OK] map vs truth 0.012 m, 0.2 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[WARN] spawn/remove test box failed (spawn False, listed True, removed False); A4 unavailable
```

## 2026-10-17 15:38 - a2-final

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pc_final --lists 30 --size-range 3-12 --strategies exact --seed 7 `
- Gazebo real-time factor at start: 1.0105215503825835
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=b5538d5b864c size=6.3M modified=2026-10-17 15:36
- Snapshot: `results/run_logs/20261017_153850_a2-final`
- Observations: (add with ./run.sh note "...")
- Finished 17:58 (a2-final, label `teb_icp_pc_final`, cumulative for this label):
  - missions completed 30/30; legs reached 144/144 (143 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 13; legs with clearance <= 0: 0
  - leg failures: none
  - median localisation error 0.035 m over 18834 samples
- 18:09 NOTE: 17 Oct: s7_r05 (index 5) was skipped by --start-at 6 (0-based); running it alone now.

## 2026-10-17 18:18 - probe

```
[OK] node dijkstra /dijkstra_planner
[OK] node controller_server /controller_server
[OK] node planned_path_to_teb /planned_path_to_teb
[OK] node rtabmap /rtabmap/rtabmap, /rtabmap/transform_listener_impl_58042de94f60
[OK] node motion_gate /arc_motion_gate
[OK] RTAB-Map odometry icp (/odom_icp); icp_odometry running: True
[OK] controller plugin teb_local_planner/TebController
[OK] goal topic /goal_pose (subscribed by /dijkstra_planner)
[OK] /navigation_goal_reached published by planned_path_to_teb
[OK] FollowPath action status publisher present, no goals yet
[OK] map /rtabmap/map 179x161 @ 0.050 m
[OK] TF map -> base_footprint x=-0.01 y=0.06 yaw=-80.1
[OK] Gazebo true pose x=-0.02 y=0.05 yaw=-80.3 (source gzcli)
[OK] map vs truth 0.009 m, 0.0 deg (PASS)
[OK] /cmd_vel type TwistStamped (matches arc_motion_gate)
[OK] BT mode /arc/behavior_mode LEAD
[OK] item file /home/akshada-raghuvanshi/slam_ws/src/arc_store_sim/config/item_locations.yaml: 29 items, 9 driving stops
[OK] gz CLI found
[OK] leftover test boxes none in Gazebo
[OK] spawn/remove test box works (checked in `gz model --list`)
```

## 2026-10-17 18:18 - a2-pc

- Command: `python3 arc_tests/mission_runner.py --out results --label teb_icp_pc --lists 30 --size-range 3-12 --strategies as_entered --seed 7 --start-at 5 --lists 6`
- Gazebo real-time factor at start: 0.99560415873813146
- Controller: teb_local_planner/TebController; RTAB-Map odometry: icp; BT mode: LEAD
- Map: rtabmap.db md5=5873eecee71b size=6.1M modified=2026-10-17 18:16
- Snapshot: `results/run_logs/20261017_181836_a2-pc`
- Observations: (add with ./run.sh note "...")
- Finished 18:22 (a2-pc, label `teb_icp_pc`, cumulative for this label):
  - missions completed 29/30; legs reached 140/141 (140 by /navigation_goal_reached)
  - recoveries (controller aborts + goal re-sends) 4; legs with clearance <= 0: 1
  - leg failures: teb_icp_pc_s7_004 leg 2 (localisation_lost)
  - median localisation error 0.025 m over 24288 samples
  - STOPPED (localisation or Gazebo link lost): teb_icp_pc_s7_004 leg 2 - map pose 2.32 m from truth for 10 s
