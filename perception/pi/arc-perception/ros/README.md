# ROS 2 integration: perception → behaviour tree

`perception_bridge.py` replaces the `person_state_input` keyboard menu.
- **Input:** the camera, through YOLOv8n + ByteTrack.
- **Processing:** the same `TargetLock` and `AssistFSM` code as the offline evaluation and `tools/live_lock.py`.
- **Output:** the two flags your behaviour tree already reads.

| Lock / mode | `/person_moving` | `/person_confused` | BT branch → motion gate |
|---|---|---|---|
| LOCKED + FOLLOW | true | false | FollowPerson → FOLLOW |
| LOCKED + LEAD (shopper pressed "lead me") | false | true | LeadPerson → LEAD (reduced speed) |
| LOST, NEEDS_ENROL, WAIT, IDLE | false | false | WaitForPerson → robot held still |

Nothing in `~/slam_ws` is changed: `arc_behavior_tree`, `arc_motion_gate`, the Dijkstra planner and the controller are all untouched.
YOLO stays out of the C++ `arc_person_follower` package and out of colcon. The node runs with the perception venv's Python, with ROS sourced.

## Run (Ubuntu laptop)
1. Start the simulation and the navigation stack as in `compact_store_RUN.md`, **but not B6** (`person_state_input`).
2. In a new terminal:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/arc-perception && ~/arc_perception/.venv/bin/python ros/perception_bridge.py --max-lost-s 4
   ```
3. Click yourself in the window. In the B5 terminal, the BT should print `WAIT -> FOLLOW`.
4. Press `l` (lead me). The BT should print `FOLLOW -> LEAD`. Pick an item in the Shopping Manager and the robot drives to it.
5. Step out of view. The BT goes to WAIT and the robot stops.

Watch the bridge's output with `ros2 topic echo /arc/target_lock`.

## Run on the Raspberry Pi (headless)
The Pi setup, network and start scripts are in the separate `arc_pi` kit (`~/arc_pi/README.md`).
On the Pi the bridge runs without a window:
```bash
python ros/perception_bridge.py --headless --width 640 --height 480 --mjpg --imgsz 320 --debug-image-hz 2
```
- **Shopper buttons** come from the laptop on `/arc/ui/command` (`enrol`, `reset`, `lead_me`, `yes`, `no`, `stop`).
  Use `~/arc_pi/laptop/arc_ui.py`.
- **The annotated view** goes to the laptop on `/arc/perception/debug/compressed` (`--debug-image-hz`).
  Use `~/arc_pi/laptop/view_debug.py`.
- **If YOLO is too slow on the Pi**, the Pi can stream frames only (`arc_pi/pi/camera_pub.py`) and the laptop
  runs the bridge with `--camera ros:/arc/camera/image/compressed`. If frames stop for `--ros-timeout-s`,
  the bridge publishes wait and the shopper must be enrolled again.
- **SIGTERM** stops it the same way as Ctrl+C: it publishes wait before exiting.
- **The crash gap** (the BT keeping old flags) is covered on the laptop by `arc_pi/laptop/flag_watchdog.py`,
  without changing the BT.

## Safety notes
- It never publishes `/emergency_stop` or `/person_state`, so perception cannot raise or clear an emergency.
  Keep `person_state_input` handy for option 4 (emergency stop). Running it also publishes the two flags, so use it only to stop the robot, then close it.
- **Known gap:** if the bridge crashes, the BT keeps the last flags it received, because the BT has no staleness timeout.
  The fix is a small timeout in `person_assistant_bt`, which needs your approval to change.
- FOLLOW mode needs a goal near the shopper (`/person_pose` → `follow_goal_generator`). A single laptop camera gives only a direction, not a distance.
  Until a depth source is added, FOLLOW lets the robot move but there is no person goal for it to drive to.
  LEAD works with Shopping Manager goals.
