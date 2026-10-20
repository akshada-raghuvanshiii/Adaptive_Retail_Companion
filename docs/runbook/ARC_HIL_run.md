# ARC HIL run

The HIL day setup (Pi kit over UDP, run by hand), with the **Windows laptop instead of the Pi**. Same chain:

webcam → Windows laptop (YOLOv8n + ByteTrack) → UDP → relay → BT → motion gate → `/cmd_vel` → Gazebo cart + Uno → real wheels

Two laptops:
- **ARC laptop**: Ubuntu, ROS 2, Gazebo, your nav stack, the Uno on USB.
- **Camera laptop**: Windows, VS Code, the webcam. No ROS.

Tonight (no webcam, no motors): skip step 4 and use the test clip in step 9.

## Before you start

1. **Both laptops on the same phone hotspot.** Wi-Fi stays on the whole time today. Don't switch networks until you stop.

## ARC laptop: one terminal per step, in this order

2. **Terminal 1, once per boot.** Loopback multicast on, ROS daemon reset:
   ```bash
   sudo ip link set lo multicast on && ros2 daemon stop
   ```

3. **Terminal 1: bring up the stack.** About 15 min. auto_map drives the store first; don't touch the robot.
   ```bash
   cd ~/arc_test_kit && source env.sh && TEB_YAML=$HOME/arc_test_kit/configs/teb_progress.yaml ./bringup.sh all
   ```
   Wait for **navigation stack is up**. Then **close the B6 `person_state_input` window** (Ctrl+C in it). The webcam replaces it, and the relay in step 5 refuses to start while B6 is open.

4. **Terminal 2: motors.** Wheels off the ground. Skip tonight.
   ```bash
   source ~/arc_test_kit/env.sh && ros2 launch arc_hw_bridge arc_hw.launch.py params_file:=$HOME/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml
   ```
   Wait for **Uno OK**.

5. **Terminal 3: relay** (webcam data → BT). It prints this laptop's addresses: **note the hotspot one** (it's the one sharing its first three numbers with the camera laptop). That's ARC_IP.
   ```bash
   ~/arc_pi_udp/laptop/run_relay.sh
   ```

6. **Terminal 4: buttons.** Click into this window before pressing **e** (enrol), **l** (lead me), **r** (reset).
   ```bash
   source ~/arc_test_kit/env.sh && python3 ~/arc_pi_udp/laptop/arc_ui.py
   ```

7. **Terminal 5: camera view** (what the camera laptop sees, with boxes).
   ```bash
   source ~/arc_test_kit/env.sh && ~/arc_perception/.venv/bin/python ~/arc_pi_udp/laptop/view_debug.py
   ```

8. **Terminal 6: RViz with the store labels.**
   ```bash
   source ~/arc_test_kit/env.sh && (python3 ~/arc_compact_store_kit/ros_tools/item_markers.py &) && rviz2 -d ~/arc_compact_store_kit/run/arc_nav.rviz --ros-args -p use_sim_time:=true
   ```

## Camera laptop (Windows, VS Code)

9. In VS Code, with the `arc_perception_laptop` folder open: **Terminal** menu (top bar) → **Run Task…**
   - Tonight: **ARC: 3 perception on the test clip**
   - Tomorrow: **ARC: 4 perception on the webcam** (camera number usually 0)

   Type **ARC_IP** from step 5 when it asks. Back on the ARC laptop, Terminal 3 prints **receiving from the Pi at …** (it calls any sender "the Pi") and Terminal 5 shows the picture.

## ARC laptop: the shopping list, last

10. **Terminal 7: Shopping Manager.**
    ```bash
    source ~/arc_test_kit/env.sh && ros2 run arc_planner shopping_manager
    ```
    Type the list (e.g. `pasta, milk, tea_coffee, fruits`), answer **n** to "Add more items?", **y** to "Confirm shopping list?", pick route **1** (optimised), and **y** at "Continue to next item?" after each stop.

## The demo

| Do | You should see |
|---|---|
| Stand alone in front of the webcam, press **e** in Terminal 4 | Terminal 5 shows **LOCKED** |
| Press **l** | B5 window prints **-> LEAD** |
| Enter the list in Terminal 7 | Gazebo cart drives, RViz shows the path, the real wheels turn |
| Step out of view | A few seconds later: BT **WAIT**, cart and wheels stop |
| Come back, press **e** then **l** | It carries on |
| Ctrl+C in the VS Code task on the camera laptop | After 1.5 s Terminal 3 prints **holding the BT in WAIT** |

## Stop, motors first

11. Ctrl+C in the camera laptop's task.
12. Ctrl+C in Terminal 2 (motors: sends STOP to the Uno).
13. Ctrl+C in Terminal 3 (relay), then:
    ```bash
    ~/arc_test_kit/bringup.sh stop
    ```

## If something's off

| What you see | Fix |
|---|---|
| Relay: `person_state_input (B6) is running` | Close the B6 window, run step 5 again |
| Relay keeps saying waiting for the Pi | Task 3/4 not running, wrong ARC_IP typed, laptops on different networks, or firewall: `sudo ufw allow 5600/udp` on the ARC laptop. On Windows allow python.exe on Private **and** Public |
| **e** / **l** do nothing | Windows firewall is blocking the replies: allow python.exe on Public too |
| **e** doesn't lock | Only one person in view, press **e** again |
| Cart never moves | Press **l** (LEAD); B6 still open; Terminal 2 not running |
| Robot stops mid-trip | Terminal 1: `cd ~/arc_test_kit && source env.sh && TEB_YAML=$PWD/configs/teb_progress.yaml ./bringup.sh nav`, close the new B6 window, restart step 10 |
| Camera laptop dead | Ctrl+C in Terminal 3, then `source ~/arc_test_kit/env.sh && ros2 run arc_behavior_tree person_state_input`, type **3** (LEAD); play `arch_w_perception_1.webm` for the perception part |
| Gazebo or RTAB-Map died | Play `final_1_1.webm` |
