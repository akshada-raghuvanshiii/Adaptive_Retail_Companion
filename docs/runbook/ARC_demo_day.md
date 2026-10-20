# ARC Demo Day

The final review on 21 October: the simulated store and your nav stack, webcam perception on Manav's Pi, and the real motors in the loop, all at once. Start bring-up at least 30 minutes before you present. Tick steps as you go; ticks stay in this browser.

laptop: Ubuntu 24.04 · Jazzy · Harmonic
Pi: manav@172.20.10.2, no ROS
≈ 20 min to ready

webcam→Pi: YOLOv8n + ByteTrack→ UDP →relay→BT→motion gate→ /cmd_vel →Gazebo cart+Uno → real wheels

Purple is real hardware, green is the laptop. The store, lidar and robot body are simulated; the camera, Pi, Uno and motors are real, with the wheels off the ground.
0 / 0

## Check the pieces tonight

All read-only except the UI install. Do the full rehearsal on the next tab once tonight, so tomorrow is a repeat, not a first try.

- [ ] **The kits are where the demo expects them**
_(Terminal K)_

```bash
ls ~/arc_pi_udp/laptop/arc_demo.sh ~/arc_test_kit/bringup.sh ~/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml && grep "port:" ~/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml && ls /dev/ttyACM*
```

Three paths, then the Uno port, which must match `/dev/ttyACM…`. Plug the Uno into the laptop first.

- [ ] **The laptop is in local network mode**
_(Terminal K)_

```bash
cat ~/.arc_net 2>/dev/null || echo local
```

Must print **local**. The Pi talks plain UDP, so ROS stays on the laptop. If it prints **hil**, run `~/arc_pi/laptop/hil_mode.sh off`.

- [ ] **Make the progress-check controller file**
_(Terminal K)_

```bash
cd ~/arc_test_kit && source env.sh && ./run.sh make-progress && grep -n -A4 "progress_checker" configs/teb_progress.yaml
```

Shows **0.25** and **20.0**. This stops the "robot stopped, not moving" abort from the UI trip. Your own `teb_controller.yaml` is only read.

- [ ] **Motors answer (wheels off the ground)**
_(Terminal K)_

```bash
source ~/arc_test_kit/env.sh && ros2 run arc_hw_bridge motor_bench check
```

The sketch answers and the E-stop reads closed. **E-STOP OPEN** means D4 isn't reaching GND through the E-stop.

- [ ] **Install the new shopping UI (only if you want the web UI)**
_(Terminal K)_

```bash
cd ~ && unzip -o ~/Downloads/arc_ui_bridge.zip -d ~/arc_ui_bridge_pkg && ~/arc_ui_bridge_pkg/arc_ui_bridge/install_into_slam_ws.sh
```

Builds only `arc_ui_bridge` and runs its tests. Nothing else in `slam_ws` changes.

- [ ] **Backup videos on the desktop**
_(Files)_

Put `final_1_1.webm` (full shopping run) and `arch_w_perception_1.webm` (perception run) on the desktop. If anything fails on stage, you play these instead of debugging live.

- [ ] **Charge and pack**
_(Bag)_

- Laptop charger, Manav's iPhone and its cable (the hotspot must stay on), the Pi's own power supply.

- Uno USB cable, motor battery (charged), the stand that keeps the wheels off the ground, the E-stop.

- The webcam, plugged into the Pi.

## Network first

Today Wi-Fi stays **on**, on Manav's hotspot, because the Pi sends the webcam data over it. Join before starting anything and don't touch Wi-Fi until you shut down. ROS itself stays on the laptop, so this is safe.

- [ ] **Hotspot on, both machines on it**
_(iPhone + laptop)_

On the iPhone: Personal Hotspot on, **Maximise Compatibility** on, phone plugged in and close to the Pi. Join **Manav's iPhone** on the laptop. Power the Pi; it joins by itself.

- [ ] **Loopback multicast on, daemon reset, Pi answers**
_(Terminal K)_

```bash
sudo ip link set lo multicast on && ros2 daemon stop && hostname -I && ping -c 3 172.20.10.2
```

One address should be **172.20.10.x** (the laptop on the hotspot) and the ping gets replies. Don't run the usual `nmcli radio wifi off` today.

## Everything with one command

This runs your normal bring-up (Gazebo, ICP odometry, RTAB-Map, auto_map, Dijkstra, your TEB-like controller, BT and motion gate) on the progress-check file, then opens the HIL windows.

- [ ] **Bring-up plus HIL windows**
_(Terminal K)_≈ 15 min · don't touch the robot while auto_map drives

```bash
cd ~/arc_test_kit && source env.sh && TEB_YAML=$HOME/arc_test_kit/configs/teb_progress.yaml ~/arc_pi_udp/laptop/arc_demo.sh up
```

It checks the hotspot, Pi, Uno port and `arc_hw_bridge` first. Look for **progress_checker.required_movement_radius = 0.25** in the printed limits, then **all windows started**. B6 is not used: the webcam drives the BT.

- [ ] **Start perception on the Pi**
_(ARC_pi window)_

Type the Pi's password. Then **ARC_relay** prints **receiving from the Pi at 172.20.10.2** and **ARC_view** shows the camera.

- [ ] **RViz with the store labels**
_(Terminal R)_

```bash
source ~/arc_test_kit/env.sh && (python3 ~/arc_compact_store_kit/ros_tools/item_markers.py &) && rviz2 -d ~/arc_compact_store_kit/run/arc_nav.rviz --ros-args -p use_sim_time:=true
```

Map, scan, robot, aisle and item labels, and the yellow `/planned_path` once a trip starts.

- [ ] **Pose check**
_(Terminal K)_

```bash
source ~/arc_test_kit/env.sh && python3 ~/arc_compact_store_kit/ros_tools/check_frames.py --model arc --samples 1
```

**PASS**: the map pose is within 0.15 m and 5° of Gazebo's truth. Good to show live.

- [ ] **Optional: web UI instead of the terminal list**
_(ARC_shopping, then Terminal W)_

Only one goal source at a time. Press **Ctrl+C in ARC_shopping** first, then:

```bash
source ~/arc_test_kit/env.sh && ros2 launch arc_ui_bridge arc_ui.launch.py
```

Open `http://localhost:8080`. "Connected to ARC" means it's live. To hand an examiner a phone on the hotspot, add `host:=0.0.0.0` and open `http://<laptop 172.20.10.x>:8080` on the phone.

- [ ] **Arrange the screen**
_(Desktop)_

Left half: RViz (Gazebo minimised to save CPU, bring it up when you show the store). Right half: **ARC_view** on top, **ARC_chain** under it, **ARC_buttons** and **ARC_shopping** at the bottom. Leave the T and B windows on a second workspace.

## The run, about 10 minutes

One sentence to open: the store, lidar and robot body are simulated; the webcam, Pi, Uno and motors are real and in the loop.

- [ ] **1. The store and the map**
_(Gazebo, RViz)_1 min

Show the Gazebo store, then RViz: the RTAB-Map map (built by auto_map, now locked), the lidar scan, the aisle labels. Run the pose check (step on the previous tab) and show **PASS**.

ICP lidar odometry plus RTAB-Map, then localisation-only mode, so the map can't drift during a trip.

- [ ] **2. Perception: lock onto the shopper**
_(ARC_view, ARC_buttons)_2 min

Stand alone in front of the webcam. Click into **ARC_buttons** and press **e**: ARC_view shows **LOCKED**. Press **l** (lead me): the B5 window shows **-> LEAD**.

YOLOv8n and ByteTrack run on the Pi; only two flags and a small preview come to the laptop.

- [ ] **3. The shopping list and the route**
_(ARC_shopping (or the web UI))_2 min

```bash
pasta, milk, tea_coffee, fruits
```

At **Add items (comma separated):** paste the list, then answer **n** to "Add more items?" and **y** to "Confirm shopping list?". At the route choice, show both lengths and pick **1** (optimised). RViz draws the Dijkstra path; the cart drives, and the trip always ends at the checkout. Answer **y** at "Continue to next item?" after each stop.

Held–Karp picks the visiting order from Dijkstra path lengths through the aisles, not straight-line distance.

- [ ] **4. Hardware in the loop**
_(The rig, ARC_chain)_2 min

Point at the real wheels turning with the Gazebo cart. **ARC_chain** shows one live line: webcam, flags, BT, `cmd_vel`, PWM and measured wheel speed.

The same /cmd_vel drives both. The sketch caps PWM at 60, so the real wheels turn slower than the simulated cart.

- [ ] **5. Safety: walk away, then pull the Pi**
_(Webcam, ARC_relay)_2 min

- Step out of view. Within a few seconds: lock lost, BT **WAIT**, `cmd_vel` 0, wheels stop, cart stops in Gazebo.

- Come back, press **e** then **l**: it carries on.

- Press Ctrl+C in **ARC_pi** (or pull the Pi's power). After 1.5 s **ARC_relay** prints **holding the BT in WAIT**.

- Physical E-stop cuts motor power directly.

Every layer fails safe: lost person, silent Pi, lost /cmd_vel at the Uno (watchdog), and the hardware E-stop.

- [ ] **6. The numbers from this run**
_(Terminal K)_1 min, after Shut down

Shut down (last tab) prints the chain log path. The plot shows webcam flags, BT mode, `cmd_vel` and commanded vs measured wheel speed, with the reaction time of each stage.

```bash
python3 ~/arc_pi_udp/laptop/plot_hil_timeline.py $(ls -t ~/arc_hil_logs/chain_*.csv | head -1)
```

**Don't mix goal sources.** Either ARC_shopping or the web UI, never both. Never teleop while the BT stack is up: the motion gate owns `/cmd_vel`.

## Every terminal, and what it is

The first two groups open by themselves. You open K, R and (optionally) W.

| Window | What it is | During the demo | 

| Opened by bring-up (your nav stack) | 

| T1_gazebo | Gazebo, compact store | Show once, then minimise | 

| T2_robot_description | Robot state publisher | Leave | 

| T3_spawn_bridge | Spawns `arc`, ROS–Gazebo bridge | Leave | 

| T5a_icp_odometry | ICP lidar odometry | Leave | 

| T5b_rtabmap | RTAB-Map, locked to localisation | Leave | 

| B1_controller | TEB-like controller, progress-check file | Leave | 

| B2_lifecycle | Activates the controller | Leave | 

| B3_dijkstra | Dijkstra planner | Leave | 

| B4_path_to_teb | Path → FollowPath | Leave | 

| B5_bt_stack | BT + motion gate | Show the `-> LEAD` / `-> WAIT` lines | 

| Opened by arc_demo.sh (HIL + perception) | 

| ARC_relay | Pi UDP → `/person_moving`, `/person_confused`; holds WAIT if the Pi goes quiet | Show "holding the BT in WAIT" | 

| ARC_motors | `arc_hw_bridge`: `/cmd_vel` → Uno → wheels | Leave | 

| ARC_chain | One live line: webcam, flags, BT, `cmd_vel`, PWM, wheels | Keep visible | 

| ARC_recorder | Chain log to `~/arc_hil_logs/` | Leave | 

| ARC_view | The Pi's annotated camera view | Keep visible | 

| ARC_buttons | Shopper keys: **e** enrol, **l** lead me, **r** reset | Click into it before pressing keys | 

| ARC_pi | ssh to the Pi, running `run_on_pi.sh` | Password; Ctrl+C for the safety demo | 

| ARC_shopping | Shopping Manager | Type the list here | 

| You open these | 

| Terminal K | Commands: bring-up, pose check, shut down | Pose check | 

| Terminal R | RViz and store labels | Keep visible | 

| Terminal W | Web UI bridge (optional, instead of ARC_shopping) | Browser at localhost:8080 | 

Every terminal you open yourself starts with `source ~/arc_test_kit/env.sh`, or it won't see the nodes. Logs for the HIL windows are in `~/arc_logs/demo_<time>/`.

## If something breaks on stage

Rule of thumb: if a fix takes more than a minute, play the backup video and keep talking.

| What you see | Do this | 

| ARC_relay: waiting for the Pi | Pi password not typed yet, the Pi isn't on the hotspot, or the firewall: `sudo ufw allow 5600/udp` | 

| e doesn't lock | Be the only person in view at that moment, then press **e** again in ARC_buttons | 

| Cart never moves | BT not in LEAD (press **l**), ARC_motors not running, or a B6 window is open | 

| Robot stops mid-trip | Restart only the nav nodes, then start the Shopping Manager again: `cd ~/arc_test_kit && source env.sh && PERSON_INPUT=camera TEB_YAML=$PWD/configs/teb_progress.yaml ./bringup.sh nav` | 

| ARC_motors: E-STOP OPEN | The E-stop is pressed or D4 isn't wired to GND through it | 

| ARC_motors: Uno NOT connected | Replug the Uno; check `ls /dev/ttyACM*` matches `port:` in `arc_hw.yaml` | 

| Pi very slow | In ARC_pi: Ctrl+C, then `IMGSZ=256 ~/arc_pi_udp/pi/run_on_pi.sh <laptop 172.20.10.x> 0` | 

| Pi or hotspot dead | Keyboard instead of webcam: Ctrl+C in ARC_relay, then in a new terminal `source ~/arc_test_kit/env.sh && ros2 run arc_behavior_tree person_state_input` and type **3** (LEAD). Motors still follow `/cmd_vel`. Then play `arch_w_perception_1.webm` for the perception part | 

| Gazebo, ICP or RTAB-Map died | The map is gone and a restart is 15 min: play `final_1_1.webm` | 

| arc_demo.sh: windows from an earlier run | `~/arc_pi_udp/laptop/arc_demo.sh stop`, then start again | 

| Web UI refuses the trip | ARC_shopping is still running: Ctrl+C in it | 

**Never** run the old `RESTORE.sh` in `arc_backups/compact_store_20261012_133112`.

## Shut down, motors first

- [ ] **Stop the Pi**
_(ARC_pi window)_

Ctrl+C. It sends "wait" to the laptop before exiting.

- [ ] **Stop everything on the laptop**
_(Terminal K)_

```bash
~/arc_pi_udp/laptop/arc_demo.sh stop
```

Motors stop first (STOP to the Uno), the chain log is saved, then `bringup.sh stop`. It prints the chain log and motor log paths and their plot commands. Close RViz and the web UI with Ctrl+C.

- [ ] **Plot the motor log too**
_(Terminal K)_

```bash
python3 ~/arc_hw/plot_hil.py $(ls -t ~/arc_hw_logs/hil_*.csv | head -1)
```

Commanded vs measured wheel speed, shaded by BT mode, and the median speed error. Keep both plots for the report.

Clear ticks on this tab

Live version (pinned): https://claude.ai/artifact/4iGoz5nsPchAXVt8uA6uLT
