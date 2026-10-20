# ARC webcam on the Pi, without ROS on the Pi

The Pi runs **Raspberry Pi OS 13 (trixie)**, which has no ROS 2 Jazzy packages. So:

```
Pi (no ROS)                                         your laptop (ROS 2 Jazzy, ARC stack)
webcam -> YOLOv8n + ByteTrack + target lock          perception_relay.py
   perception_bridge.py --udp  ── UDP 5600 ───────►    /person_moving, /person_confused  ──► BT ──► motion gate ──► /cmd_vel ──► motors
                               ◄── UDP 5601 ───────    /arc/ui/command (enrol, lead me...) ◄── arc_ui.py
                               ── debug JPEG ──────►    /arc/perception/debug/compressed   ──► view_debug.py
```

- **The Pi runs the same tested `perception_bridge.py`.** `ros/netshim.py` replaces its ROS layer with plain UDP; the perception logic is unchanged.
- **The relay holds the BT in WAIT if the Pi goes quiet for 1.5 s,** whether the Pi is off, Wi-Fi drops or the bridge crashes. That closes the "BT keeps old flags" gap without changing the BT.
- **Nothing in Manav's setup changes.** His venv is reused, and every replaced file goes to `~/backups/` first.
- **Nothing in `~/slam_ws` changes.**

Both machines must be on **Manav's iPhone hotspot** (the Pi is already on it, at 172.20.10.2).

## Once

**Laptop:**
1. Join *Manav's iPhone* hotspot.
2. Unpack the kit, then copy it to the Pi. This asks for the Pi's password:
   ```bash
   cd ~ && unzip -o ~/Downloads/arc_pi_udp_kit.zip -d ~ && scp -r ~/arc_pi_udp manav@172.20.10.2:~/
   ```

**Pi** (`ssh manav@172.20.10.2`):
```bash
~/arc_pi_udp/pi/install_on_pi.sh      # copies the code, checks the venv (prints a pip command if something is missing)
~/arc_pi_udp/pi/find_camera.sh        # prints which index is the USB webcam, e.g. CAMERA=0
```

## Everything together, with one command (laptop)

Webcam on the Pi → BT → `/cmd_vel` → real motors, plus the Gazebo cart. Stay on Manav's hotspot, then:

```bash
~/arc_pi_udp/laptop/arc_demo.sh up
```
1. **It checks first:**
   - you're on the hotspot;
   - the Pi answers;
   - the Uno port in `arc_hw.yaml` exists;
   - `arc_hw_bridge` is built.
2. **It runs `bringup.sh all`** (about 15 min) and closes B6.
3. **It opens 8 windows:**

   | Window | What it is |
   |---|---|
   | `ARC_relay` | Pi → BT flags |
   | `ARC_motors` | `/cmd_vel` → wheels |
   | `ARC_chain` | One live line: webcam, flags, BT, `cmd_vel`, PWM and wheels |
   | `ARC_recorder` | Chain log for the timeline figure |
   | `ARC_view` | The Pi's camera view |
   | `ARC_buttons` | **e** enrol, **l** lead me |
   | `ARC_pi` | ssh to the Pi; type the Pi's password here |
   | `ARC_shopping` | Shopping Manager; enter the list here |

Stop everything, motors first, with `~/arc_pi_udp/laptop/arc_demo.sh stop`. It prints the commands that plot the chain log and the motor log.

If the stack is already up, use `arc_demo.sh up --no-bringup`.

The steps below do the same thing by hand.

## Each session (by hand)

**Laptop**, in order:
1. Join the hotspot **before** starting anything, and don't change Wi-Fi until you stop.
2. Bring up the stack:
   ```bash
   cd ~/arc_test_kit && source env.sh && ./bringup.sh all
   ```
   **Close the B6 `person_state_input` window** when it opens. It publishes the same flags, and `run_relay.sh` refuses to start while it runs.
3. Each of these needs its own terminal:

   | Terminal | Command |
   |---|---|
   | Relay | `~/arc_pi_udp/laptop/run_relay.sh` → prints this laptop's hotspot IP (172.20.10.x) |
   | Motors | `source ~/arc_test_kit/env.sh && ros2 launch arc_hw_bridge arc_hw.launch.py params_file:=$HOME/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml` |
   | Buttons | `source ~/arc_test_kit/env.sh && python3 ~/arc_pi_udp/laptop/arc_ui.py` → **e** enrol, **l** lead me, **r** reset |
   | Camera view | `source ~/arc_test_kit/env.sh && ~/arc_perception/.venv/bin/python ~/arc_pi_udp/laptop/view_debug.py` |
   | Shopping | `source ~/arc_test_kit/env.sh && ros2 run arc_planner shopping_manager` |

**Pi:**
```bash
~/arc_pi_udp/pi/run_on_pi.sh <laptop IP from the relay window> <camera index>
```
The relay should print `receiving from the Pi at 172.20.10.2`.

**Demo:**
1. Stand alone in view and press **e** in the Buttons window. The camera view shows `LOCKED`.
2. Press **l**. B5 shows `-> LEAD`.
3. Pick an item. The Gazebo cart and the real wheels drive.
4. Step out of view. A few seconds later both stop.
5. Pull the Pi's power, or stop `run_on_pi.sh`. After 1.5 s the relay prints `holding the BT in WAIT`, and the cart stops.

If **e** doesn't lock, you weren't in view at that instant. Press it again.

**Stop:**
1. Press Ctrl+C on the Pi first. It sends "wait".
2. Then press Ctrl+C in the relay.
3. Then run `~/arc_test_kit/bringup.sh stop`.

## Pi speed for the paper

`ros2 topic echo /arc/target_lock` shows `fps` and `proc_ms`, measured **on the Pi**. RViz shows the Pi's fps next to the status text. If the cart reacts too slowly, try `IMGSZ=256 ~/arc_pi_udp/pi/run_on_pi.sh ...`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| The relay keeps saying `waiting for the Pi` | Wrong laptop IP given to `run_on_pi.sh`, the laptop isn't on the hotspot, or the firewall is on (`sudo ufw allow 5600/udp`) |
| `'enrol' not sent: no data from the Pi yet` | Start `run_on_pi.sh` first; the relay learns the Pi's address from its data |
| `cannot read camera 0` on the Pi | Use the index from `find_camera.sh` |
| The cart never moves | B6 still open, BT not in LEAD (press **l**), or the motors window isn't running |
| Very slow on the Pi | `IMGSZ=256`; check `vcgencmd measure_temp` and use the Pi's proper power supply |

## Tested, and not yet tested

**Tested here, in the cloud sandbox (x86, no Pi):**
- **The real bridge in `--udp` mode, with ROS removed from the environment:** YOLOv8n + ByteTrack ran on a store-aisle clip. The relay's enrol and lead-me commands came back over UDP, and `confused=true` arrived along with the debug JPEG. SIGTERM sent `SHUTDOWN` and wait. This passed 3 runs out of 3.
- **The relay:** stale data holds WAIT, packets from other addresses are ignored, and commands go back to the Pi.
- **The Pi install script,** run on a stand-in home folder.
- **The full perception suite:** 63 tests.

**Not tested yet:**
- the real Pi, its camera and its speed;
- the iPhone hotspot passing UDP between its clients;
- the relay with real ROS (`rclpy`) on your laptop.

Your first session is that test.
