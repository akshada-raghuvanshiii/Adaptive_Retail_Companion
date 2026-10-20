# ARC on a Raspberry Pi: webcam + motors with the simulated stack

The laptop runs the simulation and the ARC stack (Gazebo, RTAB-Map, Dijkstra, TEB-like controller, BT, motion gate), exactly as in the tests.
The Pi carries the two real-world parts:

| Where | What | Topics |
|---|---|---|
| **Pi**, webcam | `perception_bridge.py` (YOLOv8n + ByteTrack + target lock), headless | publishes `/person_moving`, `/person_confused`, `/arc/target_lock`, `/arc/perception/debug/compressed` |
| **Pi**, USB to the Uno | `arc_hw/bridge_node.py` | subscribes `/cmd_vel`; publishes `/arc_hw/cmd_wheels`, `/arc_hw/wheels` |
| **Laptop** | stack + BT, unchanged | the BT reads the two webcam flags, the motion gate publishes `/cmd_vel` |
| **Laptop** | `flag_watchdog.py`, `hil_logger.py`, `arc_ui.py`, `view_debug.py` | safety net, recorder, shopper buttons, camera view |

The chain you can show is webcam (Pi) → BT mode (laptop) → `/cmd_vel` (laptop) → real wheels (Pi + Uno).
The cart in Gazebo and the real wheels move together, and both stop when the shopper leaves the camera's view.

**Not changed:** anything in `~/slam_ws`. That covers the nav stack, RTAB-Map, Dijkstra, the controller, the BT, TF, and `arc_person_follower`. YOLO stays in its own venv on the Pi.

**Link:** both machines join the same **phone hotspot** (`LINK=hotspot` in `hil.conf`, the default).
- The phone hands out the addresses, so none are typed in. Each machine uses its current Wi-Fi address, and finds the other by name: the Pi is `arc-pi.local`, and the laptop is `<its hostname>.local`.
- ROS 2 is limited to loopback, shared memory and that one Wi-Fi address on each machine.
- Discovery uses multicast, plus direct (unicast) messages to the other machine, because some phones drop multicast between hotspot clients.
- A direct Ethernet cable still works: set `LINK=cable` (static 192.168.50.1 / .2).

**Keep the hotspot steady during a session.** Join it on both machines before `hil_mode.sh on`, and don't switch networks until `bringup.sh stop`. If the phone gives a new address after a reconnect, `bringup.sh` stops with a message telling you to run `hil_mode.sh on` again.

## What you need

- **Board:** Raspberry Pi 4 (4 or 8 GB) or Pi 5. A Pi 3 has too little memory for PyTorch.
- **Storage and power:** a 32 GB microSD card and the official power supply. Under-voltage slows the Pi down.
- **Network:** a phone with a Wi-Fi hotspot. Turn off any "client isolation" or "AP isolation" option, and prefer 2.4 GHz if the Pi struggles to connect. Mobile data isn't needed during a run, only for the Pi's setup downloads.
- **Camera:** a USB webcam (any UVC camera; MJPG support helps).
- **Motors:** the Uno running `arc_motor_bridge.ino` on the Pi's USB, with the motor battery and driver as in `arc_hw/README.md`.
- **Safety:** the wheels must be **off the ground** for every HIL run, and the physical E-stop must cut motor power.

## Steps (in order)

`L$` means run it on the laptop and `P$` on the Pi (over ssh). Each script explains itself in the comment at its top.

**0. Laptop: unzip and install the updates.** This is safe while a test run is going. Without HIL mode switched on, `env.sh` behaves exactly as before.
```
L$ unzip arc_pi_kit.zip -d ~ && ~/arc_pi/install_updates.sh
```
It updates `perception_bridge.py`, `bridge_node.py`, `env.sh` and `bringup.sh`, keeping backups, then runs the four offline test suites.

**1. SD card.**
- In Raspberry Pi Imager, choose *Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)*. ROS 2 Jazzy needs Ubuntu 24.04; Raspberry Pi OS has no Jazzy packages.
- In the Imager settings, set:
  - hostname `arc-pi`;
  - user `arc` (or change `PI_USER` in `hil.conf`) and a password;
  - **Wi-Fi: your phone hotspot's name and password;**
  - **enable SSH**.
- (Cable only: after flashing, run `L$ ~/arc_pi/laptop/prep_sd.sh` to put the static cable address on the card. With the hotspot it has nothing to do.)

**2. First boot.** Turn the hotspot on, join it with the laptop, then power the Pi. First boot takes 2–5 minutes.
```
L$ ~/arc_pi/laptop/net_laptop.sh up          # laptop on the hotspot? Pi found and answering?
L$ ssh arc@arc-pi.local                      # check you can log in, then exit
L$ ssh-keygen -t ed25519 ; ssh-copy-id arc@arc-pi.local   # optional: no password prompts
```
`arc-pi.local` works once the Pi announces its name. If the Pi image doesn't do that yet (`pi_setup.sh mdns` in step 3 adds it), then:
- read the Pi's address on the phone (hotspot → connected devices);
- put `PI_HOST=<that address>` in `hil.conf`, and use that address for `ssh`.

**3. Copy the code to the Pi, then set up the Pi.**
```
L$ ~/arc_pi/laptop/deploy.sh
P$ ~/arc_pi/pi/pi_setup.sh                   # each step says what it installs and why, and waits for y
P$ exit ; ssh arc@arc-pi.local               # log in again so the dialout/video groups apply
P$ ~/arc_pi/pi/pi_check.sh env net serial camera venv thermal
```
`pi_setup.sh` installs these, each only after your y:
- `ros-jazzy-ros-base`;
- `python3-serial`, `python3-venv`, `v4l-utils`, `rsync`;
- a YOLO venv (`ultralytics`, `lap`; ncnn is optional);
- a boot service that turns on loopback multicast;
- `avahi-daemon` and `libnss-mdns`, so the two machines find each other by name on the hotspot;
- the hotspot Wi-Fi, if it wasn't set in Imager (cable mode: the static IP instead).

`deploy.sh` also tells the Pi the laptop's name. Run it again if you change laptops.

**4. Motor bench tests on the Pi, without ROS.** The Uno is plugged into the Pi, so `bench.py` runs there. Wheels off the ground.
```
P$ cd ~/arc_pi_run/arc_hw
P$ python3 bench.py monitor   --wheel <d> --track <w> --ppr <magnets>
P$ python3 bench.py watchdog  --wheel <d> --track <w> --ppr <magnets>     # must print PASS
P$ python3 bench.py calibrate --wheel <d> --track <w> --ppr <magnets>     # prints pwm_min and ff
P$ python3 bench.py step      --wheel <d> --track <w> --ppr <magnets> --pwm-min .. --ff ..
```
Put the measured values into `~/arc_pi/hil.conf` **on the laptop**: `WHEEL_D`, `TRACK`, `PPR`, `PWM_MIN` and `FF`. Then run `L$ ~/arc_pi/laptop/deploy.sh` again.

**5. Measure perception speed on the Pi.** These are real Pi numbers for the report.
```
P$ ~/arc_pi_venv/bin/python ~/arc_pi/pi/pi_perf.py                  # yolov8n at 320 and 416, live webcam
P$ ~/arc_pi_venv/bin/python ~/arc_pi/pi/pi_perf.py --export-ncnn --imgsz 320   # optional, needs ncnn
```
Choose `IMGSZ` (and optionally `PI_MODEL`) in `hil.conf` from the results. If the Pi manages less than about 2 fps, use **stream mode** in step 6: the Pi sends frames and the laptop runs YOLO.

**6. A HIL session.** Do this after Phase 2 finishes, because it restarts the stack.
```
L$ ~/arc_pi/laptop/net_laptop.sh up          # both on the hotspot, Pi answering
L$ ~/arc_pi/laptop/hil_mode.sh on            # profile with this laptop's hotspot address and the Pi's
L$ ~/arc_test_kit/bringup.sh stop
L$ PERSON_INPUT=camera ~/arc_test_kit/bringup.sh all     # B6 (keyboard person state) is not started
P$ ~/arc_pi/pi/pi_up.sh all                  # motors + webcam bridge, in the background
L$ ~/arc_pi/laptop/hil_laptop.sh check       # sees /arc_hw/wheels and /arc/target_lock from the Pi?
L$ ~/arc_pi/laptop/hil_laptop.sh up          # watchdog + recorder
L$ source ~/arc_test_kit/env.sh && python3 ~/arc_pi/laptop/arc_ui.py        # terminal 1: shopper buttons
L$ source ~/arc_test_kit/env.sh && python3 ~/arc_pi/laptop/view_debug.py    # terminal 2: Pi camera view
```
**Stream mode** (YOLO on the laptop) differs only in two lines:
- `P$ ~/arc_pi/pi/pi_up.sh motors && ~/arc_pi/pi/pi_up.sh stream`
- `L$ ~/arc_perception/.venv/bin/python ~/arc-perception/ros/perception_bridge.py --headless --camera ros:/arc/camera/image/compressed --debug-image-hz 2`

**7. Demo.** It takes about 2 minutes. Keep `arc_ui.py` focused, so its keys are picked up.

| Do | Expect |
|---|---|
| Stand alone in front of the webcam, press **e** | The status line shows `lock LOCKED`; the BT stays in WAIT |
| Press **l** ("lead me") | `confused=True` → the BT switches to LEAD |
| Give a goal: an item in the Shopping Manager, or a 2D goal in RViz | `/cmd_vel` > 0; the Gazebo cart and the real wheels move at LEAD speed |
| Step out of view | Within a few seconds: LOST → the BT goes to WAIT → `/cmd_vel` drops to 0 → the wheels stop |
| Come back, press **e** then **l** | It resumes |
| Turn the Pi's Wi-Fi off, or walk it out of hotspot range (optional) | The Uno stops the wheels by itself, and the laptop watchdog holds the BT in WAIT |

**LEAD is the dependable mode for this demo.** FOLLOW also needs a person goal (`/person_pose`), and one webcam gives a direction but no distance (see `arc-perception/ros/README.md`).

**8. Stop, then plot.**
```
P$ ~/arc_pi/pi/pi_up.sh down                 # bridges publish "wait" and send the Uno a stop before exiting
L$ ~/arc_pi/laptop/hil_laptop.sh down        # prints the log path
L$ python3 ~/arc_pi/laptop/plot_hil_timeline.py ~/arc_hil_logs/chain_<time>.csv
L$ ~/arc_pi/laptop/hil_mode.sh off           # back to laptop-only for simulation tests
```
The figure has four panels: webcam flags, BT mode, `/cmd_vel`, and commanded vs measured wheel speed.
The printed table gives the reaction time of each stage after every webcam stop/go change.
To copy the Pi-side wheel log to the laptop: `scp 'arc@arc-pi.local:arc_pi_run/hil_logs/*.csv' .`, then `plot_hil.py` from `arc_hw`.

## Safety layers

| If this fails | What stops the cart | Where |
|---|---|---|
| The shopper leaves the view or the lock is unsure | The flags go false/false → BT WaitForPerson → the motion gate sends 0 | Pi bridge (fail-safe mapping) |
| The Pi bridge crashes, the Pi loses power, the hotspot drops | No heartbeat for 3 s → watchdog publishes false/false | `flag_watchdog.py` on the laptop |
| `/cmd_vel` stops reaching the Pi | Wheels set to 0 after 0.5 s | `bridge_node.py` |
| The Pi or the motor bridge dies | The Uno stops after 0.5 s without a command | firmware watchdog (`bench.py watchdog` PASS) |
| Anything else | The **physical E-stop** cuts motor power | hardware |

Perception never publishes `/emergency_stop` or `/person_state`.

## Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| `bringup.sh` says "no address …" | The hotspot gave the laptop a new address, or it isn't joined → join it, `hil_mode.sh on`, then bringup again. The link must be up **before** nodes start. |
| `net_laptop.sh` says the Pi doesn't answer ping | The phone isolates hotspot clients → turn off "client isolation" / "AP isolation", or use another phone or the cable |
| `cannot find arc-pi.local` | The Pi is still booting, or is on another network → check the phone's connected devices; set `PI_HOST=<IP>` in `hil.conf` |
| The Pi can't find the laptop (`pi_check.sh net`) | Run `deploy.sh` on the laptop again (it tells the Pi the laptop's name), or `pi_setup.sh mdns` on the Pi |
| Topics appear, then drop out mid-run | Weak hotspot signal, or the phone's power saving → keep the phone close and plugged in; don't let it sleep the hotspot |
| `hil_laptop.sh check` doesn't see the Pi's topics | `hil_mode.sh` was not on when the stack started → `bringup.sh stop`, then start again. Or the Pi is missing loopback multicast → `pi_check.sh net` |
| The Pi sees no `/cmd_vel` | Same as above; also `ros2 daemon stop` on both machines |
| `pi_up.sh perception` exits at once | `pi_up.sh logs perception`. Usually the camera index (`CAMERA` in `hil.conf`) or a missing venv |
| Very low fps on the Pi | `IMGSZ=320`, the ncnn model (`PI_MODEL`), or stream mode. Check `pi_check.sh thermal` for throttling |
| The status line stays `IDLE` | Nobody is enrolled → press **e** with only one person in view |
| Wheels don't turn but `/arc_hw/cmd_wheels` changes | Motor power, the driver enable jumpers, or `PWM_MIN` too low (see `arc_hw/README.md`) |
| A simulation test after HIL can't see `/clock` | `hil_mode.sh off`, `bringup.sh stop`, start again |

## What has been tested, and what hasn't

**Tested here, in the cloud sandbox (x86, no ROS available, no Pi):**
- **Bridge and kit logic:** 7 new bridge tests and 14 kit tests, with ROS stubbed. They cover:
  - remote enrol and lead-me;
  - the debug JPEG;
  - frames from a ROS topic, and wait when they stop;
  - SIGTERM clean stop;
  - the watchdog decision;
  - logger rows;
  - reaction-time extraction on a synthetic chain;
  - the `camera_pub` rate limit;
  - `env.sh` mode switching;
  - hotspot mode with stand-in network tools: the profile allows only loopback and the Wi-Fi address, and adds the Pi as a direct peer; it falls back to multicast when the Pi isn't found; `bringup.sh` stops if the hotspot address changed; the Pi finds the laptop from `hil.peer`; and client isolation is reported;
  - `bringup.sh` refusing to start without the link;
  - `prep_sd.sh` keeping the Wi-Fi settings;
  - `install_updates.sh` on v18 copies;
  - `shellcheck` on every script.
- **Real YOLOv8n + ByteTrack through the modified bridge** (sandbox CPU, ROS stubbed), on a store-aisle clip:
  - remote enrol locked the shopper;
  - lead-me switched the flags to LEAD;
  - shutdown published wait.
- **`pi_perf.py`** runs end to end. Its sandbox timings are x86 numbers and say nothing about a Pi.

**Not tested: anything that needs your hardware or two real machines.**
- **The Fast DDS profile over a real phone hotspot.** Its XML is well formed, and the interface allowlist is the same mechanism as the working `fastdds_localhost.xml`. The initial-peers part follows the Fast DDS documentation, but I haven't been able to load it into a real Fast DDS here. `hil_laptop.sh check` and `pi_check.sh laptop` are the first real test.
- **Your phone's hotspot behaviour:** whether it passes multicast and lets clients reach each other varies by phone.
- **`pi_setup.sh` on a real Pi.**
- **The camera, serial and Uno.**
- **Any Pi timing.**

## Data this gives you for the paper

- **Real Pi perception timing:** `pi_perf.py` → `summary.json`, with median and p95 ms and fps per model and input size, plus temperature.
- **End-to-end reaction times:** `plot_hil_timeline.py` → `*_reactions.csv`. Measured on one clock, it gives the time from a webcam flag change to the BT mode change, to `/cmd_vel`, and to the wheels stopping or starting.
- **Commanded vs measured wheel speed** (Hall sensors), from `plot_hil.py` and panel 4 of the timeline. This also covers the step response and the watchdog stop time, from `bench.py`.
