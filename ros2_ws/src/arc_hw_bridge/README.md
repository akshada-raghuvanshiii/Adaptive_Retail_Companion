# arc_hw_bridge: ARC motors in slam_ws

A ROS 2 package that connects ARC's `/cmd_vel` to the real wheels through **your own Uno sketch**: SmartElex 15S drivers, `M,<left>,<right>` PWM commands and `ENC,<l>,<r>` pulse reports. The sketch is kept unchanged; a reference copy is in `firmware/arc_cart_motor/`.

```
ARC stack (Gazebo, RTAB-Map, Dijkstra, TEB-like controller, BT, motion gate)
        │  /cmd_vel (TwistStamped)
        ▼
arc_hw_bridge ── M,<pwmL>,<pwmR> at 20 Hz ──► Uno ──► SmartElex 15S ──► motors
        ▲                                      │
        └──────── ENC,<l>,<r> once per second ─┘ (motor encoders, channel A)
   publishes /arc_hw/wheels, /arc_hw/cmd_wheels, /arc_hw/connected, /arc_hw/estop_ok
```

**What changes in slam_ws:** one new package. No existing package is edited: not the nav stack, RTAB-Map, Dijkstra, your controller, the BT, TF or `arc_person_follower`. The bridge publishes no `/odom` and no TF, because Gazebo owns those in the simulation.

**Safety:** keep the wheels off the ground. The sketch's own E-stop input (D4) and its 300 ms watchdog stay in charge.

## Install (once)

```bash
cd ~ && unzip -o ~/Downloads/arc_hw_bridge.zip -d ~/arc_hw_bridge_pkg && ~/arc_hw_bridge_pkg/arc_hw_bridge/install_into_slam_ws.sh
```
The script:
- copies the package to `~/slam_ws/src/arc_hw_bridge`, backing up an older copy first;
- runs `colcon build --packages-select arc_hw_bridge`, so nothing else is rebuilt;
- runs the offline tests.

It installs no packages. If `python3-serial` is missing, or you're not in the `dialout` group, it prints the command for you to run.

## Use

Every terminal needs: `source ~/arc_test_kit/env.sh`. That loads `~/slam_ws/install/setup.bash` too.

**1. Set the basics** in `~/slam_ws/src/arc_hw_bridge/config/arc_hw.yaml`. `motor_bench` reads this file too, so you don't repeat them on the command line.
- `port`: from `ls /dev/ttyACM*`.
- `wheel_diameter` and `track_width`: measured, in m.
- `pulses_per_rev`: per **wheel** turn, which is the encoder PPR × the gear ratio. Count it: run `monitor` and turn one wheel exactly once by hand.

**2. Bench test, without ROS, wheels off the ground:**
```bash
ros2 run arc_hw_bridge motor_bench check       # sketch answers? E-stop closed? encoders quiet?
ros2 run arc_hw_bridge motor_bench monitor     # turn a wheel by hand: its count rises
ros2 run arc_hw_bridge motor_bench watchdog    # must print PASS
ros2 run arc_hw_bridge motor_bench calibrate   # prints pwm_start, pwm_min, ff for EACH wheel
ros2 run arc_hw_bridge motor_bench step        # speed steps using the values in the YAML
```
**Put the six calibration lines in the YAML,** then rebuild: `cd ~/slam_ws && colcon build --packages-select arc_hw_bridge`.

**3. With the simulation.** First `~/arc_test_kit/bringup.sh all`, then:
```bash
ros2 launch arc_hw_bridge arc_hw.launch.py
```
Give a shopping item or an RViz goal, and switch the BT mode in B6. The Gazebo cart and the real wheels move together. Every 5 s the status line shows the `/cmd_vel` it received and the PWM it sent.

**4. Plot a run.** The log is `~/arc_hw_logs/hil_<time>.csv`, with the same columns `plot_hil.py` in the motor test kit reads:
```bash
python3 ~/arc_hw/plot_hil.py ~/arc_hw_logs/hil_<time>.csv
```

## What the status line tells you

| Message | Meaning → fix |
|---|---|
| `Uno OK (… OK, … ENC)` | Working |
| `E-STOP OPEN …` | The sketch answers `ERR,ESTOP`: D4 doesn't reach GND through a closed NC E-stop. With no E-stop wired, D4 floats HIGH, so the motors never run |
| `WRONG SKETCH …` | The Uno isn't running the cart sketch → upload `firmware/arc_cart_motor/arc_cart_motor.ino` |
| `Uno NOT connected` / `Permission denied` | Wrong port (`ls /dev/ttyACM* /dev/ttyUSB*`; set `port`), or you're not in `dialout` |
| `left/right encoder: N pulses … while that wheel was commanded 0` | Electrical noise, or a loose encoder connector (VCC, GND, channel A) |
| `calibrate`: `speed FELL at PWM …` | That wheel slowed although PWM rose: binding, a stall or the driver limiting current. Free the wheel, then calibrate again |

## Limits to state honestly

- **Speed control is open-loop feed-forward.** The sketch takes raw PWM and has no speed loop. The bridge sets `|PWM| = max(pwm_start, pwm_min + ff·|speed|)` **per wheel** from your calibration, because your two motors differ: the left needs much more PWM to break free. Load and battery voltage change the real speed.
- **Top speed:** the sketch caps PWM at **60 of 255**. The calibration prints the highest wheel speed that allows. If it's below what ARC asks for (LEAD is about 150 mm/s), the wheels run slower than the simulated cart. Raising `MAX_PWM` is a change to your sketch, so it's your call.
- **Measured speed is coarse.** The sketch reports pulses once per second, so the reading is a 1 s average. Resolution is one pulse per second: π × wheel diameter ÷ pulses per wheel turn. For example, 1.5 mm/s for a 0.10 m wheel at 210 pulses per turn.
- **Direction isn't sensed.** The sketch counts only channel A, so the sign comes from the command.
- **Tested here only against a simulated copy of your sketch** (same replies, 1 s ENC, 300 ms watchdog, E-stop pin), with ROS stubbed out. The first real test is `motor_bench check` on your Uno.
