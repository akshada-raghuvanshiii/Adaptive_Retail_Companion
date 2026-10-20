# ARC hardware-in-the-loop: real motors driven by the ARC stack

The simulated ARC stack (Gazebo, RTAB-Map, Dijkstra, TEB-like controller, BT, motion gate) publishes `/cmd_vel`.
`bridge_node.py` sends the same `/cmd_vel` to an Arduino Uno, which drives the real motors and reports wheel speed from Hall sensors.
The robot moves in Gazebo, and the real wheels turn with the same commands.

The stack is not changed. The bridge publishes no `/odom` and no TF, because Gazebo owns those.

**Safety:** keep the wheels **off the ground** (cart on a stand). The physical E-stop must cut motor power directly.

## Files

| File | What it is |
|---|---|
| `firmware/arc_motor_bridge/arc_motor_bridge.ino` | Uno sketch: speed control, Hall pulse counting, 0.5 s watchdog |
| `bench.py` | Tests without ROS: `monitor`, `watchdog`, `calibrate`, `step` |
| `bridge_node.py` | ROS 2 node: `/cmd_vel` → Arduino; publishes `/arc_hw/wheels` and `/arc_hw/cmd_wheels`; logs to `hil_logs/` |
| `plot_hil.py` | Plots commanded vs measured wheel speed, shaded by BT mode |
| `arc_hw/protocol.py`, `tests/` | Shared protocol and kinematics; offline tests (`python3 -m unittest discover -s tests`) |

## Wiring (Uno)

| Uno pin | L298N (default) | BTS7960 (edit `#define` at top of sketch) |
|---|---|---|
| D5 (PWM) | ENA (remove the ENA jumper) | left RPWM |
| D6 (PWM) | ENB (remove the ENB jumper) | left LPWM |
| D7, D8 | IN1, IN2 (left) | — |
| D9, D10 | IN3, IN4 (right) | right RPWM, LPWM |
| D2 | left Hall module D0 | same |
| D3 | right Hall module D0 | same |
| 5V / GND | Hall modules VCC / GND | R_EN, L_EN → 5 V |
| GND | **driver GND (common ground with the motor battery)** | same |

Notes on the wiring and sensors:
- **Motor power** comes from the battery into the driver, never from the Uno's 5 V.
- **Hall modules (49E + LM393):** glue **8–16 small magnets** evenly around each wheel hub, all with the **same pole** facing the sensor, 2–5 mm away. Turn the module's potentiometer until its LED blinks once per magnet when you turn the wheel slowly by hand.
- **`--ppr`** is the number of magnets per wheel. If the magnets are on the motor shaft before the gearbox, use magnets × gear ratio.
- **No direction sensing.** These sensors have one channel, so the sketch takes the direction from the command.
- **Wrong rotation:** if a wheel turns the wrong way, set `INVERT_L` or `INVERT_R` in the sketch.

## Steps

1. **Flash** the sketch with the Arduino IDE (board: Arduino Uno). Serial Monitor at 115200 should print `ARC motor bridge ready` and then `E ...` lines.
2. **Serial access, once:**
   - `sudo apt install python3-serial` gives Python access to the USB serial port.
   - `sudo usermod -aG dialout $USER`, then log out and back in.
3. **Measure** the wheel diameter and track width (centre to centre of the tyres). Use them in every command below.
4. Run these with the wheels off the ground (example values: 0.10 m wheel, 0.40 m track, 8 magnets):

   1. Check each sensor: spin each wheel by hand and confirm its count changes.
      ```
      python3 bench.py monitor   --wheel 0.10 --track 0.40 --ppr 8
      ```
   2. Check the watchdog. It must print `PASS` before anything else runs.
      ```
      python3 bench.py watchdog  --wheel 0.10 --track 0.40 --ppr 8
      ```
   3. Calibrate. It prints `pwm_min` and `ff`.
      ```
      python3 bench.py calibrate --wheel 0.10 --track 0.40 --ppr 8
      ```
   4. Step test: commanded vs measured speed.
      ```
      python3 bench.py step      --wheel 0.10 --track 0.40 --ppr 8 --pwm-min <from 3> --ff <from 3>
      ```
5. **Hardware-in-the-loop with the simulation.** Bring the stack up as usual (`~/arc_test_kit/bringup.sh all`), then:
   ```
   source ~/arc_test_kit/env.sh
   python3 bridge_node.py --wheel 0.10 --track 0.40 --ppr 8 --pwm-min <..> --ff <..>
   ```
   Drive the simulated robot (a goal in RViz, or the shopping manager) and change the BT mode. Both the Gazebo cart and the real wheels respond. Then plot the run:
   ```
   python3 plot_hil.py hil_logs/hil_<time>.csv
   ```
   This saves a PNG and prints the median speed error.

## Running it on the Raspberry Pi

With the Uno on the Pi's USB and the stack on the laptop, use the `arc_pi` kit:
- `deploy.sh` copies this folder to the Pi;
- `pi_up.sh motors` starts the bridge there;
- `bench.py` runs on the Pi as shown above.

The bridge stops cleanly on SIGTERM (it sends the Uno a stop). `--log-dir` sets where the CSV goes.

## Limits to state honestly

- **Measured wheel speed** comes from a few pulses per revolution. At 0.10 m/s, 8 magnets give only a few pulses per second, so the reading lags and steps.
- **The control loop** is feedforward (calibrated). `--ki` adds a slow correction; start at 0.
- **Odometry from these sensors** has no direction sensing and coarse resolution. That's fine for showing speed following. Real navigation needs a lidar (see the B1 simulation test for wheel-only drift).
- **The bench tests were checked against a simulated Arduino**, not the real board. The real numbers come from your run.
