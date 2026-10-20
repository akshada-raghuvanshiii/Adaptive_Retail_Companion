# ARC simulation test kit (groups A–C)

This kit runs the Gazebo experiments A1–A4, B1, C1 and C2 from *ARC Novelties, Experiments and Publication Plan* on the laptop, logs everything to CSV, and turns the logs into tables and figures.

**It changes nothing in ARC.**
- It sends goals on `/goal_pose`, the topic RViz uses.
- It reads the true robot pose from Gazebo.
- It adds and removes small test boxes (A4).
- It may cancel a stuck FollowPath goal between legs, and logs every time it does.

Version 2026-10-15.1, checked against your source:
- `arc_motion_gate`, `dijkstra_planner`, `planned_path_to_teb`, `shopping_manager`, `teb_controller.yaml`
- the compact world, and `item_locations.yaml`

| ID | Test | Command | Trials (plan) | Rough time at 0.10 m/s* |
|---|---|---|---|---|
| A1 | Localisation error vs Gazebo truth, every 2 s | logged in every run | all A2 missions | — |
| A2 | Shopping-mission reliability | `./run.sh a2` | 30 random lists of 3–12 items | 2.5–3 h |
| A3 | TEB-like vs stock Nav2 RPP controller | `./run.sh a3-teb`, then `./run.sh a3-rpp` | 20 fixed start–goal pairs × 2 | 0.7 h each |
| A4 | Boxes dropped on the path after planning | `./run.sh a4` | 20 runs, 1–3 boxes | 0.8 h |
| B1 | Wheel-only vs ICP odometry | `./run.sh b1` (wheel stack) vs the first 10 A2 lists | 10 missions each | 0.9 h + remap |
| C1 | Ordering benchmark vs exact Held–Karp (offline) | `./run.sh c1` | 50 lists × n = 3, 5, 8, 10, 12 | 2 min |
| C2 | Ordering strategies, driven | `./run.sh c2` | n = 5, 8, 12 × 10 lists × 3 strategies = 90 | 8–9 h (overnight) |

\*These times are estimates from path lengths on a model of the store, at the LEAD speed cap with Gazebo at real time. They are not measurements. The snapshot records the real-time factor; if it is below 1, runs take longer.

---

## 0. Install (once)

```bash
cd ~ && unzip -o ~/Downloads/arc_test_kit.zip -d ~ && chmod +x ~/arc_test_kit/run.sh
cd ~/arc_test_kit && python3 -m unittest discover -s tests
```
The second line runs the offline tests (no ROS needed). It should end with `OK`.

Optional extra packages. `analyze.py` uses scipy for Wilcoxon tests (it falls back to a sign test) and matplotlib for the figures (it skips them without it). Check first:
```bash
python3 -c "import scipy, matplotlib; print('both present')"
```
Install only if that fails: `pip3 install --break-system-packages scipy matplotlib`.

## 1. Bring the stack up (the verified way)

**Every extra terminal starts with `source ~/arc_test_kit/env.sh`.**
- It keeps Gazebo transport (`GZ_IP=127.0.0.1`) and ROS 2 discovery (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) on the laptop itself.
- Without it, a Wi-Fi change cuts Gazebo off from ROS mid-run, which is what happened on 15 Oct at 10:30. A terminal without it also won't see the nodes.

**Quickest:** `./bringup.sh all`. It runs the steps below and checks each one before the next:
- every node gets its own window and its own log in `~/arc_logs/<time>/`;
- B6 is set to LEAD automatically;
- it stops with `[ FAIL ]` if a check fails. For example, if `auto_map` does not reach all 11 waypoints, the map is not locked.

You can also run the stages one at a time: `./bringup.sh stop | sim | slam | map | nav | status`.

The manual steps are below.

Follow `~/arc_compact_store_kit/RUN.md`, one terminal each:
1. T1–T3: simulation and robot.
2. T5a: ICP odometry.
3. T5b: RTAB-Map with `-d`.
4. Map with `auto_map.py`, then lock the map: `ros2 service call /rtabmap/set_mode_localization std_srvs/srv/Empty`.
5. B1–B5.
6. **B6: type `3` (LEAD).** Use LEAD for every run so that all runs share one speed cap. The probe and the samples record the mode.
7. **Do not start B7 (Shopping Manager).** The runner takes its place.
8. B8 (labels) and RViz are optional. Gazebo's GUI and RViz cost CPU, so close RViz for the long runs.

Reloading a saved map in localisation mode does not work yet, so map and lock in the same session. After every restart, mapping takes about 8 min. The snapshot stores the map's fingerprint, so the logs show which map each run used.

## 2. Pre-flight, then a 3-minute smoke test

Every terminal used for the kit starts with:
```bash
cd ~/arc_test_kit && source /opt/ros/jazzy/setup.bash && source ~/slam_ws/install/setup.bash
```
Then:
```bash
./run.sh probe
```
Every line should read `[OK]`. The probe checks:
- the planner, controller, bridge, gate and RTAB-Map
- which odometry RTAB-Map uses and which controller plugin is loaded
- that the planner subscribes to `/goal_pose`, and that `/navigation_goal_reached` has a publisher
- the map, the map pose and the true pose (map vs truth)
- the `/cmd_vel` type, the BT mode and the item file
- that a test box can be spawned and removed in the south-east corner

Then drive one start–goal pair end to end and check the logs:
```bash
./run.sh smoke && ./run.sh report
```

## 3. Run order

| Step | Command | Stack |
|---|---|---|
| 1 | `./run.sh a2` | ICP + TEB (also gives A1, and the ICP arm of B1) |
| 2 | `./run.sh c1` | offline; uses the map the runner saved |
| 3 | `./run.sh a3-teb` | ICP + TEB |
| 4 | swap the controller (below), then `./run.sh a3-rpp` | ICP + RPP |
| 5 | swap back to TEB, then `./run.sh a4` | ICP + TEB |
| 6 | `./run.sh c2` (overnight) | ICP + TEB |
| 7 | restart with wheel odometry (below), then `./run.sh b1` | wheel + TEB |
| 8 | `./run.sh report`, then `./run.sh pack` | — |

- **Labels are checked.** A label containing `icp`, `wheel`, `teb` or `rpp` must match the running stack, or the runner stops before driving. This prevents an RPP run being logged as TEB.
- **Resume after a crash or Ctrl-C:** `./run.sh status` prints the `--start-at N` value to use, then e.g. `./run.sh c2 --start-at 41`. Rows are written as they happen, so nothing logged is lost.
- **Running short of time:** extra arguments override a preset, for example `./run.sh c2 --per-size 5` (45 missions). Then write the deviation from the plan in the notebook: `./run.sh note "C2 reduced to 5 lists per size"`.
- **Preview a preset without driving:** `./run.sh plan c2` lists the missions and their planned costs.

### Controller swap for A3
```bash
./run.sh make-rpp
```
This writes `configs/rpp_controller.yaml` from your `teb_controller.yaml`:
- Your file is only read, never changed.
- Plugin id `TEB` stays, because `planned_path_to_teb` asks for `controller_id 'TEB'`.
- The RPP block takes your `max_vel_x`, `max_vel_theta` and `acc_lim_theta`, so the comparison is at equal limits.

Then:
1. Stop B1 and B2 (Ctrl-C).
2. In B1 run:
   ```bash
   ros2 run nav2_controller controller_server --ros-args --params-file ~/arc_test_kit/configs/rpp_controller.yaml -r cmd_vel:=/cmd_vel_nav
   ```
3. Start B2 again with the same command as before.
4. Run `./run.sh a3-rpp`.

Afterwards, restart B1 with `teb_controller.yaml`, and restart B2.

### Wheel-odometry stack for B1
`./bringup.sh all-wheel` does everything in one go: it skips T5a (no ICP odometry), starts RTAB-Map on `/odom`, maps, locks, checks and starts B1–B6. Then run `./run.sh b1`.

The mapping drive's guard is relaxed to 0.6 m / 20° in this mode, because wheel-only drift is what B1 measures. The locked map must still pass the usual 0.15 m check, so B1 navigation is compared on a map as good as A2's.

The manual steps it replaces:
1. Close everything.
2. Start T1–T3.
3. **Skip T5a.**
4. Start T5b with `-r odom:=/odom`.
5. Map, lock, then start B1–B6.

`b1` drives the same 10 lists as the first 10 of `a2`, so the report pairs them.

If `auto_map` stops on its truth guard because the wheel-only map drifts, that is itself a B1 result. Log it:
```bash
./run.sh note "B1: auto_map guard stopped at waypoint N, err X m"
```
You can then retry with `--guard-m 0.6 --guard-deg 20` to get a usable map. Note that in the notebook too.

## 4. What gets logged

Each test starts with a snapshot in `results/run_logs/<time>_<test>/`:
- `system.txt` and the full command
- `source_state.txt`: git commits or md5 of the planner, bridge, Shopping Manager, controller params, BT and item file
- `configs/`: copies of the params and item file used
- `map_state.txt`: fingerprint of `rtabmap.db`
- `real_time_factor.txt`
- `controller_plugin.txt`, `odometry.txt` (icp / wheel) and `bt_mode.txt`
- `nodes.txt` and `topics.txt`

`results/LAB_NOTEBOOK.md` gets:
- a start entry for each test, giving the controller, odometry, mode and RTF
- an end summary: missions completed, failures, recoveries, collisions and median localisation error

Your own notes go in with `./run.sh note "..."`, and screenshots with `./run.sh shot "..."`.

Data files in `results/` (appended across tests; the `label` column separates them):

| File | One row per | Key columns |
|---|---|---|
| `samples.csv` | 0.2 s while driving a leg | map pose, true pose, `pos_err`, `yaw_err_deg`, `truth_src`, commanded v/ω, `bt_mode`, `static_clearance` |
| `legs.csv` | drive to one stop | `success`, `reason`, `reached_msg`, `planned_len`, `driven_len_true`, `time_s`, `ctrl_aborts`, `resends`, `stall_events`, box and static clearance, `mean_abs_wdot`, `w_sign_changes` |
| `missions.csv` | list or pair | `order`, `planned_cost_J`, `exact_cost_J`, `driven_total`, `time_total`, `completed`, `recoveries`, `collisions` |
| `events.csv` | event | `goal_sent`, `path_received`, `goal_reached_msg`, `controller_aborted`, `goal_resent`, `stall`, `box_spawned`, `box_removed`, `leg_failed`, `runner_cancelled_follow_path`, … |
| `ordering_benchmark.csv` | C1 list | `J_*` and `gap_*_pct` per strategy, solve times |
| `run_<label>_<time>.json` | runner start | all arguments, the stack config and the full mission plan |

## 5. How the numbers are defined

- **Arrival.** A leg succeeds when `planned_path_to_teb` publishes `/navigation_goal_reached = True` after the goal was sent. This is the same signal the Shopping Manager waits for (`reason = arrived`). If that message never comes but the robot stops within 0.35 m with the controller idle, the leg is `arrived_tol`. Failures are:
  - `controller_aborted`
  - `stalled` (no motion for 40 s)
  - `no_path`
  - `timeout`
- **Waiting between legs.** `planned_path_to_teb` ignores new paths while a FollowPath runs. So the runner waits for the action to go idle before each goal. If it stays busy for more than 20 s, the runner cancels it and logs `controller_still_busy`.
- **Recoveries.** ARC has no recovery behaviour of its own: after a FollowPath abort, the robot just stops. The runner re-sends the goal once (`--retries 1`) and counts recoveries as `ctrl_aborts + resends`. `legs first try` in the report is the success rate with no help from the runner. Use `--retries 0` to measure the stack completely alone.
- **A1.** The `map → base_footprint` pose is looked up at the timestamp of the Gazebo pose message, so pose motion between the two readings does not show up as error. It is reported every 2 s of sim time.
  - PASS = within 0.15 m and 5°, the same limit as `check_frames.py`.
  - A loss episode starts when the error exceeds 0.5 m or 15°.
  - If the pose bridge does not name the model, the kit polls `gz model -p` instead, as `check_frames.py` does. This is marked `gzcli` in `truth_src`.
- **Collisions (A2) and contacts (A4).** These come from geometry, not a contact sensor:
  - for mapped obstacles, the true-pose distance from the robot edge (r = 0.105 m) to the nearest occupied cell of the RTAB-Map grid (5 cm cells);
  - for test boxes, the distance to the box edge.

  A value of 0 or less counts as a collision.
- **Smoothness (A3).** Mean |dω/dt| over consecutive `/cmd_vel` messages (TwistStamped from the motion gate), differenced in sim time; gaps over 0.5 s are skipped. Steering reversals per metre are also given.
- **Planned cost J.** The kit's own Dijkstra on the RTAB-Map grid: 8-connected, with 0.15 m inflation, from home through the stops, with no return leg. All orderings and the Held–Karp optimum use the same cost matrix.
- **Stops.** Items that share an aisle spot (the two faces of one aisle) are one driving stop. 29 items give 13 named stops at 9 distinct spots.

## 6. Findings from reading the code (for the paper)

- **`shopping_manager.py` visits items in the order they are typed.** It has no Dijkstra-cost table and no nearest-neighbour step. This answers the plan's open question: it neither recomputes costs nor uses a fixed table. `nn_dijkstra` and `exact` in C1/C2 are the kit's implementations, so describe N3 as evaluated in simulation, not as running in ARC, until the manager does it. `as_entered` is ARC's current behaviour, which is why A2 and B1 use it.
- `dijkstra_planner` replans every 2 s while the robot moves. `planned_path_to_teb` ignores those paths while a FollowPath runs, so the robot follows the first path of each leg. A4 therefore tests the TEB-like controller's local avoidance, not global replanning.
- There is no recovery behaviour after a FollowPath abort (see above).
- **First A2 attempt, 14 Oct, 22:00:**
  - ICP odometry jumped by about 2.5 m and 87° during a return-to-home drive.
  - RTAB-Map's correction (`map → icp_odom`) stayed at about 5 cm, so the map never pulled the pose back.
  - The robot then navigated in a shifted frame and pushed against walls.
  - The kit now logs each TF layer and stops a run on localisation loss (below).

## 7. Threats to validity to state

- The robot is a TurtleBot3 Burger, not the cart, and there are no real motors.
- There is one store map and no moving shoppers.
- The goal sender stands in for the Shopping Manager.
- LEAD mode caps linear speed at 0.10 m/s.
- Gazebo timing depends on the laptop's real-time factor.
- Collisions are measured geometrically.

## 8. Changes from the kit you sent

- `/cmd_vel` is subscribed as TwistStamped, detected from the topic. Before, the kit got no messages, so v/ω, smoothness and the "stopped" check were wrong.
- Goals go to `/goal_pose`. Before, the kit took the planner's first PoseStamped subscription, which could have been `/shopping_goal` or `/person_follow_goal`.
- Arrival uses ARC's `/navigation_goal_reached`, and the runner waits for FollowPath to be idle. FollowPath aborts are read from the action status topic.
- Presets now match the plan's trial counts: random-size lists for A2/B1, fixed pairs for A3/A4, 1–3 random boxes, and C2 at 90 missions. A1 is reported every 2 s.
- Added `events.csv`, recoveries, collisions, `status`, `smoke`, `plan TEST`, the label/stack check, and the controller, odometry and BT mode in snapshots.
- RPP uses the TEB speed and angular-acceleration limits.
- The item file is the one the Shopping Manager reads (`~/slam_ws/src/arc_store_sim/config/item_locations.yaml`).
- Bring-up uses the verified manual steps instead of `arc_up.sh`.
- Offline tests are in `tests/`.
- v4:
  - Drives to the start are logged as leg -1. They are left out of the A2 tables but count for A1.
  - `samples.csv` adds each TF layer against truth: RTAB-Map correction (`corr_*`), ICP (`lidar_*`, `lidar_err`) and wheels (`wheel_*`, `wheel_err`).
  - A run stops when the map pose is more than 1 m off for 10 s. Change this with `--loss-abort-m` and `--loss-abort-s`; `0` turns it off.
  - Ctrl-C exits cleanly.
- v12 (2026-10-15.11), A4 box removal:
  - On 15 Oct, A4 left 12 test boxes in Gazebo. Gazebo replied "true" to every removal, but the boxes stayed, so later legs drove into blocked aisles. That run is void.
  - Each removal is now confirmed in `gz model --list` and retried up to 4 times. A failure is logged as `box_remove_failed`.
  - Before every leg, leftover test boxes are swept (`stray_boxes_removed`). If one cannot be removed, the run stops (`boxes_not_removed`).
  - The probe now checks for leftover boxes and confirms the test box really appears and disappears.
  - New commands: `./run.sh clear-boxes`, and `./run.sh void LABEL "why"`. `void` moves a bad run's rows into `results/void/` (nothing is deleted) and notes it in the notebook.
- v13 (2026-10-15.12): stale paths and false arrivals.
  - After an arrival, `dijkstra_planner` can still publish a re-plan to the **previous** goal, because it replans whenever the start cell changes.
  - On 15 Oct (A4 pair01), the runner took that 0.07 m path as the new leg's path. `planned_path_to_teb` drove it, published `/navigation_goal_reached`, and the leg was logged as "arrived" after 0.02 s with 0 m driven.
  - A path now counts for a goal only if it ends within `--goal-snap-m` (1.0 m, the planner's snap radius) of that goal. Otherwise it is logged as `stale_path`.
  - `/navigation_goal_reached` counts only if the robot is within 1.0 m of the goal. Otherwise it is logged as `reached_msg_far` and the goal is re-sent.
  - `analyze.py` and the notebook summary re-check every earlier leg. A "reached" leg that ended more than 1 m from its goal is counted as a failure (`false_arrival`) and listed in the report.
  - For the paper: ARC's `shopping_manager` sends the next item right after a reached message, so ARC itself can hit the same race.
- v14 (2026-10-16.1): `./bringup.sh all-wheel` brings up the B1 wheel-odometry stack in one step.

## Phase 2: variants on the same tasks (ARC files untouched)

Run these after phase 1 (A1–C2, B1), on a normal ICP stack.

1. **Make the progress-check variant, once.** `./run.sh make-progress` writes `configs/teb_progress.yaml`, a copy of ARC's `teb_controller.yaml`. The TEB block is unchanged. The only addition is a progress check of 0.25 m within 20 s; Nav2's default, which ARC uses, is 0.5 m within 10 s. ARC's own file is only read.
2. **Restart the navigation layer with the copy.** The map stays up, so no re-map is needed:
   ```
   TEB_YAML=$PWD/configs/teb_progress.yaml ./bringup.sh nav
   ```
   Bring-up prints `progress_checker.required_movement_radius = 0.25`.
3. **Run the two variants** on the same 30 lists and seed as A2:
   - `./run.sh a2-pc`: progress fix only, typed order (label `teb_icp_pc`).
   - `./run.sh a2-final`: progress fix plus exact path-cost order (label `teb_icp_pc_final`).

   Each takes about 2–3 h.
4. **Go back to the baseline controller:** `./bringup.sh nav`, without `TEB_YAML`.
5. **`./run.sh report`** adds three paired comparisons:
   - A2 vs A2-pc;
   - A2 vs A2-final;
   - A2-pc vs A2-final.

   A2-final uses a different visiting order, so it is paired by shopping list. Time and distance are compared only on lists that both configurations completed. Completion is compared with a sign test on the discordant lists.

**Labels are checked.** A `pc` label refuses to run unless the controller has the relaxed progress check. A baseline label (`teb_icp`, `teb_icp_pairs`, …) refuses to run if it does.

**Ordering.** `a2-final` uses the kit's exact (Held–Karp) ordering, the same method as C1/C2. The kit's cost grid approximates ARC's planner and is not identical: occupied at 65 or above, robot radius 0.15 m, unknown cells blocked. ARC's planner uses 50 or above, 0.20 m inflation and unknown cost 5. `route_order.py`, the patch proposed for ARC, mirrors the planner exactly, so routes from the patch can differ slightly from the kit's.

- v15 (2026-10-16.2):
  - Phase 2 presets (`make-progress`, `a2-pc`, `a2-final`) and the `pc` label check.
  - Comparisons can pair by list, use only jointly completed missions for time and distance, and add a sign test on completion.
  - A1 is now a cumulative plot instead of overlaid histograms.
  - Bring-up prints the loaded progress check.
- v16 (2026-10-16.3): bring-up now refuses to start when loopback multicast is off, instead of printing a note. On 16 Oct this left RTAB-Map without `/tf_static` (`"base_scan" does not exist`), and the wheel bring-up failed. Fix it with `sudo ip link set lo multicast on && ros2 daemon stop`.
- v17 (2026-10-16.4): checkout as the final destination.
  - `./run.sh c1-checkout X,Y` runs the C1 benchmark with every route ending at the checkout. It is offline, so it is safe to run during a driven test.
  - It compares four orders: typed + checkout, nearest-next + checkout, the best open order + checkout, and the exact order planned with the checkout as the fixed end.
  - `common.order_exact(D, n, end)` supports a fixed end. The driven presets are unchanged: A2, A2-pc and A2-final all end at the last item, so their comparison stays like-for-like.
- v18 (2026-10-16.5): `./run.sh footprint`, an offline check of whether a standard 180 L supermarket cart (1000 × 610 mm) fits the store.
- v19 (2026-10-16.6): optional Raspberry Pi link. `env.sh` reads `~/.arc_net` (written by `~/arc_pi/laptop/hil_mode.sh`); without that file nothing changes. In HIL mode `bringup.sh` stops if the laptop's cable IP is missing, and `PERSON_INPUT=camera ./bringup.sh nav` skips B6 so the webcam bridge drives the BT.
  - It reports stops reachable from home at four robot radii: 0.15 m (kit grid), 0.20 m (ARC planner and TurtleBot), 0.35 m (cart driving straight, lenient) and 0.59 m (cart turning on the spot, strict).
  - A stop counts only if the robot can stand within 1.0 m of it, the planner's snap radius.
  - The command reads `results/map.npz` only.
