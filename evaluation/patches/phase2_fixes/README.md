# ARC phase 3: the two recommended changes

Phase 2 showed that these two changes together completed all 30 A2 lists, using 26% less time and 38% less distance than the original stack. These patches put both changes into ARC itself.

**Nothing is written until you run a step with `--write`.** Each `--write` backs up the original file to `~/slam_ws/arc_backups/` and prints the undo command.

| Patch | File in `~/slam_ws` | Change |
|---|---|---|
| 1. Progress check | `src/arc_planner/config/teb_controller.yaml` | Adds 6 lines under `controller_server.ros__parameters`. Nothing else in the file changes. |
| 2. Route order | `src/arc_planner/arc_planner/shopping_manager.py` + new `route_order.py` | Adds 4 insertions (+117 lines) that order the list by Dijkstra path cost (exact Held–Karp). Use `route_ordering:=as_entered` to get the old behaviour. |

Untouched: the Dijkstra planner, the TEB-like controller, the BT, TF, RTAB-Map, and arc_person_follower.

## Patch 1: what is added

```yaml
    controller_plugins: ["TEB"]

    # Phase 3: explicit progress check (Nav2 default 0.5 m / 10 s caused all 62 A2 aborts).
    progress_checker_plugins: ["progress_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.25
      movement_time_allowance: 20.0
```

These are the same values that A2-pc and A2-final used through the test kit's copy of the file.

The script refuses to write if:
- a progress checker is already set;
- the `controller_plugins` line is not found exactly once;
- the patched file does not parse back to the original plus these keys alone.

## Steps (laptop), one at a time

1. Unpack the kit:
   ```bash
   cd ~ && unzip -o ~/Downloads/arc_phase3_patch.zip -d ~
   ```
2. Dry run of patch 1. It shows the diff and writes nothing:
   ```bash
   python3 ~/arc_phase3/apply_progress_check.py
   ```
3. Dry run of patch 2. It checks every anchor and compiles the result:
   ```bash
   cd ~/arc_phase3/route_order && python3 -m unittest discover -s test && python3 apply_route_order.py
   ```
4. Apply both changes once you're happy with the two dry runs:
   ```bash
   python3 ~/arc_phase3/apply_progress_check.py --write && python3 ~/arc_phase3/route_order/apply_route_order.py --write
   ```
5. Rebuild:
   ```bash
   cd ~/slam_ws && colcon build --packages-select arc_planner && source install/setup.bash
   ```
6. Check that the changes took effect:
   - `bringup.sh all` prints `progress_checker.required_movement_radius = 0.25`.
   - `ros2 run arc_planner shopping_manager` prints the chosen route, its cost, and how much shorter it is than the typed order.

**Undo:** each `--write` prints its own `cp …` undo command. After undoing, rebuild `arc_planner`.

## What is and isn't proven
- **Measured in simulation (phase 2):** the progress-check values, and exact ordering done by the test kit's mission runner. That ordering uses the same cost model as `route_order.py`.
- **Tested here, offline:**
  - patch 1 on a copy of your `teb_controller.yaml`: dry run, write and backup, and refusing a second run;
  - patch 2's 9 tests pass;
  - patch 2's dry run applies cleanly to the `shopping_manager.py` I inspected.
- **Not yet run:**
  - the patched `shopping_manager.py` inside ROS. The first real check is one typed list in `shopping_manager`, watching the printed route and the arrivals.
  - a test-kit run on the patched file. `./run.sh a2 --lists 3` now drives with the patched progress check; its aborts should look like A2-pc's (none), not A2's.
- **Caution from phase 2:** with the more lenient check, a robot pressed against an obstacle keeps pushing for up to 20 s. On the real cart, pair this with contact or slip detection.
