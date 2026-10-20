# ARC compact store kit

A replacement 8 × 7 m supermarket for the ARC simulation that fits the current RTAB-Map
coverage, with destinations, item poses and planner goals in one consistent `map` frame.
Existing architecture is kept: Shopping Manager → `/shopping_goal` → Dijkstra →
`/planned_path` → your controller.

**Start here:** `RUNBOOK.md` (steps 1–12, with what is done, checked, and still to test).

| File | Purpose |
|---|---|
| `layout.py` | Single source of truth: fixtures, destinations, spawn convention |
| `verify_layout.py` | Clearance / reachability / aisle-width / fits-the-map checks, preview PNG |
| `generate_compact_store.py` | Writes the Gazebo world (copying your world-level settings), layout JSON, destinations YAML |
| `remap_item_locations.py` | Moves your items to the new destinations, keeping IDs, categories and fields |
| `arc_planner_addon/goal_frame_guard.py` | Planner goal guard: map goals pass, TF-transform others, reject the rest |
| `apply_patches.py` | Minimal patches to `dijkstra_planner.py` (2 lines) and `shopping_manager.py` (frame literal) |
| `install.sh` | Dry run by default; `--apply` backs up, installs, writes `RESTORE.sh` |
| `ros_tools/check_frames.py` | Measures Gazebo world pose vs `map → base_footprint` |
| `ros_tools/send_test_goal.py` | Drive test for one or all destinations, with a reason for each failure |
| `output/` | Pre-generated world (standalone defaults), layout JSON, destinations, previews, verification report |
| `tests/` | `python3 -m unittest discover -s tests` (19 tests, no ROS needed) |

Requirements on the laptop: Python 3, `python3-yaml` (already present with ROS 2), and
`matplotlib` only for the preview image.
