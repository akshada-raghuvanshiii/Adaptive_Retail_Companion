# ARC route ordering by Dijkstra path cost

Makes `shopping_manager.py` visit the items in the shortest order. It measures distance as the Dijkstra path cost through the store, not straight-line distance, and not the order you typed.

## What changes
1. **New file `arc_planner/route_order.py`.** It contains no ROS code. It uses the same cost model as `dijkstra_planner.py`:
   - 0.20 m inflation, with cells of value 50 or more treated as occupied;
   - unknown cells cost 5×;
   - 8-connected grid;
   - start and goals snap to the nearest safe cell.
2. **`shopping_manager.py`: four small insertions.** Nothing else is changed. The patch script backs up the file first.
   - **When the list is confirmed:** the manager reads `/rtabmap/map` and the robot pose (`map → base_footprint`). It then orders the stops by **exact optimum (Held–Karp)**; with more than 12 stops it uses nearest-by-path-cost instead. Items on the same aisle spot become one stop, for example `atta_flour + bread`.
   - **At every reached stop:** the stops still to come are re-ordered from where the robot now is. The costs are recomputed from the current pose; there is no fixed table.
   - **It prints the chosen route,** its cost, and how much shorter it is than the typed order.
   - **If anything is missing, it keeps the typed order and says why.** That covers no map, no pose within 10 s, or items not in the `map` frame.

## Install (on the laptop)
```bash
cd ~ && unzip -o ~/Downloads/arc_route_order.zip -d ~ && cd ~/arc_route_order && python3 -m unittest discover -s test
python3 apply_route_order.py            # dry run: checks every anchor, compiles the result
python3 apply_route_order.py --write    # backup -> ~/slam_ws/arc_backups/, then patch
cd ~/slam_ws && colcon build --packages-select arc_planner && source install/setup.bash
```

## Use
```bash
ros2 run arc_planner shopping_manager                                     # ordered (exact)
ros2 run arc_planner shopping_manager --ros-args -p route_ordering:=nearest
ros2 run arc_planner shopping_manager --ros-args -p route_ordering:=as_entered   # old behaviour
ros2 run arc_planner shopping_manager --ros-args -p route_reorder_each_stop:=false
```

## Undo
The `--write` step prints the exact undo command. It is:
```bash
cp ~/slam_ws/arc_backups/shopping_manager.before_route_order_<time>.py ~/slam_ws/src/arc_planner/arc_planner/shopping_manager.py
rm ~/slam_ws/src/arc_planner/arc_planner/route_order.py
cd ~/slam_ws && colcon build --packages-select arc_planner
```

## Tested here (offline, no ROS)
`test/test_route_order.py` uses a model of the compact store, built from the positions in `arc_store.sdf`. It checks that:
- the exact order equals a brute-force optimum;
- same-spot items are merged;
- the route is never longer than the typed order;
- path costs go round the shelves;
- an unreachable stop goes last;
- `as_entered` keeps the typed order.

It has not yet been run inside ROS on the laptop.


## Update 16 Oct: checkout as the final destination (optional, off by default)

```
ros2 run arc_planner shopping_manager --ros-args -p route_end_at_checkout:=true \
    -p checkout_x:=<x> -p checkout_y:=<y> -p checkout_yaw:=<yaw>
```

- **The checkout goal:** a goal named "Checkout" is appended after the last item. Its position comes from these parameters, or from a `checkout` entry in the item file.
- **Ordering with the end fixed:** the exact order is optimised for start → items → checkout. That is not the same as the best open route followed by a drive to the checkout.
- **Re-ordering:** when the robot re-orders at each stop, the checkout stays last.
- **The shopping manager's own flow is unchanged:** "Next item: Checkout" is asked like any other item, and "SHOPPING COMPLETE" now prints at the counter.
- **Default (false):** exactly the previous behaviour.

**Checks:**
- The tests now cover the fixed end: Held–Karp with an end equals brute force, and is never worse than the open order plus checkout (9 tests, OK).
- The dry run on the `shopping_manager.py` I inspected applies cleanly (+117 lines) and compiles.
- The checkout logic was exercised with a stand-in node.
- **Not yet run in ROS.**
