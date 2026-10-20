# ARC route option: Dijkstra-optimised order or typed order

Once the shopping list is confirmed, the shopping manager works out two routes and lets you choose one:

- **[1] Optimised:** visits the item coordinates in the order with the shortest total **Dijkstra path cost**. The cost is measured through the aisles on the live map, starting from where the robot is now. It is not the order you typed, and not straight-line distance.
- **[2] As typed:** keeps your order. This is ARC's original behaviour.

Both routes are priced on the same costs the planner drives on, so the numbers can be compared directly.

```
========== ROUTE OPTIONS (Dijkstra path cost from the robot) ==========
  [1] Optimised (exact):   14.20 m   pasta -> fruits -> tea_coffee -> milk
  [2] As typed:            17.31 m   pasta -> tea_coffee -> milk -> fruits
  The optimised route is 3.10 m (18%) shorter.
  (computed in 535 ms)
========================================================================

Choose route: 1 = optimised, 2 = as typed [1]:
```

(Output above is from the offline test, using a model of the compact store with the robot at (−0.5, −2.0). Your numbers will depend on the live map and the robot's pose.)

## How the optimised order is found
1. **Item coordinates** come from `item_locations.yaml`, which must be in the `map` frame. Items on the same spot (within 0.10 m) become one stop.
2. **Start pose:** the robot's pose comes from TF (`map → base_footprint`), and the map from `/rtabmap/map`.
3. **Cost model:** the same as `dijkstra_planner.py`:
   - 0.20 m inflation;
   - cells of value 50 or more are occupied;
   - unknown cells cost 5×;
   - 8-connected grid.

   The cost between every pair of stops is the Dijkstra path length. Start and goals snap to the nearest safe cell.
4. **Ordering:** the exact optimum comes from **Held–Karp** dynamic programming for up to 12 stops; the compact store has 9. With more stops it uses nearest-next by path cost.
5. **Re-ordering at each stop:** if you chose the optimised route, the stops still to come are re-ordered from wherever the robot has actually got to. If you chose the typed order, nothing is re-ordered.
6. **Fallbacks:** if there is no map, no pose within 10 s, or the items are not in `map`, it says why and keeps the typed order.

## Parameter

| `route_ordering` | Behaviour |
|---|---|
| `ask` (default) | Shows both routes and asks; Enter picks the optimised one |
| `exact` | Always optimised, no question (for scripted runs) |
| `nearest` | Always nearest-next by path cost |
| `as_entered` | Always the typed order (original behaviour) |

```bash
ros2 run arc_planner shopping_manager                                          # ask
ros2 run arc_planner shopping_manager --ros-args -p route_ordering:=exact
ros2 run arc_planner shopping_manager --ros-args -p route_ordering:=as_entered
```

The other parameters from the earlier patch still work:
- `route_reorder_each_stop` (default `true`);
- `route_end_at_checkout` (default `false`), together with `checkout_x`, `checkout_y` and `checkout_yaw`.

## Install (laptop), one step at a time

1. Unpack:
   ```bash
   cd ~ && unzip -o ~/Downloads/arc_route_option.zip -d ~
   ```
2. Run the offline tests. They need no ROS:
   ```bash
   cd ~/arc_route_option && python3 -m unittest discover -s test
   ```
3. Dry run. It checks every anchor and compiles the result, but writes nothing:
   ```bash
   python3 apply_route_option.py
   ```
   It reports either **fresh**, meaning your file has no route ordering yet so it adds both, or **upgrade**, meaning the earlier route-order patch is there and it only adds the choice.
4. Apply. This backs up to `~/slam_ws/arc_backups/` and prints the undo command:
   ```bash
   python3 apply_route_option.py --write
   ```
5. Rebuild:
   ```bash
   cd ~/slam_ws && colcon build --packages-select arc_planner && source install/setup.bash
   ```

**Changed:** `shopping_manager.py`, plus `route_order.py` (new, or replaced with this version).

**Untouched:** the Dijkstra planner, the TEB-like controller, the BT, TF, RTAB-Map and `arc_person_follower`.

## Tested here (offline, no ROS)

All 22 tests pass (`python3 -m unittest discover -s test`).

**`test_route_order.py`** checks the route maths:
- the exact order equals the brute-force optimum, with and without a fixed checkout end;
- same-spot items merge;
- costs are path lengths, not straight lines;
- an unreachable stop goes last;
- the optimised and typed routes are priced on the same costs.

**`test_shopping_manager_option.py`** runs the **patched `shopping_manager.py`**, using stand-in ROS objects and typed answers. It checks that:
- option 1 publishes the optimised order and option 2 the typed order;
- Enter defaults to option 1, and a bad answer asks again;
- the later stops are re-ordered only after option 1;
- `exact` and `as_entered` never ask;
- with no map it keeps the typed order and says so;
- an upgraded file is identical to a fresh one.

**Not yet run inside ROS on the laptop.** The first real run will show the actual map costs.
