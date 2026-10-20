# arc_ui_bridge: the ARC web UI connected to the ARC stack

This package serves your ARC shopping UI from the laptop. It turns the shopping list into stops
in the simulated store and sends them to the robot one at a time. It uses the same topics as
`shopping_manager`, which are `/shopping_goal` and `/shopping_item`.

It does not change any ARC package. The Dijkstra planner, hand-over, TEB-like controller,
behaviour tree, motion gate, RTAB-Map and TF are untouched. This is a separate Python package,
so YOLO and other Python dependencies stay out of `arc_person_follower`.

## Install (once)

```bash
cd ~/Downloads/arc_ui_bridge        # wherever you unzipped it
./install_into_slam_ws.sh
```

The script does three things:
- It copies the package into `~/slam_ws/src/arc_ui_bridge`. Any older copy is moved to
  `~/arc_backups/` first.
- It builds **only** this package.
- It runs the offline tests.

It installs no apt or pip packages. It prints any that are missing.

## Run

1. Bring up the stack as usual: `cd ~/arc_test_kit && ./bringup.sh all`. This starts the sim and
   the nav stack, and B6 sets the BT to LEAD.
   - Do **not** start `shopping_manager`. The bridge replaces it for UI trips, and two goal
     sources would fight. The bridge refuses a trip if it sees another `/shopping_goal` publisher.
2. Open a new terminal and run:
   ```bash
   source ~/slam_ws/install/setup.bash
   ros2 launch arc_ui_bridge arc_ui.launch.py
   ```
3. Open **http://localhost:8080**. The toast "Connected to ARC (ROS 2)" means the link is live.
   - To use a phone or tablet on the same hotspot, launch with `host:=0.0.0.0`, then open
     `http://<laptop IP>:8080`.
4. Add products and press **START SHOPPING**. Then:
   - **Mapping:** the bridge maps each UI product to its store spot (`config/product_map.yaml`).
     Products in the same spot are merged into one stop.
   - **Choose the route:** a dialog shows two routes, each with its Dijkstra path length through the
     aisles from where the robot is now, and how much shorter the optimised one is:
     - **Optimised route:** the shortest order, exact (Held–Karp) up to 12 stops. It uses the same
       `route_order.py` as the shopping manager.
     - **My order:** the order in which you added the products.

     Click one, or press **Cancel** or Esc to go back without starting.
   - **Ending:** both routes end at the **checkout counter**.
   - It sends the first stop.
5. At each stop, press **ITEM PICKED · NEXT**. After the last item the button reads **ITEM PICKED ·
   TO CHECKOUT**, and reaching the counter finishes the trip: the status shows "AT CHECKOUT · DONE".
   **SKIP STOP**, **PAUSE**, **RESUME**, **RETURN HOME** and **STOP (SOFTWARE)** also work.
   - "Home" means the pose where the trip started. RETURN HOME is still available to abandon a trip.
   - Skipping the checkout ends the trip where the robot is.

**Checkout counter.** Every trip ends at the store's checkout waiting point, so the optimised
route is planned with the checkout as its fixed end.
- **Where it comes from:** the `checkout` entry in `~/slam_ws/src/arc_store_sim/config/destinations_map.yaml`,
  at (−1.80, −1.25) in `map`, facing the counter. That is the same point the shopping manager uses.
- **Overrides:** `checkout_x`, `checkout_y` and `checkout_yaw`.
- **To switch it off:** `end_at_checkout:=false` gives the old behaviour, where the trip returns home after the last item.

The concept layout on the navigation screen shows the robot's chosen order. Its distances are
those of the concept drawing, so the header shows the robot's real path length on the store map.

**The robot only moves while the BT is in LEAD or FOLLOW.** The UI shows the BT mode. If it
is WAIT, the trip is still sent, but the motion gate holds the robot.

**Safety.** STOP and PAUSE are software stops. They hold the robot at its current pose and
cancel the controller's FollowPath goal. The hardware E-stop on the Uno is the safety device.

## How an arrival is accepted

`/navigation_goal_reached` is latched. The planner replans to a reached goal every 2 s, and each
replan re-sends "reached". We saw this race on the bench: 2 of 4 goals "completed" within 2 ms.
So the bridge accepts an arrival only when both of these hold:
- the robot is within `arrive_m` (1.0 m) of the stop;
- at least `min_leg_s` (3 s) have passed since the goal was sent.

Ignored arrivals are counted and appear in the UI event log.

## Parameters (`ros2 launch ... name:=value` or `--ros-args -p`)

| name | default | meaning |
|---|---|---|
| host / port | 127.0.0.1 / 8080 | web server address |
| route_choice | ask | `ask`: the UI shows both routes and the shopper picks. `fixed`: no dialog, use `strategy` |
| strategy | exact | `exact` or `nearest` for the optimised route, or `as_entered`. Used when no choice is sent |
| end_at_checkout | true | end every trip at the checkout counter |
| destinations_file | ~/slam_ws/src/arc_store_sim/config/destinations_map.yaml | where the checkout is read from |
| checkout_x / checkout_y / checkout_yaw | not set | override the checkout position |
| item_locations | ~/slam_ws/src/arc_store_sim/config/item_locations.yaml | store spots |
| product_map | (package) config/product_map.yaml | UI product id → store item |
| arrive_m / min_leg_s | 1.0 / 3.0 | arrival guard |
| map_topic | /rtabmap/map | grid used for path-cost ordering |
| follow_path_action | /follow_path | controller action cancelled on pause/stop |

## What has and has not been tested

- **Run here, without ROS:**
  - `python3 test/run_tests.py` (14/14). It covers:
    - product mapping, merging and ordering;
    - the arrival guard;
    - a full trip, and pause, stop, resume, home and skip;
    - the checkout: last stop, finishes the trip, skipping it, home still works, refused anywhere
      but last;
    - the server API against stand-in ROS objects: plan, both route choices, no map, checkout
      off, overrides, missing file.
  - A browser end-to-end test of the UI against `test/mock_bridge.py`:
    - the route dialog;
    - an optimised trip run to "AT CHECKOUT · DONE";
    - a typed-order trip in the order the products were added;
    - Cancel/Esc;
    - no page errors.
- **Not yet run on your laptop with the real stack:**
  - QoS matching;
  - the TF lookup;
  - whether `planned_path_to_teb` stops cleanly after the FollowPath cancel.

  For the first real run, watch `ros2 topic echo /shopping_goal` and the controller terminal
  while you press PAUSE.

## API (for scripts)

- `GET /api/status`
- `GET /api/products`
- `POST /api/plan {"items":[...]}`: both routes with path lengths and stops (ending `CHECKOUT`), or `options: null` with a reason
- `POST /api/trip {"items":[{"item_id":"FP001","quantity":1,"name":"Apple"}], "route":"optimised|typed"}`
- `POST /api/control {"action":"next|skip|pause|stop|resume|home|reset"}`
