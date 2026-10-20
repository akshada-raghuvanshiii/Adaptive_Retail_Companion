#!/usr/bin/env bash
# ARC test bring-up: the verified manual sequence (RUN.md, 12-15 Oct 2026), scripted.
# Each node gets its own terminal window (gnome-terminal) and its own log file;
# every stage waits for its check before going on and stops if the check fails.
#
#   ./bringup.sh stop      kill everything from earlier runs
#   ./bringup.sh sim       T1 Gazebo, T2 robot description, T3 spawn + bridge   (check: /scan, /odom)
#   ./bringup.sh slam      T5a ICP odometry, T5b RTAB-Map with a fresh map       (check: /odom_icp, map TF)
#   ./bringup.sh map       auto_map drive (here), lock the map, check map vs truth (check: 11 reached, PASS)
#   ./bringup.sh check     map vs Gazebo truth now (check_frames.py)
#   ./bringup.sh nav       B1-B6 (B6 gets "3" = LEAD automatically)              (check: active, TEB limits, LEAD)
#   ./bringup.sh all       stop, sim, slam, map, nav - in one go (about 15 min)
#   ./bringup.sh all-wheel the same with wheel odometry only (B1): no T5a, RTAB-Map on /odom
#   ./bringup.sh status    what is running now
#   PERSON_INPUT=camera ./bringup.sh nav|all   no B6: BT inputs come from the webcam bridge (arc_pi kit)
#   Network: laptop-only by default; ~/arc_pi/laptop/hil_mode.sh on adds the Pi Ethernet link.
#
# Every window uses env.sh (GZ_IP=127.0.0.1, ROS discovery on localhost), so Wi-Fi
# changes cannot break the simulation. Use `source ~/arc_test_kit/env.sh` in any
# extra terminal, or it will not see these nodes.
# Nothing in ARC is edited by this script. Logs: ~/arc_logs/<date_time>/<name>.log
# no "set -u": the ROS setup scripts read unset variables and would abort this script silently
KITDIR=$(cd "$(dirname "$0")" && pwd)
SRC="source $KITDIR/env.sh"
source "$KITDIR/env.sh" 2>/dev/null
if [ "${ARC_NET_MODE:-local}" = hil ]; then
  [ "${1:-}" = stop ] || [ "${1:-}" = status ] || \
  echo "[bringup] NOTE: HIL mode. Keep the link to the Pi (hotspot or cable) as it is until 'bringup.sh stop'."
elif command -v nmcli >/dev/null && [ "$(nmcli radio wifi 2>/dev/null)" = "enabled" ]; then
  echo "[bringup] NOTE: Wi-Fi is ON. That is fine if it stays connected; a Wi-Fi drop or network change"
  echo "          mid-run has cut nodes off before. If nothing else needs it: nmcli radio wifi off"
fi
# ROS discovery is pinned to the loopback interface (fastdds_localhost.xml) and needs multicast on it.
# Without it some nodes never see each other (16 Oct: RTAB-Map got no /tf_static, so "base_scan does not
# exist" and no map TF), while other topics still work - so it is a hard stop, not a note.
if [ "${1:-}" != "status" ] && [ "${1:-}" != "stop" ] && ! ip link show lo 2>/dev/null | grep -q MULTICAST; then
  echo "[bringup] STOP: the loopback interface has no multicast, so ROS nodes cannot all find each other."
  echo "          Fix (once per boot, or after network changes):  sudo ip link set lo multicast on && ros2 daemon stop"
  echo "          then run this command again."
  exit 1
fi
# HIL mode (~/.arc_net = hil): the laptop's link IP must exist BEFORE any node starts. Fast DDS only
# uses whitelisted interfaces that are up when a node starts, so a late cable leaves the Pi unseen.
if [ "${ARC_NET_MODE:-local}" = hil ] && [ "${1:-}" != "status" ] && [ "${1:-}" != "stop" ]; then
  _HIL_IP=$(sed -n 's/.*<address>\(.*\)<\/address>.*/\1/p' "$HOME/.arc_hil/fastdds_hil.xml" | grep -v '^127\.' | head -1)
  if ! ip -4 addr show 2>/dev/null | grep -q "inet $_HIL_IP/"; then
    echo "[bringup] STOP: HIL mode is on but this laptop has no address $_HIL_IP (link to the Pi)."
    echo "          Hotspot: join it, then  ~/arc_pi/laptop/hil_mode.sh on   (the phone may have given a new address)"
    echo "          Cable:   plug it in, then  ~/arc_pi/laptop/net_laptop.sh up"
    echo "          or go back to laptop-only:  ~/arc_pi/laptop/hil_mode.sh off"
    exit 1
  fi
  echo "[bringup] network: HIL (laptop + Pi on $_HIL_IP; other interfaces excluded)"
fi
KIT=$HOME/arc_compact_store_kit
TEB_YAML=${TEB_YAML:-$HOME/slam_ws/src/arc_planner/config/teb_controller.yaml}
SESSION_FILE=$HOME/arc_logs/.current
if [ "${1:-}" = "all" ] || [ "${1:-}" = "stop" ] || [ ! -f "$SESSION_FILE" ]; then
  LOG=$HOME/arc_logs/$(date +%Y%m%d_%H%M%S)
else
  LOG=$(cat "$SESSION_FILE")
fi
mkdir -p "$LOG"

say()  { echo -e "\033[1;36m[bringup]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[  OK  ]\033[0m $*"; }
fail() { echo -e "\033[1;31m[ FAIL ]\033[0m $*"; echo "Logs: $LOG"; exit 1; }

# start_win NAME "command"  - new window titled NAME, output also to $LOG/NAME.log
start_win() {
  local name=$1 cmd=$2
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$name" -- bash -c "$SRC && { $cmd ; } 2>&1 | tee -a '$LOG/$name.log'; echo; echo '[$name exited - close this window]'; exec bash" >/dev/null 2>&1
  else
    nohup bash -c "$SRC && $cmd" > "$LOG/$name.log" 2>&1 < /dev/null &
  fi
  say "started $name  (log: $LOG/$name.log)"
}

# wait_for SECONDS "description" command...
wait_for() {
  local t=$1 what=$2; shift 2
  local end=$((SECONDS + t))
  while [ $SECONDS -lt $end ]; do
    if "$@" >/dev/null 2>&1; then ok "$what"; return 0; fi
    sleep 1
  done
  fail "$what (waited ${t}s)"
}

topic_once() { timeout 4 ros2 topic echo --once "$1" >/dev/null 2>&1; }
tf_ok() { timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q Translation; }

do_stop() {
  say "stopping everything from earlier runs"
  for p in "gz sim" parameter_bridge rtabmap icp_odometry robot_state_publisher rviz2 controller_server \
           lifecycle_manager dijkstra_planner planned_path_to_teb person_assistant_bt arc_motion_gate \
           person_state_input shopping_manager item_markers auto_map.py mission_runner.py; do
    pkill -f "$p" 2>/dev/null
  done
  sleep 3
  ros2 daemon stop >/dev/null 2>&1
  rm -f "$SESSION_FILE"
  ok "stopped"
}

do_sim() {
  echo "$LOG" > "$SESSION_FILE"
  start_win T1_gazebo "ros2 launch arc_store_sim store.launch.py world:=arc_store.sdf"
  wait_for 90 "Gazebo world arc_store is up" bash -c "gz service -l 2>/dev/null | grep -q '/world/arc_store/create'"
  start_win T2_robot_description "export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true"
  sleep 3
  start_win T3_spawn_bridge "ros2 run ros_gz_sim create -name arc -file \$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x 0.0 -y 0.0 -z 0.01 -Y 0.0 && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/slam_ws/src/arc_store_sim/config/arc_store_burger_bridge.yaml -p use_sim_time:=true"
  wait_for 60 "/scan is publishing" topic_once /scan
  wait_for 30 "/odom is publishing" topic_once /odom
  local x; x=$(timeout 4 ros2 topic echo --once /odom --field pose.pose.position.x 2>/dev/null | head -1)
  say "odom x at start: ${x:-?} (should be ~0)"
}

do_slam() {
  if [ "${ODOM:-icp}" = "wheel" ]; then
    # B1: wheel odometry only - no ICP odometry; RTAB-Map takes Gazebo's /odom (scan matching still
    # closes loops in the map, but between keyframes the pose is dead-reckoned from the wheels)
    say "ODOM=wheel: skipping T5a (ICP odometry); RTAB-Map uses /odom"
    start_win T5b_rtabmap "ros2 run rtabmap_slam rtabmap -d --Reg/Strategy 1 --Reg/Force3DoF true --Grid/Sensor 0 --Grid/RangeMin 0.2 --RGBD/NeighborLinkRefining true --Optimizer/GravitySigma 0 --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.5 --ros-args -r __ns:=/rtabmap -p use_sim_time:=true -p frame_id:=base_footprint -p subscribe_depth:=false -p subscribe_rgb:=false -p subscribe_scan:=true -p approx_sync:=true -r scan:=/scan -r odom:=/odom"
    wait_for 60 "TF map -> base_footprint exists" tf_ok map base_footprint
    return
  fi
  start_win T5a_icp_odometry "ros2 run rtabmap_odom icp_odometry --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.5 --Icp/Iterations 10 --Icp/Epsilon 0.001 --Icp/OutlierRatio 0.7 --Icp/CorrespondenceRatio 0.2 --Odom/ScanKeyFrameThr 0.4 --OdomF2M/ScanSubtractRadius 0.05 --OdomF2M/ScanMaxSize 15000 --Reg/Force3DoF true --ros-args -p use_sim_time:=true -p frame_id:=base_footprint -p odom_frame_id:=icp_odom -p guess_frame_id:=odom -p publish_tf:=true -r scan:=/scan -r odom:=/odom_icp"
  wait_for 45 "/odom_icp is publishing" topic_once /odom_icp
  start_win T5b_rtabmap "ros2 run rtabmap_slam rtabmap -d --Reg/Strategy 1 --Reg/Force3DoF true --Grid/Sensor 0 --Grid/RangeMin 0.2 --RGBD/NeighborLinkRefining true --Optimizer/GravitySigma 0 --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.5 --ros-args -r __ns:=/rtabmap -p use_sim_time:=true -p frame_id:=base_footprint -p subscribe_depth:=false -p subscribe_rgb:=false -p subscribe_scan:=true -p approx_sync:=true -r scan:=/scan -r odom:=/odom_icp"
  wait_for 60 "TF map -> base_footprint exists" tf_ok map base_footprint
}

do_map() {
  say "auto_map: driving the 11-waypoint route (about 8 min). Do not touch the robot."
  # guard at the A1 limit: a map more than 0.15 m / 5 deg off the world fails here, not 3 h into A2
  python3 "$KIT/ros_tools/auto_map.py" --guard-m "${MAP_GUARD_M:-0.15}" --guard-deg "${MAP_GUARD_DEG:-5}" 2>&1 | tee "$LOG/auto_map.log"
  local rc=${PIPESTATUS[0]}
  if [ "$rc" -ne 0 ] || ! grep -q "11 reached" "$LOG/auto_map.log"; then
    fail "auto_map did not reach all 11 waypoints (exit $rc). Do not lock this map; run ./bringup.sh all again."
  fi
  ok "auto_map reached 11 waypoints"
  # Before locking: if the robot is off at home, revisit the start area so RTAB-Map can close the loop.
  local i
  for i in 1 2 3; do
    yes "" | timeout 90 python3 "$KIT/ros_tools/check_frames.py" --model arc --samples 2 --pos-tol 0.05 --yaw-tol 3 \
      2>&1 | tee "$LOG/premap_check_$i.log" | tail -1
    if grep -q "^PASS" "$LOG/premap_check_$i.log"; then ok "map end agrees with the world within 0.05 m"; break; fi
    say "map end is off; revisit $i/3 of the start area so RTAB-Map can close the loop"
    python3 "$KITDIR/arc_tests/close_loop.py" "$LOG/close_loop_$i.csv" 2>&1 | tee -a "$LOG/close_loop.log" | tail -2
  done
  lock_map
  check_map
}

check_map() {
  say "checking the locked map against Gazebo truth (check_frames.py)"
  # check_frames waits for Enter between samples; answer it automatically
  yes "" | timeout 90 python3 "$KIT/ros_tools/check_frames.py" --model arc --samples 3 2>&1 | tee "$LOG/check_frames.log" | tail -3
  grep -q "^PASS" "$LOG/check_frames.log" && ok "map agrees with the world within 0.15 m / 5 deg" \
    || fail "map is off the world by more than 0.15 m / 5 deg. Re-map: ./bringup.sh all"
}

lock_map() {
  local i
  for i in 1 2 3; do
    # the service is /rtabmap/set_mode_localization on some RTAB-Map builds and
    # /rtabmap/rtabmap/set_mode_localization on others: use whichever exists
    local svc
    svc=$(timeout 15 ros2 service list 2>/dev/null | grep -m1 'rtabmap.*/set_mode_localization$')
    [ -z "$svc" ] && { say "set_mode_localization service not listed yet"; sleep 3; continue; }
    say "calling $svc"
    if timeout 45 ros2 service call "$svc" std_srvs/srv/Empty 2>&1 | tee -a "$LOG/auto_map.log" | grep -q "Empty_Response"; then
      ok "map locked (localization mode)"; return 0
    fi
    say "lock attempt $i did not answer; retrying"
    sleep 3
  done
  fail "could not lock the map. Run: ./bringup.sh lock   (the map is still there; do not re-map)"
}

do_nav() {
  # never run two copies of a navigation node: stop any that are already up
  for p in controller_server lifecycle_manager dijkstra_planner planned_path_to_teb person_assistant_bt \
           arc_motion_gate person_state_input; do
    pkill -f "$p" 2>/dev/null && say "stopped running $p (restarting it once)"
  done
  sleep 3
  grep -n "max_vel_theta" "$TEB_YAML" | sed 's/^/    teb_controller.yaml: /'
  start_win B1_controller "ros2 run nav2_controller controller_server --ros-args --params-file $TEB_YAML -r cmd_vel:=/cmd_vel_nav"
  sleep 4
  start_win B2_lifecycle "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_controller -p use_sim_time:=true -p autostart:=true -p node_names:=\"['controller_server']\""
  wait_for 60 "controller_server is active" bash -c "ros2 lifecycle get /controller_server 2>/dev/null | grep -q '^active'"
  say "TEB limits loaded by the running controller:"
  for p in max_vel_x max_vel_theta acc_lim_theta; do
    v=$(timeout 6 ros2 param get /controller_server TEB.$p 2>/dev/null | sed 's/.*value is: //')
    echo "    TEB.$p = ${v:-not reported}"
  done
  for p in required_movement_radius movement_time_allowance; do
    v=$(timeout 6 ros2 param get /controller_server progress_checker.$p 2>/dev/null | sed 's/.*value is: //')
    echo "    progress_checker.$p = ${v:-not reported}   (Nav2 default 0.5 m / 10 s; phase-2 variant 0.25 m / 20 s)"
  done
  say "controller params file: $TEB_YAML"
  timeout 8 ros2 param list /controller_server 2>/dev/null | grep -i "theta" | sed 's/^/    param: /' > "$LOG/teb_theta_params.txt"
  [ -s "$LOG/teb_theta_params.txt" ] && cat "$LOG/teb_theta_params.txt"
  start_win B3_dijkstra "ros2 run arc_planner dijkstra_planner --ros-args -p use_sim_time:=true"
  start_win B4_path_to_teb "ros2 run arc_planner planned_path_to_teb --ros-args -p use_sim_time:=true"
  start_win B5_bt_stack "ros2 launch arc_behavior_tree arc_bt_stack.launch.py"
  sleep 5
  if [ "${PERSON_INPUT:-keyboard}" = camera ]; then
    # The webcam perception bridge (on the Pi, or on this laptop) publishes /person_moving and
    # /person_confused instead of B6. Two sources at once would fight, so B6 is not started.
    say "PERSON_INPUT=camera: B6 person_state_input NOT started; the BT mode comes from the webcam."
    say "Start it on the Pi:  ~/arc_pi/pi/pi_up.sh perception   (enrol: ~/arc_pi/laptop/arc_ui.py)"
    wait_for 40 "BT is publishing its mode" bash -c "timeout 4 ros2 topic echo --once /arc/behavior_mode --field data >/dev/null 2>&1"
  else
    # "3" = LEAD, typed for you; the window stays open so you can type another state
    start_win B6_person_state "(sleep 3; echo 3; cat) | ros2 run arc_behavior_tree person_state_input"
    wait_for 40 "BT mode is LEAD" bash -c "timeout 4 ros2 topic echo --once /arc/behavior_mode --field data 2>/dev/null | grep -q LEAD"
  fi
  wait_for 30 "planner has a map (/planned_path topic exists)" bash -c "ros2 topic list | grep -q /planned_path"
  ok "navigation stack is up. Next: cd ~/arc_test_kit && ./run.sh probe"
}

do_status() {
  echo "session logs: $(cat "$SESSION_FILE" 2>/dev/null || echo none)"
  for p in "gz sim" parameter_bridge icp_odometry "rtabmap_slam" controller_server lifecycle_manager dijkstra_planner \
           planned_path_to_teb person_assistant_bt arc_motion_gate person_state_input shopping_manager mission_runner.py; do
    if pgrep -f "$p" >/dev/null; then echo "  running  $p"; else echo "  -        $p"; fi
  done
}

case "${1:-}" in
  stop)   do_stop ;;
  sim)    do_sim ;;
  slam)   do_slam ;;
  map)    do_map ;;
  lock)   lock_map; check_map ;;
  check)  check_map ;;
  nav)    do_nav ;;
  all)    do_stop; mkdir -p "$LOG"; do_sim; do_slam; do_map; do_nav ;;
  all-wheel)  # B1 stack: same as all, wheel odometry only. The mapping drive's guard is relaxed
              # (0.6 m / 20 deg) because wheel-only drift is what B1 measures; the locked map must
              # still pass the 0.15 m check, so B1 navigation is compared on an equally good map.
          export ODOM=wheel MAP_GUARD_M=${MAP_GUARD_M:-0.6} MAP_GUARD_DEG=${MAP_GUARD_DEG:-20}
          do_stop; mkdir -p "$LOG"; do_sim; do_slam; do_map; do_nav ;;
  status) do_status ;;
  *) sed -n '2,15p' "$0"; exit 1 ;;
esac
