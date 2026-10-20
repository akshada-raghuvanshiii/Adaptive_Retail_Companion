#!/usr/bin/env bash
# Records the exact state of the system before a test, for reproducibility.
# Usage: snapshot.sh OUT_DIR TEST_NAME "full command"
# Writes OUT_DIR/run_logs/<time>_<test>/ and appends a header to OUT_DIR/LAB_NOTEBOOK.md.
OUT="$1"; TEST="$2"; CMD="$3"
TS=$(date +%Y%m%d_%H%M%S)
D="$OUT/run_logs/${TS}_${TEST}"
mkdir -p "$D"
WS=${WS:-$HOME/slam_ws}
{
  echo "test: $TEST"
  echo "started: $(date '+%Y-%m-%d %H:%M:%S %Z')"
  echo "command: $CMD"
  echo "host: $(hostname)  user: $(whoami)"
  echo "cpu: $(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2 | xargs)  cores: $(nproc 2>/dev/null)"
  echo "ram: $(free -h 2>/dev/null | awk '/Mem:/{print $2}')"
  echo "os: $(. /etc/os-release 2>/dev/null; echo "$PRETTY_NAME")"
  echo "ros: ${ROS_DISTRO:-not sourced}"
  echo "gazebo: $(gz sim --version 2>/dev/null | head -1)"
} > "$D/system.txt"

# Source state: git commit if the package is a repo, otherwise file hashes.
{
  for p in "$WS"/src/*/; do
    name=$(basename "$p")
    if git -C "$p" rev-parse HEAD >/dev/null 2>&1; then
      echo "$name git $(git -C "$p" rev-parse --short HEAD) dirty_files=$(git -C "$p" status --porcelain | wc -l)"
    fi
  done
  echo "--- md5 of key files"
  find "$WS/src" -maxdepth 4 \( -name 'dijkstra_planner*.py' -o -name 'planned_path_to_teb*.py' \
       -o -name 'shopping_manager*.py' -o -name '*controller*.yaml' -o -name 'item_locations.yaml' \
       -o -name '*.xml' -path '*behavior*' \) -not -path '*/build/*' -not -path '*/install/*' 2>/dev/null \
       | sort | xargs -r md5sum
} > "$D/source_state.txt"

# Copies of the configs this run used
mkdir -p "$D/configs"
find "$WS/src" -maxdepth 4 \( -name '*controller*.yaml' -o -name 'item_locations.yaml' \) \
     -not -path '*/build/*' -not -path '*/install/*' 2>/dev/null | while read -r f; do
  cp "$f" "$D/configs/$(basename "$(dirname "$(dirname "$f")")")__$(basename "$f")" 2>/dev/null
done
[ -d "$(dirname "$0")/../configs" ] && cp -r "$(dirname "$0")/../configs/." "$D/configs/" 2>/dev/null

# Map database
DB=$HOME/.ros/rtabmap.db
if [ -f "$DB" ]; then
  echo "rtabmap.db md5=$(md5sum "$DB" | cut -c1-12) size=$(du -h "$DB" | cut -f1) modified=$(date -r "$DB" '+%Y-%m-%d %H:%M')" > "$D/map_state.txt"
else
  echo "rtabmap.db not found" > "$D/map_state.txt"
fi

# Live graph and simulation speed
timeout 8 ros2 node list > "$D/nodes.txt" 2>/dev/null
timeout 8 ros2 topic list -t > "$D/topics.txt" 2>/dev/null
RTF=$(timeout 6 gz topic -e -t /stats -n 1 2>/dev/null | grep -m1 real_time_factor | awk '{print $2}')
[ -z "$RTF" ] && RTF=$(timeout 6 gz topic -e -t /world/arc_store/stats -n 1 2>/dev/null | grep -m1 real_time_factor | awk '{print $2}')
echo "${RTF:-unknown}" > "$D/real_time_factor.txt"

# Which controller and odometry are actually running (A3 / B1 must be labelled right)
CTRL=$(timeout 8 ros2 param get /controller_server TEB.plugin 2>/dev/null | sed 's/.*value is: //')
echo "${CTRL:-unknown}" > "$D/controller_plugin.txt"
timeout 8 ros2 node info /rtabmap/rtabmap > "$D/rtabmap_node_info.txt" 2>/dev/null
if grep -q "/odom_icp" "$D/rtabmap_node_info.txt" 2>/dev/null; then ODOM=icp
elif grep -q "/odom:" "$D/rtabmap_node_info.txt" 2>/dev/null; then ODOM=wheel
else ODOM=unknown; fi
echo "$ODOM" > "$D/odometry.txt"
MODE=$(timeout 4 ros2 topic echo --once /arc/behavior_mode --field data 2>/dev/null | head -1)
echo "${MODE:-unknown}" > "$D/bt_mode.txt"

{
  echo ""
  echo "## $(date '+%Y-%m-%d %H:%M') - $TEST"
  echo ""
  echo "- Command: \`$CMD\`"
  echo "- Gazebo real-time factor at start: ${RTF:-unknown}"
  echo "- Controller: ${CTRL:-unknown}; RTAB-Map odometry: $ODOM; BT mode: ${MODE:-unknown}"
  echo "- Map: $(cat "$D/map_state.txt")"
  echo "- Snapshot: \`$D\`"
  echo "- Observations: (add with ./run.sh note \"...\")"
} >> "$OUT/LAB_NOTEBOOK.md"
echo "$D"
