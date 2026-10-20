#!/usr/bin/env bash
# Put arc_ui_bridge into ~/slam_ws/src and build ONLY this package. Run on the laptop.
#   ./install_into_slam_ws.sh            install / update
#   WS=~/other_ws ./install_into_slam_ws.sh
# - An existing ~/slam_ws/src/arc_ui_bridge is moved to ~/arc_backups/arc_ui_bridge_<time> first: OUTSIDE the
#   workspace, because colcon searches the whole workspace and would find two packages with the same name.
# - No other package is touched or rebuilt. No apt/pip packages are installed: missing ones are printed.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/slam_ws}
DST=$WS/src/arc_ui_bridge
[ -d "$WS/src" ] || { echo "[FAIL] no $WS/src"; exit 1; }
[ -f /opt/ros/jazzy/setup.bash ] || { echo "[FAIL] ROS 2 Jazzy not found in /opt/ros/jazzy"; exit 1; }

if [ "$HERE" != "$DST" ]; then
  if [ -e "$DST" ]; then
    B=$HOME/arc_backups/arc_ui_bridge_$(date +%Y%m%d_%H%M%S)
    mkdir -p "$HOME/arc_backups"
    mv "$DST" "$B"
    touch "$B/COLCON_IGNORE"
    echo "[ OK ] old copy moved to $B"
  fi
  mkdir -p "$DST"
  cp -r "$HERE"/. "$DST"/
  find "$DST" -name __pycache__ -prune -exec rm -rf {} +
  echo "[ OK ] package copied to $DST"
fi

# Older installs (16 Oct) put backups in $WS/src_backups, where colcon finds them: move them out.
if [ -d "$WS/src_backups" ]; then
  mkdir -p "$HOME/arc_backups"
  mv "$WS/src_backups" "$HOME/arc_backups/slam_ws_src_backups_$(date +%Y%m%d_%H%M%S)"
  echo "[ OK ] moved old $WS/src_backups to ~/arc_backups (colcon saw duplicate packages there)"
fi
n=$(find "$WS" -name package.xml -not -path "*/install/*" -not -path "*/build/*" -exec grep -l "<name>arc_ui_bridge</name>" {} + 2>/dev/null | wc -l)
[ "$n" -le 1 ] || { echo "[FAIL] $n copies of arc_ui_bridge inside $WS (colcon allows one):"; \
  find "$WS" -name package.xml -not -path "*/install/*" -not -path "*/build/*" -exec grep -l "<name>arc_ui_bridge</name>" {} +; exit 1; }

set +e
source /opt/ros/jazzy/setup.bash
( cd "$WS" && colcon build --packages-select arc_ui_bridge ) || { echo "[FAIL] colcon build failed (output above)"; exit 1; }
echo "[ OK ] built arc_ui_bridge (only this package)"

echo "offline tests:"
( cd "$DST" && python3 test/run_tests.py 2>&1 | tail -1 ) | sed 's/^/  /'

python3 -c "import yaml" 2>/dev/null && echo "[ OK ] python3-yaml present" \
  || echo "[TODO] python3-yaml missing:  sudo apt install python3-yaml"
python3 -c "import action_msgs.srv, tf2_ros" 2>/dev/null && echo "[ OK ] action_msgs and tf2_ros present" \
  || echo "[TODO] action_msgs / tf2_ros not importable (they come with ROS 2 Jazzy desktop)"
[ -f "$WS/src/arc_store_sim/config/item_locations.yaml" ] && echo "[ OK ] store item_locations.yaml found" \
  || echo "[TODO] $WS/src/arc_store_sim/config/item_locations.yaml not found: pass item_locations:=<path>"
echo
echo "Next: bring up the stack (cd ~/arc_test_kit && ./bringup.sh all), then in a new terminal:"
echo "  source $WS/install/setup.bash && ros2 launch arc_ui_bridge arc_ui.launch.py"
echo "and open http://localhost:8080"
