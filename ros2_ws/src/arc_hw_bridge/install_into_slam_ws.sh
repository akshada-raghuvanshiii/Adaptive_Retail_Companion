#!/usr/bin/env bash
# Put arc_hw_bridge into ~/slam_ws/src and build ONLY this package. Run on the laptop (or the Pi).
#   ./install_into_slam_ws.sh            install / update
#   WS=~/other_ws ./install_into_slam_ws.sh
# - An existing ~/slam_ws/src/arc_hw_bridge is moved to ~/arc_backups/arc_hw_bridge_<time> first: OUTSIDE the
#   workspace, because colcon searches the whole workspace and would find two packages with the same name.
# - No other package is touched or rebuilt. No apt/pip packages are installed: missing ones are printed.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HOME/slam_ws}
DST=$WS/src/arc_hw_bridge
[ -d "$WS/src" ] || { echo "[FAIL] no $WS/src"; exit 1; }
[ -f /opt/ros/jazzy/setup.bash ] || { echo "[FAIL] ROS 2 Jazzy not found in /opt/ros/jazzy"; exit 1; }

if [ "$HERE" != "$DST" ]; then
  if [ -e "$DST" ]; then
    B=$HOME/arc_backups/arc_hw_bridge_$(date +%Y%m%d_%H%M%S)
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
n=$(find "$WS" -name package.xml -not -path "*/install/*" -not -path "*/build/*" -exec grep -l "<name>arc_hw_bridge</name>" {} + 2>/dev/null | wc -l)
[ "$n" -le 1 ] || { echo "[FAIL] $n copies of arc_hw_bridge inside $WS (colcon allows one):"; \
  find "$WS" -name package.xml -not -path "*/install/*" -not -path "*/build/*" -exec grep -l "<name>arc_hw_bridge</name>" {} +; exit 1; }

set +e
source /opt/ros/jazzy/setup.bash
( cd "$WS" && colcon build --packages-select arc_hw_bridge ) || { echo "[FAIL] colcon build failed (output above)"; exit 1; }
echo "[ OK ] built arc_hw_bridge (only this package)"

echo "offline tests:"
( cd "$DST" && python3 -m unittest discover -s test 2>&1 | tail -1 ) | sed 's/^/  /'

python3 -c "import serial" 2>/dev/null && echo "[ OK ] python3-serial present" \
  || echo "[TODO] python3-serial missing (talks to the Uno over USB):  sudo apt install python3-serial"
id -nG | grep -qw dialout && echo "[ OK ] you are in the dialout group" \
  || echo "[TODO] not in 'dialout' (needed to open /dev/ttyACM0):  sudo usermod -aG dialout \$USER  then log out and in"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sed 's/^/[INFO] serial port: /' || true
echo
echo "Next, in a new terminal:  source ~/arc_test_kit/env.sh && ros2 run arc_hw_bridge motor_bench check"
