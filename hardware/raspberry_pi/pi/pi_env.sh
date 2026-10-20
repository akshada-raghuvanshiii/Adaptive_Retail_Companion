# shellcheck shell=bash
# Source on the Pi in every terminal that talks to the laptop:  source ~/arc_pi/pi/pi_env.sh
# ROS 2 on loopback + shared memory + the link to the laptop only (hotspot Wi-Fi or cable, from hil.conf),
# matching the laptop's HIL mode. The profile is remade each time, so a new hotspot address is picked up.
ARC_PI_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  warn "ROS 2 Jazzy not installed yet: ~/arc_pi/pi/pi_setup.sh ros"
fi
_ARC_OWN=$(own_ip pi)
if [ -n "$_ARC_OWN" ]; then
  make_profile "$_ARC_OWN" "$(peer_ip pi)"
else
  warn "this Pi has no ${LINK} address (not on the hotspot / cable down): the laptop will not see these nodes"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.arc_hil/fastdds_hil.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/.arc_hil/fastdds_hil.xml"
export ARC_NET_MODE=hil
export ARC_RUN=$HOME/arc_pi_run          # code copied by deploy.sh, logs, pids
export ARC_VENV=$HOME/arc_pi_venv
