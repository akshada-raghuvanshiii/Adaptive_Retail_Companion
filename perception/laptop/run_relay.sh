#!/usr/bin/env bash
# Run ON YOUR LAPTOP (after bringup.sh, with NO B6 person_state_input window open):
#   ~/arc_pi_udp/laptop/run_relay.sh            accept the Pi from any address
#   PI_IP=172.20.10.2 ~/arc_pi_udp/laptop/run_relay.sh
HERE=$(cd "$(dirname "$0")" && pwd)
source ~/arc_test_kit/env.sh
pgrep -f person_state_input >/dev/null && { echo "[STOP] person_state_input (B6) is running: it publishes the same flags. Close it first."; exit 1; }
echo "This laptop's addresses: $(hostname -I)   <- give the hotspot one (172.20.10.x) to run_on_pi.sh"
if command -v ufw >/dev/null && sudo -n ufw status 2>/dev/null | grep -q "Status: active"; then
  echo "[WARN] ufw firewall is active: allow the Pi with  sudo ufw allow 5600/udp"
fi
exec python3 "$HERE/perception_relay.py" ${PI_IP:+--pi-ip "$PI_IP"}
