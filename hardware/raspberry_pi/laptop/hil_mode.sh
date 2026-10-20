#!/usr/bin/env bash
# Switch the ARC laptop between laptop-only ROS 2 (default) and HIL (laptop + Pi).
#
#   ./hil_mode.sh on       write ~/.arc_net = hil and ~/.arc_hil/fastdds_hil.xml for the link in hil.conf
#                          (hotspot: this laptop's current Wi-Fi address, plus the Pi's if it can be found)
#   ./hil_mode.sh off      remove ~/.arc_net -> env.sh is exactly as before (localhost only)
#   ./hil_mode.sh status
#
# Hotspot: the phone may give a different address after reconnecting. Run "on" again after joining the
# hotspot (and after the Pi is up, so its address goes into the profile), then restart the stack.
# The switch only affects NEW terminals and nodes (env.sh reads it when sourced). After switching:
#   ~/arc_test_kit/bringup.sh stop     then bring the stack up again.
# Do not switch while a run is going: running nodes keep the old setting, new ones get the new one,
# and the two groups cannot see each other.
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"

case "${1:-status}" in
  on)
    ip=$(own_ip laptop)
    if [ -z "$ip" ]; then
      [ "$LINK" = hotspot ] && die "this laptop has no Wi-Fi address. Join the phone hotspot first."
      die "no LAPTOP_IP in hil.conf"
    fi
    peer=$(peer_ip laptop)
    make_profile "$ip" "$peer" || exit 1
    echo hil > "$HOME/.arc_net"
    ok "HIL mode ON ($LINK): ROS 2 uses loopback, shared memory and $ip. Other interfaces stay excluded."
    if [ -n "$peer" ]; then ok "Pi at $peer: discovery also goes to it directly"
    else warn "Pi ($PI_HOST) not found now: discovery by multicast only. Run 'hil_mode.sh on' again once the Pi is up."; fi
    [ "$LINK" = cable ] && ! has_ip "$ip" && warn "this laptop has no $ip yet: run ./net_laptop.sh up before bringup"
    say "New terminals only. Next: ~/arc_test_kit/bringup.sh stop, then bring the stack up again."
    ;;
  off)
    rm -f "$HOME/.arc_net"
    ok "HIL mode OFF: back to laptop-only ROS 2 (fastdds_localhost.xml). New terminals only."
    ;;
  status)
    m=local; [ -f "$HOME/.arc_net" ] && m=$(tr -d '[:space:]' < "$HOME/.arc_net")
    echo "mode: $m   link: $LINK"
    if [ "$m" = hil ]; then
      p=$(profile_own_ip); q=$(profile_peer_ip)
      echo "profile: $HOME/.arc_hil/fastdds_hil.xml  (this laptop ${p:-?}, Pi ${q:-not in profile})"
      has_ip "$p" && ok "laptop still has $p" || warn "laptop no longer has $p: run 'hil_mode.sh on' again, then restart the stack"
      now=$(peer_ip laptop)
      [ -n "$q" ] && [ -n "$now" ] && [ "$now" != "$q" ] && warn "the Pi is now at $now (profile has $q): run 'hil_mode.sh on' again"
    fi
    echo "this terminal: ARC_NET_MODE=${ARC_NET_MODE:-<env.sh not sourced>}"
    ;;
  *) echo "usage: $0 on|off|status"; exit 2 ;;
esac
