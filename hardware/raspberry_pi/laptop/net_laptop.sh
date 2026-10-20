#!/usr/bin/env bash
# Laptop side of the link to the Pi.
#
#   ./net_laptop.sh up      hotspot: check the laptop is on Wi-Fi and the Pi answers
#                           cable:   create (first time, asks first) and activate the "arc-hil" profile
#   ./net_laptop.sh down    cable only: deactivate the profile (it stays for next time)
#   ./net_laptop.sh status
#
# Hotspot mode changes nothing on the laptop. Cable mode adds one NetworkManager profile with a static
# address (LAPTOP_IP from hil.conf), no gateway and no DNS, so it never becomes the internet route.
# No packages are installed.
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"

eth_dev() {   # first wired device NetworkManager knows (built-in port or USB adapter)
  nmcli -t -f DEVICE,TYPE device 2>/dev/null | awk -F: '$2=="ethernet"{print $1; exit}'
}

hotspot_report() {
  local ip ssid pi
  ip=$(wifi_ip)
  ssid=$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | awk -F: '$1=="yes"{print $2; exit}')
  [ -n "$ip" ] && ok "laptop on Wi-Fi ${ssid:+'$ssid' }at $ip" || { bad "laptop has no Wi-Fi address: join the phone hotspot"; return 1; }
  pi=$(resolve "$PI_HOST")
  if [ -z "$pi" ]; then
    warn "cannot find $PI_HOST. Pi still booting, not on this hotspot, or its mDNS is off (pi_setup.sh mdns)."
    echo "      Or read the Pi's address on the phone (hotspot > connected devices) and put PI_HOST=<that IP> in hil.conf."
    return 1
  fi
  if ping -c 2 -W 1 "$pi" >/dev/null 2>&1; then ok "Pi answers at $pi ($PI_HOST)"
  else
    bad "Pi found at $pi but does not answer ping."
    echo "      Some phones isolate hotspot clients from each other. Look for 'client isolation' / 'AP isolation'"
    echo "      in the hotspot settings and turn it off, or use the Ethernet cable (LINK=cable in hil.conf)."
    return 1
  fi
}

if [ "$LINK" = hotspot ]; then
  case "${1:-status}" in
    up|status) hotspot_report ;;
    down) say "hotspot mode: nothing to switch off here" ;;
    *) echo "usage: $0 up|down|status"; exit 2 ;;
  esac
  exit $?
fi

command -v nmcli >/dev/null || die "nmcli not found (NetworkManager). Set $LAPTOP_IP/$PREFIX on the Ethernet port by hand."
case "${1:-status}" in
  up)
    dev=$(eth_dev)
    [ -n "$dev" ] || die "no Ethernet device found. Laptops without a port need a USB-Ethernet adapter."
    if ! nmcli -t -f NAME connection show | grep -qx arc-hil; then
      say "This creates a NetworkManager profile 'arc-hil' on $dev:"
      say "  address $LAPTOP_IP/$PREFIX, no gateway, no DNS, IPv6 off, never the default route."
      say "  Why: a fixed address for the Pi cable; nothing else on the laptop changes."
      ask "Create it?" || die "not created"
      nmcli connection add type ethernet ifname "$dev" con-name arc-hil \
        ipv4.method manual ipv4.addresses "$LAPTOP_IP/$PREFIX" ipv4.never-default yes \
        ipv4.ignore-auto-dns yes ipv6.method disabled connection.autoconnect yes >/dev/null \
        || die "nmcli could not create the profile"
      ok "profile arc-hil created on $dev"
    fi
    nmcli connection up arc-hil >/dev/null 2>&1 || die "could not bring arc-hil up (is the cable plugged in at both ends and the Pi on?)"
    sleep 1
    has_ip "$LAPTOP_IP" && ok "laptop has $LAPTOP_IP on $dev" || die "address $LAPTOP_IP did not appear"
    if ping -c 2 -W 1 "$PI_IP" >/dev/null 2>&1; then ok "Pi answers at $PI_IP"
    else warn "Pi does not answer at $PI_IP yet (booting, or its static IP is not set: pi_setup.sh step 'network')"; fi
    ;;
  down)
    nmcli connection down arc-hil && ok "arc-hil down" ;;
  status)
    nmcli -f NAME,DEVICE,STATE connection show | grep -E '^NAME|arc-hil' || echo "no arc-hil profile yet"
    has_ip "$LAPTOP_IP" && ok "laptop has $LAPTOP_IP" || warn "laptop has no $LAPTOP_IP"
    ping -c 1 -W 1 "$PI_IP" >/dev/null 2>&1 && ok "Pi answers at $PI_IP" || warn "no answer from $PI_IP"
    ;;
  *) echo "usage: $0 up|down|status"; exit 2 ;;
esac
