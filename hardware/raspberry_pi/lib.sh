# shellcheck shell=bash
# Shared helpers for the arc_pi scripts (sourced, not run).
ARC_PI_DIR=${ARC_PI_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}
CONF=${ARC_PI_CONF:-$ARC_PI_DIR/hil.conf}
# shellcheck source=hil.conf
[ -f "$CONF" ] && source "$CONF"
LINK=${LINK:-cable}
# hil.peer: written on the Pi by the laptop's deploy.sh (LAPTOP_HOST, LAPTOP_IP_SEEN)
[ -f "$ARC_PI_DIR/hil.peer" ] && source "$ARC_PI_DIR/hil.peer"

say()  { echo -e "\033[1;36m[arc_pi]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[  OK  ]\033[0m $*"; }
warn() { echo -e "\033[1;33m[ WARN ]\033[0m $*"; }
bad()  { echo -e "\033[1;31m[ FAIL ]\033[0m $*"; }
die()  { bad "$*"; exit 1; }

# ask "question"  -> 0 if the user typed y/Y. ARC_PI_YES=1 answers yes (only for tests).
ask() {
  [ "${ARC_PI_YES:-0}" = 1 ] && return 0
  local r
  read -r -p "$1 [y/N] " r
  [[ "$r" =~ ^[Yy]$ ]]
}

is_ipv4() { [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; }
has_ip() { [ -n "$1" ] && ip -4 addr show 2>/dev/null | grep -q "inet $1/"; }
lo_multicast_ok() { ip link show lo 2>/dev/null | grep -q MULTICAST; }

# IPv4 address of this machine's Wi-Fi interface (wl*), empty if not connected
wifi_ip() {
  ip -4 -o addr show 2>/dev/null | awk '$2 ~ /^wl/ {split($4, a, "/"); print a[1]; exit}'
}

# resolve NAME -> IPv4 (an IP is returned as is; .local names need mDNS on this machine)
resolve() {
  local h=$1
  [ -n "$h" ] || return 1
  if is_ipv4 "$h"; then echo "$h"; return 0; fi
  getent ahostsv4 "$h" 2>/dev/null | awk '{print $1; exit}' | grep . || return 1
}

# own_ip SIDE / peer_ip SIDE   (SIDE = laptop | pi)
own_ip() {
  if [ "$LINK" = cable ]; then [ "$1" = pi ] && echo "$PI_IP" || echo "$LAPTOP_IP"; return; fi
  wifi_ip
}
peer_ip() {
  if [ "$LINK" = cable ]; then [ "$1" = pi ] && echo "$LAPTOP_IP" || echo "$PI_IP"; return; fi
  if [ "$1" = pi ]; then resolve "${LAPTOP_HOST:-}" || { [ -n "${LAPTOP_IP_SEEN:-}" ] && echo "$LAPTOP_IP_SEEN"; }
  else resolve "$PI_HOST"; fi
}
peer_name() { [ "$1" = pi ] && echo "laptop (${LAPTOP_HOST:-unknown: run deploy.sh})" || echo "Pi ($PI_HOST)"; }

# make_profile OWN_IP [PEER_IP] -> ~/.arc_hil/fastdds_hil.xml
#   interfaces: loopback + shared memory + OWN_IP only.
#   PEER_IP given: discovery also sent by unicast to it (hotspots that drop multicast still work).
make_profile() {
  local ip=$1 peer=${2:-}
  is_ipv4 "$ip" || { bad "no usable address for this machine: '$ip' (not on the hotspot / cable down?)"; return 1; }
  [ -z "$peer" ] || is_ipv4 "$peer" || { bad "peer address is not IPv4: '$peer'"; return 1; }
  mkdir -p "$HOME/.arc_hil"
  local mport=$((7400 + 250 * ${ROS_DOMAIN_ID:-0})) peers=""
  if [ -n "$peer" ]; then
    peers="      <builtin>
        <initialPeersList>
          <locator><udpv4><address>239.255.0.1</address><port>$mport</port></udpv4></locator>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          <locator><udpv4><address>$peer</address></udpv4></locator>
        </initialPeersList>
      </builtin>"
  fi
  awk -v ip="$ip" -v peers="$peers" '
    /@PEERS@/ { if (peers != "") print peers; next }
    { gsub(/@OWN_IP@/, ip); print }' "$ARC_PI_DIR/fastdds_hil.xml.in" > "$HOME/.arc_hil/fastdds_hil.xml"
  echo "$ip ${peer:--}" > "$HOME/.arc_hil/made_for"
}

# profile_own_ip / profile_peer_ip: what the current profile was made with
profile_own_ip()  { awk '{print $1}' "$HOME/.arc_hil/made_for" 2>/dev/null; }
profile_peer_ip() { awk '$2 != "-" {print $2}' "$HOME/.arc_hil/made_for" 2>/dev/null; }
