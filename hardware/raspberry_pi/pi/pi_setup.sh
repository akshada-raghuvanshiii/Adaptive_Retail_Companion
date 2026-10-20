#!/usr/bin/env bash
# One-time Raspberry Pi setup for ARC hardware-in-the-loop. Run ON THE PI:
#   ~/arc_pi/pi/pi_setup.sh            all steps, each explained and confirmed (y/N)
#   ~/arc_pi/pi/pi_setup.sh <step>     one step: check ros apt groups venv lo network mdns camera
#
# Safe to run again: finished steps are detected and skipped. Nothing is installed without a "y".
# Needs internet on the Pi (Wi-Fi set in Raspberry Pi Imager) for the ros, apt and venv steps.
ARC_PI_DIR=$(cd "$(dirname "$0")/.." && pwd)
source "$ARC_PI_DIR/lib.sh"
VENV=$HOME/arc_pi_venv

step_check() {
  say "== check: OS, CPU, memory, disk =="
  . /etc/os-release
  local arch; arch=$(uname -m)
  echo "  OS: $PRETTY_NAME, arch: $arch"
  echo "  board: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
  echo "  RAM: $(awk '/MemTotal/{printf "%.1f GB", $2/1048576}' /proc/meminfo), free disk in ~: $(df -h --output=avail "$HOME" | tail -1 | tr -d ' ')"
  if [ "${VERSION_ID:-}" != "24.04" ] || [ "${ID:-}" != ubuntu ]; then
    bad "ROS 2 Jazzy binary packages need Ubuntu 24.04. This is $PRETTY_NAME."
    echo "     Flash 'Ubuntu Server 24.04 LTS (64-bit)' with Raspberry Pi Imager (Other general-purpose OS > Ubuntu)."
    echo "     Raspberry Pi OS has no Jazzy packages (it would need building ROS from source or Docker)."
    exit 1
  fi
  [ "$arch" = aarch64 ] || die "needs the 64-bit image (aarch64), this is $arch"
  local kb; kb=$(df -k --output=avail "$HOME" | tail -1)
  [ "$kb" -gt 6000000 ] || warn "under 6 GB free: ROS (~1 GB) + PyTorch/YOLO venv (~2-3 GB) may not fit"
  ok "Ubuntu 24.04 aarch64"
}

step_ros() {
  say "== ros: ROS 2 Jazzy (ros-base) =="
  if [ -f /opt/ros/jazzy/setup.bash ]; then ok "already installed (/opt/ros/jazzy)"; return; fi
  echo "  Installs: the ROS 2 apt source (ros2-apt-source package from ros-infrastructure) and"
  echo "            ros-jazzy-ros-base: rclpy and the std/geometry/sensor/visualization message types."
  echo "  Why: both Pi nodes (motor bridge, webcam bridge) are ROS 2 nodes that talk to the laptop's"
  echo "       stack. No GUI tools, no Gazebo, no navigation packages go on the Pi. About 1 GB."
  ask "Install ROS 2 Jazzy ros-base now?" || { warn "skipped"; return; }
  set -e
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  local v codename
  v=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
  [ -n "$v" ] || { set +e; die "could not read the ros-apt-source version from GitHub (internet?)"; }
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${v}/ros2-apt-source_${v}.${codename}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp
  set +e
  ok "ROS 2 Jazzy installed"
}

step_apt() {
  say "== apt: small system packages =="
  local need=()
  for p in python3-serial python3-venv python3-pip v4l-utils rsync; do
    dpkg -s "$p" >/dev/null 2>&1 || need+=("$p")
  done
  if [ ${#need[@]} -eq 0 ]; then ok "all present"; return; fi
  echo "  Installs: ${need[*]}"
  echo "  Why: python3-serial = USB serial to the Uno (motor bridge, bench.py);"
  echo "       python3-venv/pip = a separate Python environment for YOLO (kept out of ROS and out of"
  echo "       arc_person_follower); v4l-utils = list the webcam's modes; rsync = deploy.sh copies code."
  ask "Install them?" || { warn "skipped"; return; }
  sudo apt-get install -y "${need[@]}" && ok "installed"
}

step_groups() {
  say "== groups: serial and camera access without sudo =="
  local add=()
  id -nG | grep -qw dialout || add+=(dialout)
  id -nG | grep -qw video || add+=(video)
  if [ ${#add[@]} -eq 0 ]; then ok "$USER is in dialout and video"; return; fi
  echo "  Adds $USER to: ${add[*]}  (dialout = /dev/ttyACM0 Uno, video = /dev/video0 webcam)"
  ask "Add?" || { warn "skipped"; return; }
  for g in "${add[@]}"; do sudo usermod -aG "$g" "$USER"; done
  warn "log out and back in (or reboot) before it takes effect"
}

step_venv() {
  say "== venv: YOLO environment at $VENV =="
  if [ -x "$VENV/bin/python" ] && "$VENV/bin/python" -c "import ultralytics, lap, cv2" 2>/dev/null; then
    ok "ready ($("$VENV/bin/python" -c 'import ultralytics;print("ultralytics", ultralytics.__version__)'))"
  else
    echo "  Creates a Python venv that can also see the system ROS packages (--system-site-packages),"
    echo "  then pip installs: ultralytics (YOLOv8 + ByteTrack; pulls PyTorch CPU and opencv-python),"
    echo "  lap (ByteTrack's assignment solver, otherwise auto-installed on first run)."
    echo "  Why: the same detector and tracker as the laptop perception tests, without touching apt"
    echo "       Python or any ROS package. Download about 1 GB, about 2-3 GB on disk, 10-30 min."
    ask "Create the venv and install?" || { warn "skipped"; return; }
    python3 -m venv --system-site-packages "$VENV" || die "venv failed (python3-venv installed? step apt)"
    "$VENV/bin/pip" install --upgrade pip
    "$VENV/bin/pip" install ultralytics lap || die "pip install failed"
    ok "venv ready"
  fi
  if ! "$VENV/bin/python" -c "import ncnn" 2>/dev/null; then
    echo "  OPTIONAL: ncnn (pip). Ultralytics' recommended runtime on Raspberry Pi; pi_perf.py --export-ncnn"
    echo "  converts yolov8n.pt and measures both. Skip it if PyTorch speed is enough."
    ask "Install ncnn (optional)?" && "$VENV/bin/pip" install ncnn && ok "ncnn installed"
  fi
}

step_lo() {
  say "== lo: multicast on the loopback interface, every boot =="
  local unit=/etc/systemd/system/arc-lo-multicast.service
  if [ -f "$unit" ] && lo_multicast_ok; then ok "service installed and lo has MULTICAST"; return; fi
  echo "  Installs a 4-line systemd service that runs 'ip link set lo multicast on' at boot."
  echo "  Why: the two Pi nodes find each other by multicast on loopback (same reason as the laptop's"
  echo "       bringup.sh check). Without it they may not see each other."
  ask "Install the service?" || { warn "skipped (pi_up.sh will remind you)"; return; }
  sudo tee "$unit" >/dev/null <<'UNIT'
[Unit]
Description=ARC: multicast on loopback for ROS 2 discovery
[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on
[Install]
WantedBy=multi-user.target
UNIT
  sudo systemctl daemon-reload && sudo systemctl enable --now arc-lo-multicast.service && ok "enabled"
}

step_network() {
  if [ "$LINK" = hotspot ]; then step_network_hotspot; return; fi
  say "== network: static $PI_IP on the Ethernet port =="
  if has_ip "$PI_IP"; then ok "Pi already has $PI_IP"; return; fi
  local dev="" n
  for n in /sys/class/net/eth* /sys/class/net/en*; do [ -e "$n" ] && { dev=${n##*/}; break; }; done
  [ -n "$dev" ] || die "no Ethernet interface found"
  local f=/etc/netplan/60-arc-hil.yaml
  echo "  Writes $f: $dev gets $PI_IP/$PREFIX (and keeps DHCP, so a router still works)."
  echo "  No gateway on this address, so the Pi's internet stays on Wi-Fi."
  ask "Write it and apply (netplan apply)?" || { warn "skipped"; return; }
  sudo tee "$f" >/dev/null <<NP
network:
  version: 2
  ethernets:
    $dev:
      dhcp4: true
      optional: true
      addresses: [$PI_IP/$PREFIX]
NP
  sudo chmod 600 "$f"
  sudo netplan apply && sleep 2
  has_ip "$PI_IP" && ok "$dev has $PI_IP" || warn "address not up yet (cable unplugged?). It applies when the link comes up."
}

step_network_hotspot() {
  say "== network: the phone hotspot (Wi-Fi) =="
  local ip; ip=$(wifi_ip)
  if [ -n "$ip" ]; then ok "Pi is on Wi-Fi at $ip"; return; fi
  local dev="" n
  for n in /sys/class/net/wl*; do [ -e "$n" ] && { dev=${n##*/}; break; }; done
  [ -n "$dev" ] || die "no Wi-Fi interface found on this Pi"
  local f=/etc/netplan/61-arc-hotspot.yaml
  echo "  The Pi is not on Wi-Fi. Easiest: set the phone hotspot's name and password as the Wi-Fi in"
  echo "  Raspberry Pi Imager when flashing. Or write it now:"
  echo "  Writes $f: $dev joins your hotspot with DHCP (the phone gives the address)."
  ask "Add the hotspot now?" || { warn "skipped"; return; }
  local ssid pw
  read -r -p "  Hotspot name (SSID): " ssid
  read -r -s -p "  Hotspot password: " pw; echo
  [ -n "$ssid" ] || { warn "no name given, skipped"; return; }
  sudo tee "$f" >/dev/null <<NP
network:
  version: 2
  wifis:
    $dev:
      dhcp4: true
      optional: true
      access-points:
        "$ssid":
          password: "$pw"
NP
  sudo chmod 600 "$f"
  sudo netplan apply && sleep 8
  ip=$(wifi_ip)
  [ -n "$ip" ] && ok "Pi is on '$ssid' at $ip" || warn "no address yet. Check the name/password, and that the hotspot is on (2.4 GHz helps)."
}

step_mdns() {
  say "== mdns: find the laptop and Pi by name on the hotspot =="
  if systemctl is-active --quiet avahi-daemon 2>/dev/null && dpkg -s libnss-mdns >/dev/null 2>&1; then
    ok "avahi-daemon running, libnss-mdns installed: this Pi is $(hostname).local"; return
  fi
  echo "  Installs: avahi-daemon and libnss-mdns (apt)."
  echo "  Why: the phone hotspot hands out addresses that can change. With these, the laptop finds the Pi as"
  echo "       $(hostname).local and the Pi finds the laptop by its name, so no address is typed anywhere."
  echo "       Without them, put the addresses in hil.conf by hand (phone: hotspot > connected devices)."
  ask "Install them?" || { warn "skipped"; return; }
  sudo apt-get install -y avahi-daemon libnss-mdns && sudo systemctl enable --now avahi-daemon \
    && ok "mDNS on: this Pi is $(hostname).local"
}

step_camera() {
  say "== camera: USB webcam =="
  ls /dev/video* >/dev/null 2>&1 || { bad "no /dev/video* (webcam not plugged in, or not a UVC camera)"; return; }
  for d in /dev/video*; do
    if command -v v4l2-ctl >/dev/null; then
      local n; n=$(v4l2-ctl -d "$d" --info 2>/dev/null | awk -F': ' '/Card type/{print $2}')
      local f; f=$(v4l2-ctl -d "$d" --list-formats 2>/dev/null | grep -o "'[A-Z0-9]*'" | tr '\n' ' ')
      [ -n "$f" ] && echo "  $d  $n  formats: $f"
    else
      echo "  $d"
    fi
  done
  echo "  The capture device is usually /dev/video0 (CAMERA=0 in hil.conf). 'MJPG' in the list means"
  echo "  --mjpg works (less USB and CPU load). Test a grab: ~/arc_pi/pi/pi_check.sh camera"
}

ALL=(check ros apt groups venv lo network mdns camera)
if [ $# -gt 0 ]; then
  for s in "$@"; do
    declare -F "step_$s" >/dev/null || die "unknown step '$s' (steps: ${ALL[*]})"
    "step_$s"
  done
else
  for s in "${ALL[@]}"; do "step_$s"; echo; done
  say "Done. Next (on the Pi):  ~/arc_pi/pi/pi_check.sh"
fi
