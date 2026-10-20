"""Offline tests for the arc_pi kit: Python tools with ROS stubbed, shell scripts with a fake HOME and
a fake `ip` command. No ROS, no Pi, no camera needed.  python3 -m unittest discover -s tests"""
import csv
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import types
import unittest
import xml.dom.minidom
from pathlib import Path
from unittest import mock

import numpy as np

KIT = Path(__file__).resolve().parents[1]
TEST_KIT = Path(os.environ.get("ARC_TEST_KIT", KIT.parent / "arc_test_kit"))
sys.path.insert(0, str(KIT / "laptop"))
sys.path.insert(0, str(KIT / "pi"))
import flag_watchdog as WD  # noqa: E402
import hil_logger as HL  # noqa: E402
import plot_hil_timeline as PT  # noqa: E402
import camera_pub as CP  # noqa: E402
import pi_perf as PP  # noqa: E402


class WatchdogTests(unittest.TestCase):
    def test_decide(self):
        self.assertTrue(WD.decide(10.0, None, 3.0, None)[0])             # never heard -> force wait
        self.assertFalse(WD.decide(10.0, 9.0, 3.0, (True, False))[0])    # fresh -> quiet
        f, why = WD.decide(10.0, 6.5, 3.0, (True, False))
        self.assertTrue(f)
        self.assertIn("3.5 s", why)


class LoggerTests(unittest.TestCase):
    def test_rows(self):
        ns = types.SimpleNamespace
        self.assertEqual(HL.row_for("/person_moving", ns(data=True))[0], 1)
        self.assertEqual(HL.row_for("/arc/behavior_mode", ns(data="LEAD"))[3], "LEAD")
        tw = ns(linear=ns(x=0.123456), angular=ns(z=-0.5))
        self.assertEqual(HL.row_for("/cmd_vel", ns(twist=tw))[:2], (0.1235, -0.5))   # TwistStamped
        self.assertEqual(HL.row_for("/cmd_vel", tw)[:2], (0.1235, -0.5))             # Twist
        js = ns(velocity=[0.1, 0.2], effort=[80.0, 90.0])
        self.assertEqual(HL.row_for("/arc_hw/wheels", js)[:3], (0.1, 0.2, 80.0))
        d = HL.row_for("/arc/target_lock", ns(data='{"fps": 3.2, "proc_ms": 290, "lock": "LOCKED", "mode": "LEAD"}'))
        self.assertEqual(d, (3.2, 290, "", "LOCKED|LEAD"))
        self.assertEqual(HL.row_for("/arc/target_lock", ns(data="{bad")), ("", "", "", "bad json"))


def synthetic_chain(path):
    """Shopper leads (confused=1) from t=2; leaves view at t=10 (flags 0); comes back at t=16.
    BT: WAIT -> LEAD 0.3 s after; LEAD -> WAIT 0.4 s after the stop; cmd_vel follows 0.2 s later;
    wheels slow down over 0.6 s (Hall lag)."""
    rows = []

    def add(t, topic, a="", b="", c="", text=""):
        rows.append([f"{t:.3f}", topic, a, b, c, text])
    t = 0.0
    while t < 22.0:
        lead = 2.0 <= t < 10.0 or t >= 16.0
        add(t, "/person_moving", 0)
        add(t, "/person_confused", 1 if lead else 0)
        t += 0.25
    for t, m in ((0.0, "WAIT"), (2.3, "LEAD"), (10.4, "WAIT"), (16.3, "LEAD")):
        add(t, "/arc/behavior_mode", text=m)
    t = 0.0
    while t < 22.0:
        mode_lead = 2.5 <= t < 10.6 or t >= 16.5
        v = 0.08 if mode_lead else 0.0
        add(t, "/cmd_vel", v, 0.0)
        add(t, "/arc_hw/cmd_wheels", v, v)
        meas = 0.0
        if 2.9 <= t < 11.2 or t >= 16.9:
            meas = 0.08
        add(t, "/arc_hw/wheels", meas, meas, 70)
        t += 0.05
    add(1.0, "/arc/ui/command", text="enrol")
    add(1.9, "/arc/ui/command", text="lead_me")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(HL.FIELDS)
        w.writerows(sorted(rows, key=lambda r: float(r[0])))


class TimelineTests(unittest.TestCase):
    def test_reactions_and_plot(self):
        with tempfile.TemporaryDirectory() as d:
            log = Path(d) / "chain_test.csv"
            synthetic_chain(log)
            rows = PT.load(log)
            ev = PT.go_changes(rows)
            self.assertEqual([k for _, k in ev], ["go", "stop", "go"])
            r = PT.reactions(rows)
            stop = r[1]
            self.assertEqual(stop["mode_before"], "LEAD")
            self.assertAlmostEqual(stop["bt_s"], 0.4, places=2)
            self.assertAlmostEqual(stop["cmd_s"], 0.6, delta=0.06)
            self.assertAlmostEqual(stop["wheel_s"], 1.2, delta=0.06)
            go = r[2]
            self.assertAlmostEqual(go["bt_s"], 0.3, places=2)
            self.assertAlmostEqual(go["wheel_s"], 0.9, delta=0.06)
            self.assertEqual(PT.main([str(log)]), 0)
            self.assertTrue((Path(d) / "chain_test_timeline.png").stat().st_size > 20000)
            self.assertIn("stop", (Path(d) / "chain_test_reactions.csv").read_text())
            self.assertTrue(any("stop events: 1" in s for s in PT.summary(r)))

    def test_go_without_motion_is_dash(self):
        with tempfile.TemporaryDirectory() as d:
            log = Path(d) / "c.csv"
            with open(log, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(HL.FIELDS)
                for i in range(40):
                    t = i * 0.25
                    w.writerow([t, "/person_moving", 1 if t >= 5 else 0, "", "", ""])
                    w.writerow([t, "/person_confused", 0, "", "", ""])
                    w.writerow([t, "/cmd_vel", 0.0, 0.0, "", ""])           # no goal: stays still
            r = PT.reactions(PT.load(log))
            self.assertEqual(len(r), 1)
            self.assertIsNone(r[0]["cmd_s"])
            self.assertEqual(PT.fmt(None), "-")


class FakeCap:
    def __init__(self, src):
        self.n = 0

    def set(self, *a):
        return True

    def read(self):
        self.n += 1
        return (self.n <= 200), np.full((480, 640, 3), 128, np.uint8)

    def release(self):
        pass


class CameraPubTests(unittest.TestCase):
    def test_rate_limited_jpeg(self):
        sent = []
        clock = {"t": 0.0}

        def mono():                         # camera delivers frames every 0.02 s (50 fps)
            clock["t"] += 0.01
            return clock["t"]

        class Pub:
            def publish(self, m):
                sent.append(m)

        class Node:
            def __init__(self, name):
                pass

            def create_publisher(self, *a):
                return Pub()

            def get_logger(self):
                return types.SimpleNamespace(info=lambda *a: None, error=lambda *a: None)

            def get_clock(self):
                return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 0))

            def destroy_node(self):
                pass

        class Img:
            def __init__(self):
                self.header = types.SimpleNamespace(stamp=None, frame_id="")
                self.format, self.data = "", b""
        rclpy = types.SimpleNamespace(init=lambda **k: None, ok=lambda: True, shutdown=lambda: None)
        mods = {"rclpy": rclpy, "rclpy.node": types.SimpleNamespace(Node=Node),
                "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=None),
                "sensor_msgs": types.SimpleNamespace(), "sensor_msgs.msg": types.SimpleNamespace(CompressedImage=Img)}
        with mock.patch.dict(sys.modules, mods), mock.patch.object(CP.cv2, "VideoCapture", FakeCap), \
                mock.patch.object(CP.time, "monotonic", mono), mock.patch.object(CP.signal, "signal"):
            self.assertEqual(CP.main(["--fps", "10"]), 0)
        # 199 frames x 0.01 s of fake time (one clock read per frame) = ~2 s at 10 fps -> ~20, never 199
        self.assertGreater(len(sent), 15)
        self.assertLess(len(sent), 25)
        self.assertEqual(sent[0].format, "jpeg")
        self.assertEqual(sent[0].header.frame_id, "arc_camera")
        self.assertTrue(sent[0].data[:2] == b"\xff\xd8")             # JPEG magic


class PerfTests(unittest.TestCase):
    def test_stats(self):
        s = PP.summarise([100.0, 110.0, 120.0, 400.0])
        self.assertEqual(s["median_ms"], 115.0)
        self.assertEqual(s["max_ms"], 400.0)
        self.assertAlmostEqual(s["fps_median"], 8.7, places=1)
        self.assertAlmostEqual(PP.pct([1, 2, 3, 4, 5], 0.5), 3)


def fake_bin(d, ip_out="", ping_ok=False, hosts=None):
    """PATH dir with fake ip / ping / ros2 / nmcli / getent so the scripts run here."""
    b = Path(d) / "bin"
    b.mkdir()
    (b / "ip").write_text(f"#!/bin/sh\ncat <<'EOF'\n{ip_out}\nEOF\n")
    (b / "ping").write_text(f"#!/bin/sh\nexit {0 if ping_ok else 1}\n")
    cases = "".join(f'  {h}) echo "{a}   STREAM {h}";;\n' for h, a in (hosts or {}).items())
    (b / "getent").write_text(f'#!/bin/sh\ncase "$2" in\n{cases}  *) exit 2;;\nesac\n')
    (b / "ros2").write_text("#!/bin/sh\nexit 0\n")
    (b / "nmcli").write_text("#!/bin/sh\necho disabled\n")
    for f in b.iterdir():
        f.chmod(0o755)
    return str(b)


LO = "1: lo: <LOOPBACK,MULTICAST,UP,LOWER_UP> mtu 65536\n    inet 127.0.0.1/8 scope host lo"
WIFI = "3: wlp2s0    inet 192.168.43.20/24 brd 192.168.43.255 scope global dynamic wlp2s0"


def run(cmd, home, path_dir, stdin="", extra_env=None):
    env = {"HOME": home, "PATH": f"{path_dir}:/usr/bin:/bin", "USER": "t", "ARC_PI_YES": "1"}
    env.update(extra_env or {})
    return subprocess.run(["bash", "-c", cmd], env=env, input=stdin, capture_output=True, text=True, timeout=60)


class ShellTests(unittest.TestCase):
    """Cable link (LINK=cable): static addresses."""
    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.home = self.d.name
        conf = Path(self.home) / "cable.conf"
        conf.write_text((KIT / "hil.conf").read_text().replace("LINK=hotspot", "LINK=cable"))
        self.env = {"ARC_PI_CONF": str(conf)}

    def tearDown(self):
        self.d.cleanup()

    def test_profile_is_valid_xml_with_only_two_interfaces(self):
        b = fake_bin(self.home)
        r = run(f"source {KIT}/lib.sh && make_profile 192.168.50.1 && cat $HOME/.arc_hil/fastdds_hil.xml", self.home, b, extra_env=self.env)
        self.assertEqual(r.returncode, 0, r.stderr)
        dom = xml.dom.minidom.parseString(r.stdout)
        addrs = [a.firstChild.data for a in dom.getElementsByTagName("address")]
        self.assertEqual(addrs, ["127.0.0.1", "192.168.50.1"])
        self.assertEqual(dom.getElementsByTagName("useBuiltinTransports")[0].firstChild.data, "false")
        r = run(f"source {KIT}/lib.sh && make_profile 999 ; echo rc=$?", self.home, b, extra_env=self.env)
        self.assertIn("rc=1", r.stdout)                                  # returns, does not exit the shell

    def test_env_sh_modes(self):
        b = fake_bin(self.home)
        show = f"source {TEST_KIT}/env.sh 2>/dev/null; echo $ARC_NET_MODE $ROS_AUTOMATIC_DISCOVERY_RANGE $FASTRTPS_DEFAULT_PROFILES_FILE"
        r = run(show, self.home, b, extra_env=self.env)
        self.assertEqual(r.stdout.split()[:2], ["local", "LOCALHOST"])
        self.assertTrue(r.stdout.strip().endswith("fastdds_localhost.xml"))
        r = run(f"{KIT}/laptop/hil_mode.sh on", self.home, b, extra_env=self.env)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        r = run(show, self.home, b, extra_env=self.env)
        self.assertEqual(r.stdout.split(), ["hil", "SUBNET", f"{self.home}/.arc_hil/fastdds_hil.xml"])
        os.remove(f"{self.home}/.arc_hil/fastdds_hil.xml")             # flag without profile -> safe default
        self.assertEqual(run(show, self.home, b, extra_env=self.env).stdout.split()[0], "local")
        run(f"{KIT}/laptop/hil_mode.sh on && {KIT}/laptop/hil_mode.sh off", self.home, b, extra_env=self.env)
        self.assertEqual(run(show, self.home, b, extra_env=self.env).stdout.split()[0], "local")

    def test_bringup_stops_in_hil_mode_without_link(self):
        b = fake_bin(self.home, ip_out=LO)
        run(f"{KIT}/laptop/hil_mode.sh on", self.home, b, extra_env=self.env)
        r = run(f"{TEST_KIT}/bringup.sh sim", self.home, b, extra_env=self.env)
        self.assertEqual(r.returncode, 1)
        self.assertIn("no address 192.168.50.1", r.stdout)
        self.assertNotIn("started", r.stdout)                            # nothing launched

    def test_bringup_hil_passes_check_with_link(self):
        b = fake_bin(self.home, ip_out=LO + "\n    inet 192.168.50.1/24 brd 192.168.50.255 scope global eth0")
        run(f"{KIT}/laptop/hil_mode.sh on", self.home, b, extra_env=self.env)
        # 'status' skips the check; use a bogus stage so it stops right after the network checks
        r = run(f"{TEST_KIT}/bringup.sh bogus-stage", self.home, b, extra_env=self.env)
        self.assertIn("network: HIL", r.stdout)

    def test_pi_up_refuses_without_serial_and_status_works(self):
        b = fake_bin(self.home, ip_out=LO)
        r = run(f"{KIT}/pi/pi_up.sh motors", self.home, b, extra_env=self.env)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("/dev/ttyACM0 not found", r.stdout)
        self.assertIn("(192.168.50.2): the laptop will not see", r.stdout)                     # link warning shown first
        r = run(f"{KIT}/pi/pi_up.sh status", self.home, b, extra_env=self.env)
        self.assertEqual(r.returncode, 0)
        self.assertIn("motors: stopped", r.stdout)
        self.assertEqual(run(f"{KIT}/pi/pi_up.sh down", self.home, b, extra_env=self.env).returncode, 0)

    def test_pi_up_start_and_clean_stop(self):
        """start()/stop() with a stand-in node: SIGTERM must reach Python and run its cleanup."""
        b = fake_bin(self.home, ip_out=LO)
        node = Path(self.home) / "fake_node.py"
        node.write_text(textwrap.dedent(f"""
            import signal, sys, time
            def h(*_): raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, h)
            try:
                while True: time.sleep(0.1)
            except KeyboardInterrupt:
                open('{self.home}/cleanup_ran', 'w').write('yes')
        """))
        script = (f"source {KIT}/pi/pi_env.sh >/dev/null 2>&1; "
                  f"eval \"$(sed -n '/^running()/,/^prechecks()/p' {KIT}/pi/pi_up.sh | sed '$d')\"; "
                  f"PIDS=$ARC_RUN/pids; LOGS=$ARC_RUN/logs; mkdir -p $PIDS $LOGS; "
                  f"start fake python3 {node} && running fake && echo RUNNING; stop fake; running fake || echo STOPPED")
        r = run(script, self.home, b, extra_env=self.env)
        self.assertIn("RUNNING", r.stdout, r.stdout + r.stderr)
        self.assertIn("STOPPED", r.stdout)
        self.assertEqual((Path(self.home) / "cleanup_ran").read_text(), "yes")

    def test_prep_sd_adds_static_eth0_keeps_wifi(self):
        b = fake_bin(self.home)
        boot = Path(self.home) / "system-boot"
        boot.mkdir()
        (boot / "network-config").write_text(textwrap.dedent("""\
            network:
              version: 2
              ethernets:
                eth0:
                  dhcp4: true
                  optional: true
              wifis:
                wlan0:
                  dhcp4: true
                  access-points:
                    "HomeNet":
                      password: "secret"
            """))
        r = run(f"{KIT}/laptop/prep_sd.sh {boot}", self.home, b, extra_env=self.env)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        import yaml
        d = yaml.safe_load((boot / "network-config").read_text())
        self.assertEqual(d["network"]["ethernets"]["eth0"]["addresses"], ["192.168.50.2/24"])
        self.assertTrue(d["network"]["ethernets"]["eth0"]["dhcp4"])
        self.assertEqual(d["network"]["wifis"]["wlan0"]["access-points"]["HomeNet"]["password"], "secret")
        self.assertTrue((boot / "network-config.bak_arc").exists())
        r = run(f"{KIT}/laptop/prep_sd.sh {boot}", self.home, b, extra_env=self.env)              # second run: no change
        self.assertIn("already has", r.stdout)

    def test_scripts_parse(self):
        for f in list((KIT / "laptop").glob("*.sh")) + list((KIT / "pi").glob("*.sh")) + [KIT / "lib.sh"]:
            r = subprocess.run(["bash", "-n", str(f)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, f"{f}: {r.stderr}")


class HotspotTests(unittest.TestCase):
    """LINK=hotspot (the default hil.conf): addresses from Wi-Fi, peers found by name."""
    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.home = self.d.name

    def tearDown(self):
        self.d.cleanup()

    def profile(self):
        dom = xml.dom.minidom.parse(f"{self.home}/.arc_hil/fastdds_hil.xml")
        wl = dom.getElementsByTagName("interfaceWhiteList")[0]
        allowed = [a.firstChild.data for a in wl.getElementsByTagName("address")]
        peers = dom.getElementsByTagName("initialPeersList")
        peer_addrs = [a.firstChild.data for a in peers[0].getElementsByTagName("address")] if peers else []
        return dom, allowed, peer_addrs

    def test_default_conf_is_hotspot(self):
        r = run(f"source {KIT}/lib.sh && echo $LINK $PI_HOST", self.home, fake_bin(self.home))
        self.assertEqual(r.stdout.split(), ["hotspot", "arc-pi.local"])

    def test_hil_mode_on_uses_wifi_ip_and_pi_by_name(self):
        b = fake_bin(self.home, ip_out=LO + "\n" + WIFI, hosts={"arc-pi.local": "192.168.43.77"})
        r = run(f"{KIT}/laptop/hil_mode.sh on", self.home, b)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("Pi at 192.168.43.77", r.stdout)
        dom, allowed, peers = self.profile()
        self.assertEqual(allowed, ["127.0.0.1", "192.168.43.20"])            # Wi-Fi IP only, nothing else
        self.assertEqual(peers, ["239.255.0.1", "127.0.0.1", "192.168.43.77"])  # multicast kept + unicast
        port = dom.getElementsByTagName("initialPeersList")[0].getElementsByTagName("port")[0].firstChild.data
        self.assertEqual(port, "7400")
        self.assertEqual(dom.getElementsByTagName("maxInitialPeersRange")[0].firstChild.data, "64")
        r = run(f"{KIT}/laptop/hil_mode.sh status", self.home, b)
        self.assertIn("laptop still has 192.168.43.20", r.stdout)

    def test_hil_mode_on_without_pi_is_multicast_only(self):
        b = fake_bin(self.home, ip_out=LO + "\n" + WIFI)
        r = run(f"{KIT}/laptop/hil_mode.sh on", self.home, b)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("not found now", r.stdout)
        _, allowed, peers = self.profile()
        self.assertEqual((allowed, peers), (["127.0.0.1", "192.168.43.20"], []))

    def test_hil_mode_on_refuses_without_wifi(self):
        r = run(f"{KIT}/laptop/hil_mode.sh on", self.home, fake_bin(self.home, ip_out=LO))
        self.assertEqual(r.returncode, 1)
        self.assertIn("Join the phone hotspot", r.stdout)
        self.assertFalse(os.path.exists(f"{self.home}/.arc_net"))

    def test_bringup_stops_when_hotspot_address_changed(self):
        b = fake_bin(self.home, ip_out=LO + "\n" + WIFI)
        run(f"{KIT}/laptop/hil_mode.sh on", self.home, b)
        b2 = Path(self.home) / "bin2"; b2.mkdir()
        (b2 / "ip").write_text("#!/bin/sh\ncat <<'EOF'\n" + LO + "\n" + WIFI.replace(".20/", ".31/") + "\nEOF\n")
        (b2 / "ip").chmod(0o755)
        r = run(f"{TEST_KIT}/bringup.sh sim", self.home, f"{b2}:{b}")
        self.assertEqual(r.returncode, 1)
        self.assertIn("no address 192.168.43.20", r.stdout)
        self.assertIn("hil_mode.sh on", r.stdout)
        self.assertNotIn("started", r.stdout)

    def test_pi_env_uses_laptop_from_hil_peer(self):
        kit = Path(self.home) / "arc_pi"
        shutil.copytree(KIT, kit, ignore=shutil.ignore_patterns("updates", "tests", "__pycache__"))
        (kit / "hil.peer").write_text("LAPTOP_HOST=akshada-laptop.local\nLAPTOP_IP_SEEN=192.168.43.20\n")
        pi_wifi = WIFI.replace(".20/", ".77/").replace("wlp2s0", "wlan0")
        b = fake_bin(self.home, ip_out=LO + "\n" + pi_wifi, hosts={"akshada-laptop.local": "192.168.43.21"})
        r = run(f"source {kit}/pi/pi_env.sh && echo MODE=$ARC_NET_MODE", self.home, b)
        self.assertIn("MODE=hil", r.stdout, r.stdout + r.stderr)
        _, allowed, peers = self.profile()
        self.assertEqual(allowed, ["127.0.0.1", "192.168.43.77"])
        self.assertEqual(peers[-1], "192.168.43.21")                          # mDNS name wins
        b3 = Path(self.home) / "b3"; b3.mkdir()
        (b3 / "getent").write_text("#!/bin/sh\nexit 2\n"); (b3 / "getent").chmod(0o755)
        run(f"source {kit}/pi/pi_env.sh", self.home, f"{b3}:{b}")
        self.assertEqual(self.profile()[2][-1], "192.168.43.20")             # falls back to the seen address

    def test_net_laptop_reports_client_isolation(self):
        b = fake_bin(self.home, ip_out=LO + "\n" + WIFI, hosts={"arc-pi.local": "192.168.43.77"}, ping_ok=False)
        r = run(f"{KIT}/laptop/net_laptop.sh up", self.home, b)
        self.assertEqual(r.returncode, 1)
        self.assertIn("client isolation", r.stdout)
        b2 = Path(self.home) / "ok"; b2.mkdir()
        (b2 / "ping").write_text("#!/bin/sh\nexit 0\n"); (b2 / "ping").chmod(0o755)
        r = run(f"{KIT}/laptop/net_laptop.sh up", self.home, f"{b2}:{b}")
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("Pi answers at 192.168.43.77", r.stdout)

    def test_prep_sd_hotspot_changes_nothing(self):
        r = run(f"{KIT}/laptop/prep_sd.sh /nonexistent", self.home, fake_bin(self.home))
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn("phone", r.stdout)


if __name__ == "__main__":
    unittest.main()
