#!/usr/bin/env python3
"""Runs the offline tests without pytest:  python3 test/run_tests.py"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_trip  # noqa: E402

fails = 0
for name in sorted(n for n in dir(test_trip) if n.startswith("test_")):
    try:
        getattr(test_trip, name)()
        print(f"  ok    {name}")
    except Exception as e:
        fails += 1
        print(f"  FAIL  {name}: {e!r}")
# the server test replaces rclpy with stand-ins, so it runs in its own process
r = subprocess.run([sys.executable, os.path.join(HERE, "test_server_stubbed.py")], capture_output=True, text=True)
print(f"  {'ok  ' if r.returncode == 0 else 'FAIL'}  test_server_api_flow" + ("" if r.returncode == 0 else "\n" + r.stderr[-800:]))
fails += r.returncode != 0
print(f"{9 - fails}/9 passed" if fails else "9/9 passed")
sys.exit(1 if fails else 0)
