#!/usr/bin/env python3
"""Drive a short revisit of the start area so RTAB-Map can close the loop (mapping mode only).

Why: on 15 Oct two maps ended 0.23-0.25 m off at home although the middle of the
route had been corrected by loop closures (auto_map log: 0.34 m at aisle 1 north,
0.01 m from the checkout on, 0.23 m back at home). RTAB-Map only adds nodes when
the robot moves (RGBD/LinearUpdate 0.1 m), so a robot parked at home never gets
matched against the first nodes of the map. Driving to the start area and back
gives it new nodes next to the oldest ones, where a proximity loop closure can
pull the end of the map back.

Reuses auto_map.py's route follower unchanged (steering by the map pose),
with a 2-point route. Run before set_mode_localization.
"""
import importlib.util
import os
import sys

AUTO_MAP = os.path.expanduser("~/arc_compact_store_kit/ros_tools/auto_map.py")


def main():
    spec = importlib.util.spec_from_file_location("auto_map", AUTO_MAP)
    am = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(am)
    am.ROUTE[:] = [(0.00, -1.50, "revisit: south of start"),
                   (-0.80, -0.80, "revisit: back via the west side"),
                   (0.00, 0.00, "home (spawn point)")]
    log = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/arc_close_loop_log.csv")
    # --force: the start check would refuse because the pose is a little off - that is why we are here
    return am.main(["--force", "--no-guard", "--log", log])


if __name__ == "__main__":
    sys.exit(main())
