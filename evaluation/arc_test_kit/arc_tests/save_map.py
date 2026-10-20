#!/usr/bin/env python3
"""Save the live /rtabmap/map to results/map.npz for the offline benchmark."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402

from arc_tests.common import Grid  # noqa: E402
from arc_tests.rosio import Watcher  # noqa: E402


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "results"
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    w = Watcher()
    try:
        if not w.wait_for(lambda: w.map_msg is not None, 15):
            sys.exit("no map on /rtabmap/map")
        i = w.map_msg.info
        g = Grid(list(w.map_msg.data), i.width, i.height, i.resolution,
                 i.origin.position.x, i.origin.position.y)
        g.save_npz(os.path.join(out, "map.npz"))
        print(f"saved {out}/map.npz ({i.width}x{i.height} @ {i.resolution} m)")
    finally:
        w.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
