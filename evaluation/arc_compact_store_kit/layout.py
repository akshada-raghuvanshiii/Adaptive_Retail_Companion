"""ARC compact store: the single source of truth for geometry and destinations.

Coordinate convention (see RUNBOOK.md, "Coordinate frames")
-----------------------------------------------------------
* Gazebo world origin = robot spawn pose (0, 0, yaw 0).
* Gazebo diff-drive odometry starts at zero at spawn, so odom == world at t = 0.
* RTAB-Map starts its map frame on the odom frame (map -> odom = identity at start),
  so map == world, up to RTAB-Map's drift corrections.
* Every coordinate in this file is therefore valid in both `world` and `map`.
  This is a CONVENTION THAT MUST BE VERIFIED on your machine (RUNBOOK step 5),
  not a static transform that is assumed.

Units: metres, radians. Fixtures are solid boxes from the floor up, so a 2D
LiDAR at ~0.2 m height sees their full footprint (tables on thin legs would not be mapped).
"""

from dataclasses import dataclass, field
from math import pi
from typing import List, Tuple

# Requested store limits (outer faces of the walls)
STORE_X = (-3.5, 4.5)
STORE_Y = (-3.0, 4.0)
WALL_T = 0.10
WALL_H = 1.0

# Map published at the time of the redesign (for the fit check only)
CURRENT_MAP_X = (-4.2251, -4.2251 + 203 * 0.05)
CURRENT_MAP_Y = (-4.0208, -4.0208 + 179 * 0.05)

SPAWN = (0.0, 0.0, 0.0)  # x, y, yaw in world == map

# Robot used for clearance checks (TurtleBot3 Waffle-sized; Burger is smaller)
ROBOT_RADIUS = 0.22
SAFETY_MARGIN = 0.10


@dataclass(frozen=True)
class Fixture:
    name: str
    kind: str                 # gondola | wall_shelf | produce | fridge | checkout | wall
    x0: float
    y0: float
    x1: float
    y1: float
    height: float
    colour: Tuple[float, float, float] = (0.7, 0.7, 0.7)

    @property
    def centre(self):
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)

    @property
    def size(self):
        return (self.x1 - self.x0, self.y1 - self.y0)


@dataclass(frozen=True)
class Destination:
    id: str
    category: str             # produce | dairy | tea_coffee | checkout | gondola
    fixture: str
    x: float
    y: float
    yaw: float                # robot heading when stopped: facing the fixture
    label: str
    keywords: Tuple[str, ...] = field(default_factory=tuple)


X0, X1 = STORE_X
Y0, Y1 = STORE_Y
IX0, IX1, IY0, IY1 = X0 + WALL_T, X1 - WALL_T, Y0 + WALL_T, Y1 - WALL_T   # interior

WALLS: List[Fixture] = [
    Fixture("wall_south", "wall", X0, Y0, X1, Y0 + WALL_T, WALL_H, (0.85, 0.85, 0.8)),
    Fixture("wall_north", "wall", X0, Y1 - WALL_T, X1, Y1, WALL_H, (0.85, 0.85, 0.8)),
    Fixture("wall_west", "wall", X0, Y0 + WALL_T, X0 + WALL_T, Y1 - WALL_T, WALL_H, (0.85, 0.85, 0.8)),
    Fixture("wall_east", "wall", X1 - WALL_T, Y0 + WALL_T, X1, Y1 - WALL_T, WALL_H, (0.85, 0.85, 0.8)),
]

GONDOLA_Y = (-1.0, 2.0)   # 3.0 m long, cross aisles at both ends

FIXTURES: List[Fixture] = [
    # Three gondolas, running along +y, with aisles between them
    Fixture("gondola_1", "gondola", 0.70, GONDOLA_Y[0], 1.20, GONDOLA_Y[1], 1.40, (0.55, 0.62, 0.72)),
    Fixture("gondola_2", "gondola", 2.40, GONDOLA_Y[0], 2.90, GONDOLA_Y[1], 1.40, (0.55, 0.62, 0.72)),
    Fixture("gondola_3", "wall_shelf", 4.05, GONDOLA_Y[0], IX1, GONDOLA_Y[1], 1.40, (0.55, 0.62, 0.72)),
    # Fresh produce island (solid display base)
    Fixture("produce_display", "produce", -1.90, 0.60, -0.90, 1.60, 0.80, (0.45, 0.70, 0.35)),
    # Refrigerated milk cabinet against the north wall
    Fixture("dairy_fridge", "fridge", -3.00, 3.30, -1.40, IY1, 1.80, (0.80, 0.88, 0.95)),
    # Tea & coffee shelf against the west wall
    Fixture("tea_coffee_shelf", "wall_shelf", IX0, -0.60, -3.00, 1.40, 1.60, (0.62, 0.45, 0.30)),
    # Checkout counter
    Fixture("checkout_counter", "checkout", -2.40, -2.30, -1.20, -1.80, 0.90, (0.30, 0.30, 0.35)),
]

ALL_OBSTACLES = WALLS + FIXTURES

STANDOFF = 0.55   # robot centre to fixture face

_g = {f.name: f for f in FIXTURES}
_ys = (-0.40, 1.40)   # two stopping points along each gondola face
_aisle_12 = (_g["gondola_1"].x1 + _g["gondola_2"].x0) / 2
_aisle_23 = (_g["gondola_2"].x1 + _g["gondola_3"].x0) / 2

DESTINATIONS: List[Destination] = [
    Destination("produce", "produce", "produce_display",
                (_g["produce_display"].x0 + _g["produce_display"].x1) / 2, _g["produce_display"].y0 - STANDOFF,
                pi / 2, "Fruit and vegetables",
                ("fruit", "vegetable", "veg", "produce", "apple", "banana", "orange", "tomato", "potato",
                 "onion", "mango", "grape", "carrot", "fresh")),
    Destination("dairy", "dairy", "dairy_fridge",
                (_g["dairy_fridge"].x0 + _g["dairy_fridge"].x1) / 2, _g["dairy_fridge"].y0 - STANDOFF,
                pi / 2, "Milk and refrigerated products",
                ("milk", "dairy", "curd", "yogurt", "yoghurt", "paneer", "cheese", "butter", "fridge",
                 "refrigerat", "chilled", "cold", "egg", "frozen", "ice_cream", "icecream")),
    Destination("tea_coffee", "tea_coffee", "tea_coffee_shelf",
                _g["tea_coffee_shelf"].x1 + STANDOFF,
                (_g["tea_coffee_shelf"].y0 + _g["tea_coffee_shelf"].y1) / 2,
                pi, "Tea, coffee and drinks", ("tea", "coffee", "beverage", "drink", "juice", "water")),
    Destination("checkout", "checkout", "checkout_counter",
                (_g["checkout_counter"].x0 + _g["checkout_counter"].x1) / 2, _g["checkout_counter"].y1 + STANDOFF,
                -pi / 2, "Checkout / waiting point", ("checkout", "billing", "cashier", "counter", "wait", "exit")),
]

for gname, west_x, east_x in (("gondola_1", _g["gondola_1"].x0 - STANDOFF, _aisle_12),
                              ("gondola_2", _aisle_12, _aisle_23)):
    for i, y in enumerate(_ys):
        tag = "ab"[i]
        DESTINATIONS.append(Destination(f"{gname}_west_{tag}", "gondola", gname, west_x, y, 0.0,
                                        f"{gname.replace('_', ' ').title()} west face, section {tag.upper()}"))
        DESTINATIONS.append(Destination(f"{gname}_east_{tag}", "gondola", gname, east_x, y, pi,
                                        f"{gname.replace('_', ' ').title()} east face, section {tag.upper()}"))
for i, y in enumerate(_ys):
    tag = "ab"[i]
    DESTINATIONS.append(Destination(f"gondola_3_{tag}", "gondola", "gondola_3", _aisle_23, y, 0.0,
                                    f"Gondola 3 (wall), section {tag.upper()}"))

GONDOLA_DESTINATIONS = [d for d in DESTINATIONS if d.category == "gondola"]

# Models kept from (or added to) the world, with safe placements in the compact store.
# Dynamic models with collision (shopper, person) are seen by the LiDAR, so they are
# parked in corners no route or destination needs; drive/move them out for person tests.
# Pose: x, y, z, roll, pitch, yaw.  radius: footprint radius used by verify_layout.py.
PLACEMENTS = {
    "shopper": {"pose": (-3.00, -2.45, 0.0, 0.0, 0.0, 0.0), "radius": 0.25,
                "note": "drivable test shopper (/shopper/cmd_vel), parked SW corner"},
    "person": {"pose": (4.15, 3.60, 0.0, 0.0, 0.0, 0.0), "radius": 0.15,
               "note": "BT visual person (moved by gazebo_person_pose), parked NE corner"},
    "overhead_cam": {"pose": (1.80, 0.50, 3.00, 0.0, 1.5708, 0.0), "radius": 0.0,
                     "note": "ceiling camera looking down on the gondola 1-2 aisle"},
}


def parked_obstacles():
    """Parked dynamic models as square obstacles (for clearance/connectivity checks)."""
    out = []
    for name, p in PLACEMENTS.items():
        r = p["radius"]
        if r > 0:
            x, y = p["pose"][:2]
            out.append(Fixture(f"parked_{name}", "parked", x - r, y - r, x + r, y + r, 1.0))
    return out
