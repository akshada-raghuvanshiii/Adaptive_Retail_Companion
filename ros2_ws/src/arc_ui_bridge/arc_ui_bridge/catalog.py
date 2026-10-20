"""The UI catalogue built from the Gazebo store's own items (item_locations.yaml). No ROS code.

With catalog:=store (the default) the web UI lists exactly the items the shopping manager accepts,
under the same names, grouped the way the RViz store labels group them:
  Aisle 1 / Aisle 2 / Aisle 3   - the aisles between the gondolas (x = 0.15, 1.80, 3.47 m)
  Fresh produce, Dairy fridge, Tea & beverages shelf - the stand-alone fixtures
Each entry keeps the store item name as its id, so a UI list is resolved exactly like a list typed
into the shopping manager.
"""
import math

AISLES = [("Aisle 1", 0.15), ("Aisle 2", 1.80), ("Aisle 3", 3.47)]
AISLE_TOL_M = 0.30
AREAS = [("Fresh produce", -1.40, 0.05), ("Dairy fridge", -2.20, 2.75), ("Tea & beverages shelf", -2.45, 0.40)]
AREA_TOL_M = 0.40
SECTION_SPLIT_Y = 0.5          # gondola sections: A below (y = -0.40), B above (y = 1.40)

PAIRS = {"tea_coffee": "Tea & coffee", "curd_paneer": "Curd & paneer", "dal_pulses": "Dal & pulses",
         "sugar_salt": "Sugar & salt", "jam_spreads": "Jam & spreads", "soap_shampoo": "Soap & shampoo",
         "water_beverages": "Water & beverages"}
ICONS = {"fruits": "🍎", "vegetables": "🥕", "milk": "🥛", "eggs": "🥚", "curd_paneer": "🧀", "bread": "🍞",
         "bakery": "🥐", "rice": "🍚", "atta_flour": "🌾", "tea_coffee": "☕", "water_beverages": "🥤",
         "juice": "🧃", "frozen": "🧊", "biscuits": "🍪", "snacks": "🍿", "breakfast_cereal": "🥣",
         "jam_spreads": "🍯", "pasta": "🍝", "sauces": "🥫", "spices": "🌶️", "dal_pulses": "🫘",
         "sugar_salt": "🧂", "cooking_oil": "🫒", "cleaning": "🧽", "detergent": "🧴", "soap_shampoo": "🧼",
         "toiletries": "🪥", "baby_care": "🍼", "pet_food": "🐾"}
SKIP = {"checkout", "Checkout"}


def pretty(item):
    """'atta_flour' -> 'Atta flour', 'tea_coffee' -> 'Tea & coffee'."""
    if item in PAIRS:
        return PAIRS[item]
    s = item.replace("_", " ").strip()
    return s[:1].upper() + s[1:]


def place(loc):
    """(area, section) of a store spot: an aisle and its section, or a fixture."""
    if loc.get("aisle"):                                    # the remapped file may already carry these
        return str(loc["aisle"]), str(loc.get("section", ""))
    x, y = float(loc["x"]), float(loc["y"])
    for name, ax, ay in AREAS:
        if math.hypot(x - ax, y - ay) <= AREA_TOL_M:
            return name, ""
    for name, ax in AISLES:
        if abs(x - ax) <= AISLE_TOL_M:
            return name, "Section A" if y < SECTION_SPLIT_Y else "Section B"
    return "Other", ""


def store_catalog(items):
    """List of UI product records, one per store item, in aisle order then name."""
    order = [a for a, _ in AISLES] + [a for a, _, _ in AREAS] + ["Other"]
    out = []
    for item, loc in items.items():
        if item in SKIP or "x" not in loc or "y" not in loc:
            continue
        area, section = place(loc)
        out.append({"id": item, "name": pretty(item), "category": area, "aisle": area,
                    "zone": section or area, "x": float(loc["x"]), "y": float(loc["y"]),
                    "yaw": float(loc.get("yaw", 0.0)), "icon": ICONS.get(item, "🛒"),
                    "unit": f"{section + ' · ' if section else ''}stop ({float(loc['x']):.2f}, {float(loc['y']):.2f})",
                    "sim_item": item, "sim_x": float(loc["x"]), "sim_y": float(loc["y"]), "source_url": ""})
    out.sort(key=lambda p: (order.index(p["category"]) if p["category"] in order else len(order), p["name"]))
    return out


def identity_map(items):
    """product_map for the store catalogue: every store item maps to itself."""
    return {k: k for k in items if k not in SKIP}
