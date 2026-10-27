"""Drone mission loading and route clearance checks (shared by the launch file, mission node and tools).

A route is the flown polyline in world ENU coordinates:
takeoff (ground) -> climb to cruise altitude -> waypoints in order (optional via points between them)
-> back above the takeoff point at cruise altitude -> land.
"""
import json
import math
import os

import yaml

FALLBACK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "missions")


def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001 - running from source without an install
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(os.path.dirname(here), pkg)


def world_yaml_path(world):
    return os.path.join(_share("fleet_worlds"), "worlds", world + ".yaml")


def drone_share():
    return _share("fleet_drone")


def load_mission(world, mission_file=""):
    """Return {cruise_altitude, takeoff:{x,y,yaw}, waypoints:[{name,x,y,z,via?}], source}.

    Priority: explicit mission_file > the world yaml's `drone:` section > config/missions/<world>.yaml.
    """
    src = None
    data = None
    if mission_file:
        with open(mission_file) as fh:
            data = yaml.safe_load(fh)
        data = data.get("drone", data)
        src = mission_file
    else:
        wy = world_yaml_path(world)
        if os.path.exists(wy):
            with open(wy) as fh:
                d = yaml.safe_load(fh) or {}
            if d.get("drone"):
                data, src = d["drone"], wy
        if data is None:
            fb = os.path.join(drone_share(), "config", "missions", world + ".yaml")
            if os.path.exists(fb):
                with open(fb) as fh:
                    data = yaml.safe_load(fh)
                data, src = data.get("drone", data), fb
    if data is None:
        raise RuntimeError("no drone mission for world %r (world yaml has no drone section and no fallback)" % world)
    t = data["takeoff"]
    out = {
        "cruise_altitude": float(data["cruise_altitude"]),
        "takeoff": {"x": float(t["x"]), "y": float(t["y"]), "yaw": float(t.get("yaw", 0.0))},
        "waypoints": [],
        "source": src,
    }
    for w in data["waypoints"]:
        out["waypoints"].append({"name": str(w["name"]), "x": float(w["x"]), "y": float(w["y"]), "z": float(w["z"]),
                                 "via": [[float(v) for v in p] for p in w.get("via", [])]})
    out["return_via"] = [[float(v) for v in p] for p in data.get("return_via", [])]
    # package route overrides: via points between the world's waypoints (keyed by waypoint name, or "return")
    rf = os.path.join(drone_share(), "config", "routes", world + ".yaml")
    if not mission_file and os.path.exists(rf):
        with open(rf) as fh:
            via = (yaml.safe_load(fh) or {}).get("via", {})
        names = {w["name"] for w in out["waypoints"]}
        for w in out["waypoints"]:
            if not w["via"] and w["name"] in via:
                w["via"] = [[float(v) for v in p] for p in via[w["name"]]]
        if not out["return_via"] and "return" in via:
            out["return_via"] = [[float(v) for v in p] for p in via["return"]]
        out["route_file"] = rf
        unknown = set(via) - names - {"return"}
        if unknown:
            out["route_warning"] = "via points for unknown waypoints: %s" % sorted(unknown)
    return out


def route_points(m, ground_z=0.0):
    """List of (name, x, y, z, is_waypoint) in flight order (world ENU, absolute z)."""
    t = m["takeoff"]
    cz = ground_z + m["cruise_altitude"] if m.get("cruise_is_agl") else m["cruise_altitude"]
    pts = [("ground", t["x"], t["y"], ground_z + 0.3, False), ("climb", t["x"], t["y"], cz, False)]
    for w in m["waypoints"]:
        for i, v in enumerate(w.get("via", [])):
            pts.append(("%s_via%d" % (w["name"], i), v[0], v[1], v[2], False))
        pts.append((w["name"], w["x"], w["y"], w["z"], True))
    for i, v in enumerate(m.get("return_via", [])):
        pts.append(("return_via%d" % i, v[0], v[1], v[2], False))
    pts.append(("return", t["x"], t["y"], cz, False))
    pts.append(("land", t["x"], t["y"], ground_z + 0.3, False))
    return pts


def load_primitives(world):
    p = os.path.join(_share("fleet_worlds"), "models", world + "_env", "collision.json")
    if not os.path.exists(p):
        return []
    with open(p) as fh:
        return json.load(fh)["primitives"]


def ground_height(prims, x, y, zmax=1.5):
    """Highest primitive top under (x, y) that is below zmax (flat-ish floors); 0 if none."""
    best = None
    for p in prims:
        cx, cy, cz, yaw = p["pose"]
        if p["type"] == "box":
            sx, sy, sz = p["size"]
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx, ly = c * (x - cx) - s * (y - cy), s * (x - cx) + c * (y - cy)
            if abs(lx) > sx / 2 or abs(ly) > sy / 2:
                continue
            top = cz + sz / 2
        else:
            if math.hypot(x - cx, y - cy) > p["radius"]:
                continue
            top = cz + p["length"] / 2
        if top <= zmax and (best is None or top > best):
            best = top
    return 0.0 if best is None else best
