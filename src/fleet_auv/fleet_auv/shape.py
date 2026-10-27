"""Hull outline shared by the Blender builder and the physics parameter estimate (pure python)."""
import math
import os


def load_cfg(path=None):
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        cands = [os.path.join(here, "..", "config", "knucker.yaml")]
        try:
            from ament_index_python.packages import get_package_share_directory
            cands.append(os.path.join(get_package_share_directory("fleet_auv"), "config", "knucker.yaml"))
        except Exception:
            pass
        for p in cands:
            if os.path.exists(p):
                path = p
                break
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
    except ImportError:   # Blender's python has no PyYAML
        import json
        import subprocess
        code = "import yaml,json,sys;print(json.dumps(yaml.safe_load(open(sys.argv[1]))))"
        return json.loads(subprocess.check_output(["/usr/bin/python3", "-c", code, str(path)]))


def hull_r(x, h):
    """Radius of the hull at station x (blunt ellipsoidal nose, cylinder, smooth taper to a rounded tail)."""
    L, R = h["length"], h["radius"]
    xn = L / 2
    x_nose0 = xn - h["nose_len"]
    x_tail0 = -xn + h["tail_len"]
    if x >= xn:
        return 0.0
    if x > x_nose0:
        t = (x - x_nose0) / h["nose_len"]
        return R * (1 - t ** 2.2) ** (1 / 2.2 * 1.0) if t < 1 else 0.0
    if x >= x_tail0:
        return R
    t = (x_tail0 - x) / h["tail_len"]
    t = min(1.0, max(0.0, t))
    sm = t * t * (3 - 2 * t)
    return R - (R - h["tail_radius"]) * sm


def collision_prims(cfg):
    """Hull slabs (cylinders along x following hull_r) + the extra primitives of the yaml."""
    h, n = cfg["hull"], int(cfg.get("slabs", 13))
    L = h["length"]
    dx = L / n
    prims = []
    for i in range(n):
        x0, x1 = -L / 2 + i * dx, -L / 2 + (i + 1) * dx
        # volume-true radius of the slab: mean of r^2 over the slab
        m = sum(hull_r(x0 + (x1 - x0) * (k + 0.5) / 20, h) ** 2 for k in range(20)) / 20
        prims.append({"name": "hull%02d" % i, "type": "cylinder", "r": round(math.sqrt(m), 5), "l": round(dx, 5),
                      "pos": [round((x0 + x1) / 2, 5), 0.0, 0.0]})
    return prims + list(cfg.get("collisions", []))
