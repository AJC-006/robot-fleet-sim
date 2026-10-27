"""Component catalog access (fleet_components): load a component's YAML by category and file stem."""
import os

import yaml


def _roots():
    here = os.path.dirname(os.path.abspath(__file__))
    roots = []
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(os.path.join(get_package_share_directory("fleet_components"), "components"))
    except Exception:
        pass
    # source tree fallback (src/fleet_auv/fleet_auv -> src/fleet_components/components)
    roots.append(os.path.abspath(os.path.join(here, "..", "..", "fleet_components", "components")))
    return roots


def path(category, stem, filename=None):
    """Absolute path of components/<category>/<stem>.yaml (or another file in that category directory)."""
    for r in _roots():
        p = os.path.join(r, category, filename or (stem + ".yaml"))
        if os.path.exists(p):
            return p
    raise FileNotFoundError("catalog component %s/%s not found in %s" % (category, filename or stem, _roots()))


def load(category, stem):
    with open(path(category, stem)) as f:
        return yaml.safe_load(f)
