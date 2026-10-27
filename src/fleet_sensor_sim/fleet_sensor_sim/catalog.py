"""Load a sensor catalog entry (fleet_components/components/<category>/<model>.yaml)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def load(category, model, catalog_dir=''):
    """Return the parsed catalog YAML of `model`; catalog_dir overrides the installed catalog."""
    root = Path(catalog_dir) if catalog_dir else (
        Path(get_package_share_directory('fleet_components')) / 'components')
    with open(root / category / (model + '.yaml')) as f:
        return yaml.safe_load(f)
