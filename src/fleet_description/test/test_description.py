"""Robot description checks: every robot expands, mass budget is current, catalog names resolve."""
import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro
import yaml

SHARE = Path(get_package_share_directory('fleet_description'))
COMPONENTS = Path(get_package_share_directory('fleet_components')) / 'components'
ROBOTS = yaml.safe_load((SHARE / 'config' / 'robots.yaml').read_text())
MASS = yaml.safe_load((SHARE / 'config' / 'mass_properties.yaml').read_text())
CATEGORY = {'lidar': 'lidar', 'imu': 'imu', 'camera': 'depth_camera', 'wheel_motor': 'dc_motor',
            'motor_driver': 'motor_driver', 'battery_cell': 'battery', 'tyre': 'tyre'}


def expand(robot, camera):
    doc = xacro.process_file(str(SHARE / 'urdf' / 'robot.urdf.xacro'),
                             mappings={'robot': robot, 'camera': camera})
    return ET.fromstring(doc.toxml())


@pytest.mark.parametrize('camera', ['false', 'true'])
@pytest.mark.parametrize('robot', list(ROBOTS))
def test_expands(robot, camera):
    urdf = expand(robot, camera)
    links = {link.get('name') for link in urdf.findall('link')}
    for frame in ('base_footprint', 'base_link', 'laser_frame', 'imu_link', 'camera_link',
                  'camera_depth_optical_frame', 'camera_color_optical_frame'):
        assert frame in links
    sensors = {s.get('name') for s in urdf.iter('sensor')}
    assert {'lidar', 'imu'} <= sensors
    assert ({'color', 'depth'} <= sensors) == (camera == 'true')
    total = sum(float(link.find('inertial/mass').get('value'))
                for link in urdf.findall('link') if link.find('inertial/mass') is not None)
    assert total == pytest.approx(MASS[robot]['total_mass'], rel=1e-3)


@pytest.mark.parametrize('robot', list(ROBOTS))
def test_inertia_is_physical(robot):
    for link in expand(robot, 'false').findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        i = inertial.find('inertia').attrib
        m = np.array([[i['ixx'], i['ixy'], i['ixz']], [i['ixy'], i['iyy'], i['iyz']],
                      [i['ixz'], i['iyz'], i['izz']]], dtype=float)
        ev = np.linalg.eigvalsh(m)
        assert ev.min() > 0, link.get('name')
        assert ev[2] <= ev[0] + ev[1] + 1e-12, link.get('name')   # triangle inequality


@pytest.mark.parametrize('robot', list(ROBOTS))
def test_component_names_resolve(robot):
    comps = ROBOTS[robot]['components']
    for key, name in comps.items():
        if key == 'joint_actuators':
            for joint, part in name.items():
                assert (COMPONENTS / 'servo' / (part + '.yaml')).exists(), (robot, joint)
        else:
            assert (COMPONENTS / CATEGORY[key] / (name + '.yaml')).exists(), (robot, key)


def test_catalog_entries_have_sources():
    for path in COMPONENTS.glob('*/*.yaml'):
        entry = yaml.safe_load(path.read_text())
        assert entry['model'] and entry['category'], path
        assert entry.get('sources'), path


def test_mass_properties_current():
    tool = Path(__file__).resolve().parents[3] / 'tools' / 'mass_properties.py'
    if not tool.exists():
        pytest.skip('source tree not available (installed test)')
    done = subprocess.run(['python3', str(tool), '--check'], capture_output=True, text=True)
    assert done.returncode == 0, done.stdout + done.stderr
    assert importlib.util.find_spec('yaml')
