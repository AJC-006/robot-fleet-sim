"""Gazebo + one ground robot + ros2_control + EKF + Nav2 (+ RViz).

  ros2 launch fleet_bringup sim.launch.py robot:=round world:=keep
      [nav:=true] [slam:=false] [rviz:=true] [gui:=true] [camera:=false]
      [spawn:=<name>] [x:=.. y:=.. yaw:=..]

Handles every `kind: wheeled` and `kind: legged` robot. use_sim_time is always true (the simulator clock) and the
nodes live in the global namespace: there is no `namespace` argument yet (planned with the Nav2/EKF retarget, G7 in
docs/notes/ground.md).

robot: any key of src/fleet_description/config/robots.yaml (its `kind:` picks the controller setup,
       default `wheeled`; see KINDS below)
world: any fleet_worlds worlds/<world>.sdf that has a matching <world>.yaml
spawn: optional name of a pose in the world yaml's `spawns` dict (default: its `spawn`)
x, y, yaw: optional spawn pose override, applied on top of the above
slam:=true maps with slam_toolbox instead of localising with AMCL on the prebuilt map.
gui:=false runs the Gazebo server only (headless rendering for the sensors).
"""
import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def write_yaml(data, directory, name):
    path = os.path.join(directory, name)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def deep_merge(base, extra):
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def inscribed_radius(polygon):
    """Distance from the origin (base_footprint) to the nearest edge of a closed polygon, in m."""
    def seg(p, q):
        dx, dy = q[0] - p[0], q[1] - p[1]
        t = max(0.0, min(1.0, -(p[0] * dx + p[1] * dy) / (dx * dx + dy * dy)))
        return math.hypot(p[0] + t * dx, p[1] + t * dy)
    return round(min(seg(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon))), 3)


def node_params(params):
    """Yield every ros__parameters dict, including nested ones like local_costmap.local_costmap."""
    for value in params.values():
        if isinstance(value, dict):
            if 'ros__parameters' in value:
                yield value['ros__parameters']
            yield from node_params({k: v for k, v in value.items() if k != 'ros__parameters'})


def wheeled_kind(spec, name=None):
    """Differential / skid-steer wheels driven by diff_drive_controller.

    Every kind takes (robots.yaml entry, robot name) and returns the same dict:
      controllers  controller_manager parameter file content
      spawn        [(controller name, extra spawner args)], started in order after joint_state_broadcaster
      odom         odometry topic fed to the EKF (odom0)
      ekf          extra EKF parameters (e.g. odom0_config)
      nav2         parameter overrides deep-merged into the Nav2 params (motion model, speed limits)
      post         extra launch actions to run once the controllers are up
      z            spawn height above the floor
    """
    joints = lambda wheels: [f'{w}_joint' for w in wheels]
    vx, wz = spec['max_vel_x'], spec['max_vel_theta']
    controllers = {
        'controller_manager': {'ros__parameters': {
            'update_rate': 100,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'diff_drive_controller': {'type': 'diff_drive_controller/DiffDriveController'},
        }},
        'diff_drive_controller': {'ros__parameters': {
            'left_wheel_names': joints(spec['left_wheels']),
            'right_wheel_names': joints(spec['right_wheels']),
            'wheel_separation': spec['wheel_separation'],
            'wheel_radius': spec['wheel_radius'],
            'wheel_separation_multiplier': spec.get('wheel_separation_multiplier', 1.0),
            'publish_rate': 50.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'enable_odom_tf': False,  # the EKF owns odom -> base_footprint
            'open_loop': False,
            'cmd_vel_timeout': 0.5,
            'pose_covariance_diagonal': [0.001, 0.001, 1e6, 1e6, 1e6, 0.03],
            'twist_covariance_diagonal': [0.001, 0.001, 1e6, 1e6, 1e6, 0.03],
            'linear.x.max_velocity': vx,
            'linear.x.min_velocity': -vx,
            'linear.x.max_acceleration': 1.5,
            'linear.x.min_acceleration': -1.5,
            'angular.z.max_velocity': wz,
            'angular.z.min_velocity': -wz,
            'angular.z.max_acceleration': 4.0,
            'angular.z.min_acceleration': -4.0,
        }},
    }
    nav2 = {
        'amcl': {'ros__parameters': {'robot_model_type': 'nav2_amcl::DifferentialMotionModel'}},
        'controller_server': {'ros__parameters': {'FollowPath': {
            'motion_model': 'DiffDrive', 'vx_max': vx, 'vx_min': -0.4 * vx, 'vy_max': 0.0, 'wz_max': wz}}},
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [vx, 0.0, wz], 'min_velocity': [-0.4 * vx, 0.0, -wz]}},
        'behavior_server': {'ros__parameters': {'max_rotational_vel': wz}},
    }
    # The installed diff_drive_controller warns "Velocity command timed out. Braking." once a second
    # whenever nobody is driving it (i.e. always while idle). Controllers have the logger service,
    # so raise just that logger to ERROR once it is up.
    quiet = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/diff_drive_controller/set_logger_levels',
             'rcl_interfaces/srv/SetLoggerLevels', "{levels: [{name: diff_drive_controller, level: 40}]}"],
        output='log')
    return {
        'controllers': controllers,
        'spawn': [('diff_drive_controller', ['--controller-ros-args', '-r /diff_drive_controller/cmd_vel:=/cmd_vel'])],
        'odom': '/diff_drive_controller/odom',
        'ekf': {},
        'nav2': nav2,
        'post': [quiet],
        'z': 0.01,
    }


def legged_kind(spec, name):
    """3-DOF legs, one position-controlled JointGroupPositionController, gait node from fleet_legged.

    gz_ros2_control turns a position command into a joint velocity command every physics step:
    v = -position_proportional_gain * (q - q_cmd) * update_rate (update_rate = the controller
    manager's), so gain * update_rate is the tracking bandwidth (1/s) and the joint `effort` limit
    in robots.yaml caps the force. The gait node (fleet_legged gait_node.py) stands the robot up
    from its spawn crouch, turns /cmd_vel into joint targets and publishes /leg_odom, which the EKF
    fuses like wheel odometry. Spawned crouched: base_footprint goes below the floor so that the feet
    (at crouch_height under base_link) just touch it.
    """
    vx, vy, wz = float(spec['max_vel_x']), float(spec.get('max_vel_y', 0.0)), float(spec['max_vel_theta'])
    joints = [f'{leg["name"]}_{j}_joint' for leg in spec['legs'] for j in ('coxa', 'femur', 'tibia')]
    rate, gain = 250, 0.5
    controllers = {
        'controller_manager': {'ros__parameters': {
            'update_rate': rate,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'leg_controller': {'type': 'position_controllers/JointGroupPositionController'},
        }},
        'joint_state_broadcaster': {'ros__parameters': {'update_rate': 125}},   # a divisor of `rate`
        'leg_controller': {'ros__parameters': {'joints': joints}},
        # declared by the plugin's own node, `gz_ros_control` (its log prints the value it uses)
        'gz_ros_control': {'ros__parameters': {'position_proportional_gain': gain, 'hold_joints': True}},
    }
    acc_lin, acc_ang = 0.5, 1.5
    nav2 = {
        # Leg odometry + IMU yaw is good to ~2-5 % (walk_test.py), far better than the alpha = 0.2
        # defaults assume. With the defaults the particle cloud spread to sd 0.5-0.7 m on a 10 m leg in
        # the warehouse and its mean wandered 0.2-0.5 m along the aisles (logged against Gazebo ground
        # truth), enough to put a 0.54 m wide spider "inside" a rack end. sigma_hit 0.1 / 120 beams and
        # an update every 0.15 m / 0.15 rad make the scans pull harder and more often.
        'amcl': {'ros__parameters': {
            'robot_model_type': 'nav2_amcl::OmniMotionModel',
            'alpha1': 0.05, 'alpha2': 0.02, 'alpha3': 0.02, 'alpha4': 0.02, 'alpha5': 0.02,
            'sigma_hit': 0.1, 'max_beams': 120, 'update_min_d': 0.15, 'update_min_a': 0.15}},
        'controller_server': {'ros__parameters': {
            # the default 0.5 m/s threshold zeroes every lateral command of these slow robots
            'min_y_velocity_threshold': 0.001,
            'progress_checker': {'required_movement_radius': 0.25, 'movement_time_allowance': 20.0},
            'FollowPath': {
                'motion_model': 'Omni', 'vx_max': vx, 'vx_min': -0.5 * vx, 'vy_max': vy, 'wz_max': wz,
                'vx_std': round(max(0.1, 0.7 * vx), 3), 'vy_std': round(max(0.05, 0.7 * vy), 3), 'wz_std': 0.4}}},
        # MPPI's ax_max/ay_max/az_max stay at Nav2's defaults (3 m/s^2) on purpose: MPPI starts every
        # rollout from the measured speed and clamps the first control to measured + a_max * model_dt.
        # With the real 0.5 m/s^2 that is +0.025 m/s per cycle, the softmax average eats it and the spider
        # cruised at 0.04 m/s (dock_north 118 s); with the defaults it cruises at ~0.2 m/s (17 s). The
        # velocity smoother and the gait node apply the real acceleration limit.
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [vx, vy, wz], 'min_velocity': [-0.5 * vx, -vy, -wz],
            'max_accel': [acc_lin, acc_lin, acc_ang], 'max_decel': [-acc_lin, -acc_lin, -acc_ang]}},
        'behavior_server': {'ros__parameters': {'max_rotational_vel': wz, 'min_rotational_vel': 0.2 * wz,
                                                'rotational_acc_lim': acc_ang}},
        # The global obstacle layer marks scan hits up to 19.5 m. A degree of heading error (a leg slips
        # on a rack corner) smears a far wall 0.2-0.3 m into the aisle, and those marks stay until a beam
        # crosses them again; they closed the warehouse east lane. Mark only near hits (still clear far).
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'obstacle_layer': {'scan': {'obstacle_max_range': 5.0}}}}},
        # the stock tree gives up at once on START_OCCUPIED / TF_ERROR; see the XML's header
        'bt_navigator': {'ros__parameters': {'default_nav_to_pose_bt_xml': os.path.join(
            get_package_share_directory('fleet_bringup'), 'config', 'bt', 'navigate_to_pose_recover_always.xml')}},
    }
    gait = Node(package='fleet_legged', executable='gait_node.py', name='gait_node', output='screen',
                parameters=[{'robot': name, 'accel_lin': acc_lin, 'accel_ang': acc_ang, 'rate': rate / 2.0,
                             'servo_lead': round(1.0 / (gain * rate), 4), 'use_sim_time': True}])
    return {
        'controllers': controllers,
        'spawn': [('leg_controller', [])],
        'odom': '/leg_odom',
        'ekf': {},
        'nav2': nav2,
        'post': [gait],
        'z': round(float(spec['crouch_height']) - float(spec['standing_height']) + 0.003, 4),
        # Global costmap circle = the footprint's inscribed radius (half the robot's narrow width) plus
        # 0.10 m, so the planner only routes through gaps at least ~0.2 m wider than the robot. An
        # earlier "inscribed - 0.10" let Smac send the 0.54 m spider through the 0.70 m slot between
        # the warehouse conveyor and the wall, where MPPI (real footprint) wedged it. Every world
        # bottleneck on the way to a goal is >= 0.50 m of clearance (apartment doors), above every
        # legged plan radius (wyrm 0.35, hound 0.29, spider 0.36). A centre that AMCL puts inside the
        # circle ("Start occupied") is handled by the BT's recoveries (clear, back up, wait, spin).
        'plan_radius': round(inscribed_radius(spec['footprint']) + 0.10, 3),
    }


KINDS = {'wheeled': wheeled_kind, 'legged': legged_kind}


def nav2_config(kind, spec, spawn, overrides_path, map_resolution, slam):
    params = load_yaml(os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'))
    if slam:  # a slam_toolbox section in the file replaces slam_toolbox's own defaults, so start from them
        params['slam_toolbox'] = load_yaml(os.path.join(
            get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_sync.yaml'))['slam_toolbox']
    deep_merge(params, load_yaml(overrides_path))
    deep_merge(params, kind['nav2'])
    # The static layer resizes the global costmap to the map's resolution (0.05 or 0.10 m/px), so say so;
    # the local costmap keeps its own 0.05.
    params['global_costmap']['global_costmap']['ros__parameters']['resolution'] = map_resolution

    for p in node_params(params):
        p['enable_stamped_cmd_vel'] = True
        if 'odom_topic' in p:
            p['odom_topic'] = '/odometry/filtered'

    footprint = str([[round(x, 3), round(y, 3)] for x, y in spec['footprint']])
    inflation = round(max(0.45, spec['robot_radius'] + 0.25), 2)
    # Local costmap + MPPI use the real footprint polygon. The global costmap uses a circle (the
    # circumscribed one, or the kind's `plan_radius`): Smac 2D checks a polygon footprint against the
    # already-inflated map, which flags "Start occupied" whenever the robot passes within two radii of a
    # wall; with a circle it is the usual inscribed test. The inflation radius always follows robot_radius.
    for costmap in ('local_costmap', 'global_costmap'):
        p = params[costmap][costmap]['ros__parameters']
        p.pop('robot_radius' if costmap == 'local_costmap' else 'footprint', None)
        if costmap == 'local_costmap':
            p['footprint'] = footprint
            p['footprint_padding'] = 0.01
        else:
            p['robot_radius'] = kind.get('plan_radius', spec['robot_radius'])
        if 'inflation_layer' in p:
            p['inflation_layer']['inflation_radius'] = inflation

    amcl = params['amcl']['ros__parameters']
    # float(): the world yaml has integers like `y: 0`, which AMCL rejects for a double parameter
    amcl['initial_pose'] = {'x': float(spawn['x']), 'y': float(spawn['y']), 'z': 0.0, 'yaw': float(spawn['yaw'])}
    return params


def available(share, subdir, suffix, need_yaml=False):
    """Names of <share>/<subdir>/*<suffix> (that also have a sibling .yaml when need_yaml)."""
    d = os.path.join(share, subdir)
    return sorted(f[:-len(suffix)] for f in os.listdir(d)
                  if f.endswith(suffix) and (not need_yaml or os.path.exists(os.path.join(d, f[:-len(suffix)] + '.yaml'))))


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    truthy = lambda name: arg(name).lower() in ('1', 'true', 'yes')
    robot, world = arg('robot'), arg('world')

    desc = get_package_share_directory('fleet_description')
    worlds = get_package_share_directory('fleet_worlds')
    bringup = get_package_share_directory('fleet_bringup')
    robots = load_yaml(os.path.join(desc, 'config', 'robots.yaml'))
    if robot not in robots:
        raise RuntimeError(f'robot:={robot} is not one of {list(robots)}')
    if world not in available(worlds, 'worlds', '.sdf', need_yaml=True):
        raise RuntimeError(f'world:={world} is not one of {available(worlds, "worlds", ".sdf", need_yaml=True)}')
    spec = robots[robot]
    kind_name = spec.get('kind', 'wheeled')
    if kind_name not in KINDS:
        raise RuntimeError(f'robot {robot} has kind {kind_name}, launch knows {list(KINDS)}')
    kind = KINDS[kind_name](spec, robot)
    meta = load_yaml(os.path.join(worlds, 'worlds', f'{world}.yaml'))
    named = meta.get('spawns', {}).get(arg('spawn'))
    if arg('spawn') and not named:
        print(f'[sim] world {world} has no spawn "{arg("spawn")}" (have {list(meta.get("spawns", {}))}); using `spawn`')
    spawn = dict(named or meta['spawn'])
    for key in ('x', 'y', 'yaw'):
        if arg(key):
            spawn[key] = float(arg(key))

    tmp = tempfile.mkdtemp(prefix=f'fleet_{robot}_{world}_')
    controllers_file = write_yaml(kind['controllers'], tmp, 'controllers.yaml')
    map_res = float(load_yaml(os.path.join(worlds, 'maps', meta['map']))['resolution'])
    nav2_file = write_yaml(nav2_config(kind, spec, spawn, os.path.join(bringup, 'config', 'nav2_overrides.yaml'),
                                       map_res, truthy('slam')), tmp, 'nav2_params.yaml')
    robot_description = xacro.process_file(
        os.path.join(desc, 'urdf', 'robot.urdf.xacro'),
        mappings={'robot': robot, 'camera': str(truthy('camera')).lower(),
                  'controllers_file': controllers_file}).toxml()
    sim_time = {'use_sim_time': True}

    gz_args = f'-r -v 2 {os.path.join(worlds, "worlds", f"{world}.sdf")}'
    if not truthy('gui'):
        gz_args += ' -s --headless-rendering'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                   'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description}, sim_time])
    spawn_robot = Node(package='ros_gz_sim', executable='create', output='screen',
                       arguments=['-topic', 'robot_description', '-name', robot, '-allow_renaming', 'false',
                                  '-x', str(float(spawn['x'])), '-y', str(float(spawn['y'])),
                                  '-z', str(kind['z']), '-Y', str(float(spawn['yaw']))])

    bridges = [os.path.join(bringup, 'config', 'gz_bridge.yaml')]
    if truthy('camera'):
        bridges.append(os.path.join(bringup, 'config', 'gz_bridge_camera.yaml'))
    bridge_nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                         name=f'gz_bridge_{i}', parameters=[{'config_file': b}, sim_time])
                    for i, b in enumerate(bridges)]

    # driver-accurate sensor topics from the raw gz ones; component names come from robots.yaml `components`
    comps = spec['components']
    adapters = [
        Node(package='fleet_sensor_sim', executable='lidar_driver_sim', name='lidar_driver_sim',
             output='screen', parameters=[{'component': comps['lidar']}, sim_time]),
        Node(package='fleet_sensor_sim', executable='imu_driver_sim', name='imu_driver_sim',
             output='screen', parameters=[{'component': comps['imu']}, sim_time])]
    if truthy('camera'):
        adapters.append(Node(package='fleet_sensor_sim', executable='camera_driver_sim',
                             name='camera_driver_sim', output='screen',
                             parameters=[{'component': comps['camera']}, sim_time]))

    spawner = lambda name, *extra: Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[name, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60', *extra])
    spawners = [spawner('joint_state_broadcaster')] + [spawner(name, *extra) for name, extra in kind['spawn']]

    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
               parameters=[os.path.join(bringup, 'config', 'ekf.yaml'), {'odom0': kind['odom'], **kind['ekf']},
                           sim_time])

    after_drive = [ekf, *kind['post']]
    if truthy('nav'):
        nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        after_drive.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': str(truthy('slam')),
                'map': os.path.join(worlds, 'maps', meta['map']),
                'params_file': nav2_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'True',
            }.items()))

    # spawn_robot -> joint_state_broadcaster -> each controller in turn -> EKF, Nav2, extras
    chain = [spawn_robot, *spawners]
    actions = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(worlds, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc)),
        gazebo, rsp, spawn_robot, *bridge_nodes, *adapters,
        *[RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[b]))
          for a, b in zip(chain, chain[1:] + [None]) if b],
        RegisterEventHandler(OnProcessExit(target_action=chain[-1], on_exit=after_drive)),
    ]
    if truthy('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', os.path.join(bringup, 'rviz', 'nav.rviz')],
                            parameters=[sim_time]))
    print(f'[sim] robot={robot} ({kind_name}) world={world} spawn=({spawn["x"]}, {spawn["y"]}, '
          f'{math.degrees(spawn["yaw"]):.0f} deg) generated params in {tmp}')
    return actions


def generate_launch_description():
    robots = list(load_yaml(os.path.join(get_package_share_directory('fleet_description'),
                                         'config', 'robots.yaml')))
    worlds = available(get_package_share_directory('fleet_worlds'), 'worlds', '.sdf', need_yaml=True)
    args = [
        DeclareLaunchArgument('robot', default_value='round', description='robot from config/robots.yaml (wheeled or legged): ' + ' | '.join(robots)),
        DeclareLaunchArgument('world', default_value='keep', description='world from fleet_worlds/worlds: ' + ' | '.join(worlds)),
        DeclareLaunchArgument('nav', default_value='true', description='start Nav2 (AMCL on the world map, or slam_toolbox with slam:=true)'),
        DeclareLaunchArgument('slam', default_value='false', description='slam_toolbox instead of AMCL + map'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz2 (false for headless runs)'),
        DeclareLaunchArgument('gui', default_value='true', description='true opens the Gazebo GUI; false runs the server only (headless rendering for the sensors)'),
        DeclareLaunchArgument('camera', default_value='false', description='add the RGB-D camera (catalog `camera` component of the robot)'),
        DeclareLaunchArgument('spawn', default_value='', description="named pose from the world yaml's `spawns`"),
        DeclareLaunchArgument('x', default_value='', description='spawn x override (m, world frame)'),
        DeclareLaunchArgument('y', default_value='', description='spawn y override'),
        DeclareLaunchArgument('yaw', default_value='', description='spawn yaw override (rad)'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
