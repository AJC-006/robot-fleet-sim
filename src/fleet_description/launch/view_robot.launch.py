"""View a ground robot in RViz: ros2 launch fleet_description view_robot.launch.py robot:=wyrm"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('fleet_description')
    xacro_file = os.path.join(share, 'urdf', 'robot.urdf.xacro')
    rviz_cfg = os.path.join(share, 'rviz', 'view_robot.rviz')

    robot = LaunchConfiguration('robot')
    camera = LaunchConfiguration('camera')

    description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot:=', robot, ' camera:=', camera]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='round',
                              description='round | box | rover | truck | wyrm | hound | spider'),
        DeclareLaunchArgument('camera', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg]),
    ])
