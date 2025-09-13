from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fast_livo2')
    
    # Load parameters from YAML files
    config_file = PathJoinSubstitution([pkg_share, 'config', 'avia.yaml'])
    camera_config_file = PathJoinSubstitution([pkg_share, 'config', 'camera_pinhole.yaml'])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'fast_livo2.rviz'])
    
    # Main mapping node
    mapping_node = Node(
        package='fast_livo2',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_file, camera_config_file]
    )
    
    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )
    
    # Image republish node
    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        arguments=['compressed', 'in:=/left_camera/image', 'raw', 'out:=/left_camera/image'],
        output='screen',
        respawn=True
    )
    
    return LaunchDescription([
        rviz_arg,
        mapping_node,
        rviz_node,
        republish_node
    ])