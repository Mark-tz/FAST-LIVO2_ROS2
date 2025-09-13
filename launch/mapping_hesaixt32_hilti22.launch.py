from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fast_livo')
    
    # Load parameters from YAML files
    config_file = PathJoinSubstitution([pkg_share, 'config', 'HILTI22.yaml'])
    camera_config_file = PathJoinSubstitution([pkg_share, 'config', 'camera_fisheye_HILTI22.yaml'])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'hilti.rviz'])
    
    # Main mapping node
    mapping_node = Node(
        package='fast_livo',
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
    
    return LaunchDescription([
        rviz_arg,
        mapping_node,
        rviz_node
    ])