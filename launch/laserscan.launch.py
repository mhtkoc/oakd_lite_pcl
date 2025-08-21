import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for transform parameters
        DeclareLaunchArgument('x', default_value='0.0', description='X translation'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y translation'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z translation'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Roll rotation (radians)'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch rotation (radians)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw rotation (radians)'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug point cloud'),
        
        # Static transform from map->oakd_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_oakd_base',
            arguments=[
                '--x', LaunchConfiguration('x'),
                '--y', LaunchConfiguration('y'), 
                '--z', LaunchConfiguration('z'),
                '--roll', LaunchConfiguration('roll'),
                '--pitch', LaunchConfiguration('pitch'),
                '--yaw', LaunchConfiguration('yaw'),
                '--frame-id', 'map',
                '--child-frame-id', 'oakd_link'
            ],
            output='screen',
        ),
        
        # LaserScan node
        Node(
            package='oakd_lite_pcl',
            executable='laserscan_cpp',
            name='oak_laserscan',
            output='screen',
            parameters=[
                {'frame_id': 'oakd_link'},
                {'debug': LaunchConfiguration('debug')}
            ],
        ),
        
        # İsteğe bağlı: RViz (varsayılan false)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
