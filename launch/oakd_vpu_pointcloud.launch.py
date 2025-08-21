#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_id',
            default_value='oakd_link',
            description='Frame ID for the point cloud'
        ),
        
        DeclareLaunchArgument(
            'downsample_factor',
            default_value='2',
            description='VPU downsampling factor (1-8). Higher values = less CPU load but lower resolution'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Publishing rate in Hz (default: 20Hz = 50ms)'
        ),
        
        Node(
            package='oakd_lite_pcl',
            executable='oakd_lite_pcl',
            name='oakd_vpu_pointcloud',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'downsample_factor': LaunchConfiguration('downsample_factor'),
            }],
            remappings=[
                ('/oak/points', '/oak/vpu_points')
            ]
        )
    ])
