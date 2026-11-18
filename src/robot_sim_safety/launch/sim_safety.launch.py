import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    allow_simulation = LaunchConfiguration('allow_simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'allow_simulation',
            default_value='true',
            description='Must be true to run simulation safety node'
        ),
        Node(
            package='robot_sim_safety',
            executable='sim_safety_node',
            name='sim_safety_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'allow_simulation': allow_simulation,
            }]
        )
    ])

