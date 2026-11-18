"""Replace the existing two stack processes; never launch duplicate nav owners."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    allow_motion = ParameterValue(LaunchConfiguration("allow_motion"), value_type=bool)
    common = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("allow_motion", default_value="false"),
        DeclareLaunchArgument("edge_state_path", default_value=os.path.expanduser("~/.ros/office_robot_edge.sqlite3")),
        DeclareLaunchArgument("navigation_ledger_path", default_value=os.path.expanduser("~/.ros/office_robot_navigation.sqlite3")),
        DeclareLaunchArgument("sim_safety", default_value="true"),
        Node(package="robot_sim_safety", executable="sim_safety_node", output="screen",
             parameters=[common, {"allow_simulation": True}],
             condition=IfCondition(LaunchConfiguration("sim_safety"))),
        Node(package="robot_system_monitor", executable="system_monitor_node", output="screen",
             parameters=[common]),
        Node(package="robot_mission_control", executable="mission_controller_node", output="screen",
             parameters=[common, {"delivery_integration_enabled": True}]),
        Node(package="robot_navigation_manager", executable="navigation_manager_node", output="screen",
             parameters=[common, {"delivery_goal_ledger_path": LaunchConfiguration("navigation_ledger_path")}]),
        Node(package="office_robot_edge_gateway", executable="gateway", output="screen",
             parameters=[common, {"allow_motion": allow_motion, "local_state_path": LaunchConfiguration("edge_state_path")}]),
    ])
