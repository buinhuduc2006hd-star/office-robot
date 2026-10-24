import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ws_config = os.path.expanduser('~/office_robot_ws/config/rooms.yaml')
    src_config = os.path.expanduser('~/office_robot_ws/src/office_core/config/rooms.yaml')
    legacy_ws_config = os.path.expanduser('~/robot_ws/config/rooms.yaml')
    try:
        pkg_share = get_package_share_directory('office_core')
        default_config = os.path.join(pkg_share, 'config', 'rooms.yaml')
    except Exception:
        default_config = ws_config

    if os.path.exists(ws_config):
        default_config = ws_config
    elif os.path.exists(src_config):
        default_config = src_config
    elif os.path.exists(legacy_ws_config):
        default_config = legacy_ws_config

    use_schedule = LaunchConfiguration('use_schedule')
    use_rosbridge = LaunchConfiguration('use_rosbridge')
    config_file = LaunchConfiguration('config_file')

    declare_use_schedule = DeclareLaunchArgument(
        'use_schedule',
        default_value='false',
        description='Whether to run schedule_manager auto toggle (default: false)'
    )
    declare_use_rosbridge = DeclareLaunchArgument(
        'use_rosbridge',
        default_value='false',
        description='Whether to run rosbridge websocket server (default: false)'
    )
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to rooms.yaml configuration file'
    )

    config_manager_node = Node(
        package='office_core',
        executable='config_manager',
        name='config_manager',
        parameters=[{'config_file': config_file}],
        output='screen'
    )

    room_access_manager_node = Node(
        package='office_core',
        executable='room_access_manager',
        name='room_access_manager',
        output='screen'
    )

    schedule_manager_node = Node(
        package='office_core',
        executable='schedule_manager',
        name='schedule_manager',
        condition=IfCondition(use_schedule),
        output='screen'
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        condition=IfCondition(use_rosbridge),
        output='screen'
    )

    return LaunchDescription([
        declare_use_schedule,
        declare_use_rosbridge,
        declare_config_file,
        config_manager_node,
        room_access_manager_node,
        schedule_manager_node,
        rosbridge_node
    ])
