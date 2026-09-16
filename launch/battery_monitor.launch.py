#!/usr/bin/env python3
"""Launch the INA260 battery monitor and battery_events nodes.

battery_events_node depends on battery_monitor_node's /battery_state, so both are started
together here rather than from separate launch files - there's no meaningful way to run
one without the other. Both nodes read their own top-level key (battery_monitor_node /
battery_events_node) out of the same params_file, ROS 2's normal params-file behavior.

overrides_file is layered on top of params_file (ROS 2 launch loads parameters files in
order, later ones winning on overlapping keys) - it exists so a caller needing to override
one or two fields (e.g. lekiwi_bringup wanting a non-default state_file_path) can do that
with a small dedicated file instead of maintaining a full duplicate of this package's own
battery.yaml, which silently drifts out of sync the moment either copy is edited (found the
hard way - see lekiwi_bringup/config/battery_overrides.yaml's own comment). Defaults to
params_file itself, a harmless no-op re-load when no override is needed.

Example usage:
    ros2 launch ina260_battery_monitor battery_monitor.launch.py
    ros2 launch ina260_battery_monitor battery_monitor.launch.py params_file:=/path/to/my_battery.yaml
    ros2 launch ina260_battery_monitor battery_monitor.launch.py overrides_file:=/path/to/small_overrides.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=[FindPackageShare('ina260_battery_monitor'), '/config/battery.yaml'],
            description='Path to the battery_monitor_node/battery_events_node parameters YAML file',
        ),
        DeclareLaunchArgument(
            'overrides_file',
            default_value=LaunchConfiguration('params_file'),
            description=(
                'Optional small YAML layered on top of params_file for a caller-specific '
                'override (e.g. state_file_path) - defaults to params_file itself (no-op).'
            ),
        ),
    ]

    battery_monitor_node = Node(
        package='ina260_battery_monitor',
        executable='battery_monitor_node',
        name='battery_monitor_node',
        output='log',
        # No respawn: a missing INA260 makes the node exit cleanly on purpose (see
        # battery_monitor_node.py's module docstring) - that's a permanent condition for
        # this launch session, not a crash respawn should try to recover from. Genuine
        # transient I2C read errors during normal operation don't exit the process at
        # all (skip-and-retry in _on_timer), so respawn has nothing left to usefully do.
        respawn=False,
        parameters=[LaunchConfiguration('params_file'), LaunchConfiguration('overrides_file')],
    )

    battery_events_node = Node(
        package='ina260_battery_monitor',
        executable='battery_events_node',
        name='battery_events_node',
        output='log',
        respawn=True,
        parameters=[LaunchConfiguration('params_file'), LaunchConfiguration('overrides_file')],
    )

    return LaunchDescription(declared_arguments + [battery_monitor_node, battery_events_node])
