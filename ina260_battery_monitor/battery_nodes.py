"""Runs battery_monitor_node and battery_events_node in one process.

The two nodes always run together (events consumes the monitor's /battery_state), and each extra
Python process costs a few percent of a core in interpreter and middleware overhead even when idle.
Node names and parameters are unchanged, so the same params files apply.
"""

import rclpy
from ina260_battery_monitor.battery_events_node import BatteryEventsNode
from ina260_battery_monitor.battery_monitor_node import BatteryMonitorNode
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    monitor = BatteryMonitorNode()
    if monitor.hardware_missing:
        monitor.destroy_node()
        rclpy.try_shutdown()
        return
    events = BatteryEventsNode()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(events)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        events.destroy_node()
        monitor.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
