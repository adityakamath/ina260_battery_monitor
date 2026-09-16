"""Publishes sensor_msgs/BatteryState from an INA260 current/voltage/power sensor.

Polls the INA260 on a timer, feeds each reading into a CoulombCounter for
charge/percentage estimation, and derives BatteryState's status/health fields from
the current sign and configured voltage cutoffs. See coulomb_counter.py for the
charge-estimation strategy and README.md for parameter documentation.

If no INA260 answers at startup (wrong I2C address, not fitted at all), the node logs
one warning and exits cleanly instead of raising - for a fleet where some robots don't
have this sensor, that's a much friendlier failure mode than a startup crash/traceback
if whoever launches it forgets to set battery_monitor:=false.
"""

import os
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ina260_battery_monitor.battery_eta import BatteryETA
from ina260_battery_monitor.coulomb_counter import CoulombCounter
from ina260_battery_monitor.ina260_sensor import INA260Error, INA260Sensor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

# Package-relative, not ~/.ros: resolved from this file's own location so it works
# wherever the package is checked out/installed, and stays gitignored in-place.
_DEFAULT_STATE_PATH = str(Path(__file__).resolve().parent.parent / 'state' / 'state.yaml')

_TECHNOLOGY_BY_NAME = {
    'LIPO': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
    'LION': BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
}


def _format_duration(seconds: float) -> str:
    minutes = seconds / 60.0
    if minutes < 60.0:
        return f'{minutes:.0f} min'
    return f'{minutes / 60.0:.1f} h'


class BatteryMonitorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('battery_monitor_node', **kwargs)

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        # 20Hz, not 2Hz: charger_presence.py's rest-window is expressed in sample count, not
        # wall-clock time, so a faster publish rate directly shortens its detection latency
        # (10 samples: 5s at 2Hz -> 0.5s here) without weakening the debounce itself. See
        # eta_current_ema_alpha below, which is scaled to compensate.
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('current_polarity_inverted', False)

        self.declare_parameter('chemistry', 'LION')
        self.declare_parameter('cell_count', 3)
        self.declare_parameter('design_capacity_ah', 2.55)
        self.declare_parameter('voltage_min_per_cell', 3.0)
        self.declare_parameter('voltage_max_per_cell', 4.2)

        self.declare_parameter('idle_current_threshold_a', 0.05)
        self.declare_parameter('rest_current_threshold_a', 0.05)
        self.declare_parameter('rest_settle_s', 60.0)
        self.declare_parameter('state_file_path', _DEFAULT_STATE_PATH)
        self.declare_parameter('state_save_interval_s', 30.0)
        self.declare_parameter('state_max_stale_s', 3600.0)

        self.declare_parameter('serial_number', '')
        self.declare_parameter('location', '')

        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        # EMA time constant is roughly dt/alpha - scaled down from 0.1 to hold the same ~5s
        # smoothing now that publish_rate_hz went from 2Hz to 20Hz (dt shrank 10x, so alpha
        # must too, or this would respond 10x faster than it was tuned for).
        self.declare_parameter('eta_current_ema_alpha', 0.01)

        chemistry = self.get_parameter('chemistry').value
        if chemistry not in _TECHNOLOGY_BY_NAME:
            raise ValueError(
                f'chemistry {chemistry!r} not supported, must be one of '
                f'{sorted(_TECHNOLOGY_BY_NAME)}'
            )

        self._cell_count = self.get_parameter('cell_count').value
        self._design_capacity_ah = self.get_parameter('design_capacity_ah').value
        self._voltage_min_per_cell = self.get_parameter('voltage_min_per_cell').value
        self._voltage_max_per_cell = self.get_parameter('voltage_max_per_cell').value
        self._idle_current_threshold_a = self.get_parameter('idle_current_threshold_a').value
        self._current_polarity_inverted = self.get_parameter('current_polarity_inverted').value
        self._technology = _TECHNOLOGY_BY_NAME[chemistry]
        self._frame_id = self.get_parameter('frame_id').value
        self._serial_number = self.get_parameter('serial_number').value
        self._location = self.get_parameter('location').value
        self._state_save_interval_s = self.get_parameter('state_save_interval_s').value

        state_path = os.path.expanduser(self.get_parameter('state_file_path').value)
        self._coulomb_counter = CoulombCounter(
            design_capacity_ah=self._design_capacity_ah,
            cell_count=self._cell_count,
            chemistry=chemistry,
            rest_current_threshold_a=self.get_parameter('rest_current_threshold_a').value,
            rest_settle_s=self.get_parameter('rest_settle_s').value,
            state_path=state_path,
            state_max_stale_s=self.get_parameter('state_max_stale_s').value,
        )
        self._battery_eta = BatteryETA(
            idle_current_threshold_a=self._idle_current_threshold_a,
            current_ema_alpha=self.get_parameter('eta_current_ema_alpha').value,
        )
        self._publish_diagnostics_enabled = self.get_parameter('publish_diagnostics').value

        self._sensor = None
        self._hardware_missing = False
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        try:
            sensor = INA260Sensor(bus_number=i2c_bus, address=i2c_address)
            sensor.check_identity()
        except INA260Error as exc:
            self.get_logger().warning(
                f'No INA260 detected on I2C bus {i2c_bus} address 0x{i2c_address:02X} '
                f'({exc}). Shutting down without publishing battery_state - set '
                f'battery_monitor:=false if this robot has no INA260 fitted.'
            )
            self._hardware_missing = True
            return
        self._sensor = sensor

        self._publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        if self._publish_diagnostics_enabled:
            diagnostics_topic = self.get_parameter('diagnostics_topic').value
            self._diagnostics_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)

        self._last_read_monotonic = None
        self._last_save_monotonic = time.monotonic()

        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f'ina260_battery_monitor started: bus={self.get_parameter("i2c_bus").value} '
            f'address=0x{self.get_parameter("i2c_address").value:02X} '
            f'chemistry={chemistry} cells={self._cell_count} '
            f'capacity={self._design_capacity_ah}Ah'
        )

    @property
    def hardware_missing(self) -> bool:
        return self._hardware_missing

    def _on_timer(self):
        try:
            voltage, current, _power = self._sensor.read()
        except INA260Error as exc:
            self.get_logger().error(f'INA260 read failed, skipping this cycle: {exc}')
            return

        if self._current_polarity_inverted:
            current = -current

        now = time.monotonic()
        dt_s = 0.0 if self._last_read_monotonic is None else now - self._last_read_monotonic
        self._last_read_monotonic = now

        self._coulomb_counter.update(voltage, current, dt_s)

        if now - self._last_save_monotonic >= self._state_save_interval_s:
            self._coulomb_counter.save_state()
            self._last_save_monotonic = now

        self._publisher.publish(self._to_battery_state(voltage, current))

        if self._publish_diagnostics_enabled:
            self._battery_eta.update(current)
            self._publish_battery_diagnostics(voltage)

    def _publish_battery_diagnostics(self, voltage: float) -> None:
        """Publish time-to-full/time-to-empty as a DiagnosticArray entry on /diagnostics.

        sensor_msgs/BatteryState has no field for this, so it goes out as KeyValue pairs
        instead of growing a custom message - matching waypoint_recorder_node's own
        DiagnosticArray convention for the same kind of derived, non-BatteryState data.
        """
        time_to_full_s, time_to_empty_s = self._battery_eta.estimate(
            self._coulomb_counter.charge_ah, self._design_capacity_ah
        )

        status = DiagnosticStatus()
        status.name = 'battery: ina260'
        status.hardware_id = self._serial_number
        status.level = (
            DiagnosticStatus.OK
            if self._health(voltage) == BatteryState.POWER_SUPPLY_HEALTH_GOOD
            else DiagnosticStatus.ERROR
        )

        values = [
            KeyValue(key='smoothed_current_a', value=f'{self._battery_eta.smoothed_current_a:.3f}')
        ]
        if time_to_full_s is not None:
            values.append(KeyValue(key='time_to_full_sec', value=f'{time_to_full_s:.0f}'))
            status.message = f'Charging - {_format_duration(time_to_full_s)} to full'
        elif time_to_empty_s is not None:
            values.append(KeyValue(key='time_to_empty_sec', value=f'{time_to_empty_s:.0f}'))
            status.message = f'Discharging - {_format_duration(time_to_empty_s)} to empty'
        else:
            status.message = 'Idle - no ETA (current within idle threshold)'
        status.values = values

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diagnostics_pub.publish(msg)

    def _to_battery_state(self, voltage: float, current: float) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.voltage = voltage
        msg.temperature = float('nan')
        msg.current = current
        msg.charge = self._coulomb_counter.charge_ah
        msg.capacity = self._design_capacity_ah
        msg.design_capacity = self._design_capacity_ah
        msg.percentage = self._coulomb_counter.percentage
        msg.power_supply_status = self._status(current)
        msg.power_supply_health = self._health(voltage)
        msg.power_supply_technology = self._technology
        msg.present = True
        msg.cell_voltage = []
        msg.cell_temperature = []
        msg.location = self._location
        msg.serial_number = self._serial_number
        return msg

    def _status(self, current: float) -> int:
        if abs(current) <= self._idle_current_threshold_a:
            if self._coulomb_counter.percentage >= 0.99:
                return BatteryState.POWER_SUPPLY_STATUS_FULL
            return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        if current > 0:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

    def _health(self, voltage: float) -> int:
        cell_voltage = voltage / self._cell_count
        if cell_voltage < self._voltage_min_per_cell:
            return BatteryState.POWER_SUPPLY_HEALTH_DEAD
        if cell_voltage > self._voltage_max_per_cell:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        return BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def destroy_node(self):
        if self._sensor is not None:
            self._coulomb_counter.save_state()
            self._sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    if node.hardware_missing:
        # No error, no traceback - the warning already logged in __init__ says why.
        # A clean exit here also means launch's respawn (if enabled) won't keep retrying
        # a condition that polling can't fix: the hardware is either there or it isn't.
        node.destroy_node()
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
