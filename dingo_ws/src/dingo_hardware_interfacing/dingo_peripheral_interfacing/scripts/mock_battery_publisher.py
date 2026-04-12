#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from dingo_peripheral_msgs.msg import ElectricalMeasurements
from sensor_msgs.msg import BatteryState


class MockBatteryPublisher(Node):
    def __init__(self):
        super().__init__('mock_battery_publisher')

        self._topic = self.declare_parameter(
            'battery_topic',
            '/electrical_measurements',
        ).value
        self._battery_state_topic = self.declare_parameter(
            'battery_state_topic',
            '/battery_state',
        ).value
        self._battery_voltage = float(
            self.declare_parameter('battery_voltage_level', 16.4).value
        )
        self._servo_buck_voltage = float(
            self.declare_parameter('servo_buck_voltage_level', 5.0).value
        )
        self._publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 2.0).value
        )
        self._drain_per_second = float(
            self.declare_parameter('drain_per_second', 0.0).value
        )
        self._min_battery_voltage = float(
            self.declare_parameter('min_battery_voltage', 14.0).value
        )

        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 2.0

        self._publisher = self.create_publisher(ElectricalMeasurements, self._topic, 10)
        self._battery_state_publisher = self.create_publisher(
            BatteryState,
            self._battery_state_topic,
            10,
        )
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish)

        self.get_logger().info(
            f'Mock battery publisher active on {self._topic} '
            f'(battery={self._battery_voltage:.2f} V, servo_buck={self._servo_buck_voltage:.2f} V)'
        )

    def _publish(self):
        msg = ElectricalMeasurements()
        msg.battery_voltage_level = float(self._battery_voltage)
        msg.servo_buck_voltage_level = float(self._servo_buck_voltage)
        self._publisher.publish(msg)

        battery_state = BatteryState()
        battery_state.voltage = float(self._battery_voltage)
        battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery_state.present = True

        denom = 16.8 - self._min_battery_voltage
        if denom > 0.0:
            battery_state.percentage = float(
                max(0.0, min(1.0, (self._battery_voltage - self._min_battery_voltage) / denom))
            )
        else:
            battery_state.percentage = 0.0

        self._battery_state_publisher.publish(battery_state)

        if self._drain_per_second > 0.0:
            self._battery_voltage = max(
                self._min_battery_voltage,
                self._battery_voltage - self._drain_per_second / self._publish_rate_hz,
            )


def main(args=None):
    rclpy.init(args=args)
    node = MockBatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()