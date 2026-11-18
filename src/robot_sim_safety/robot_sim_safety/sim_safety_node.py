#!/usr/bin/env python3
"""
Simulation Safety Node for Office Robot in Gazebo.
Provides controlled simulation of:
  - Battery (/battery_state)
  - Water (/water/status)
  - Emergency Stop (/emergency_stop)
Controls:
  - Services to trigger/release E-stop
  - Services to toggle low battery and low water states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from robot_interfaces.msg import WaterStatus


class SimSafetyNode(Node):
    def __init__(self, **kwargs):
        super().__init__('sim_safety_node', **kwargs)

        # Safety Guard: Ensure node is only run in simulation
        if not self.has_parameter('allow_simulation'):
            self.declare_parameter('allow_simulation', True)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        allow_sim = self.get_parameter('allow_simulation').value
        use_sim_time = self.get_parameter('use_sim_time').value

        if not allow_sim or not use_sim_time:
            self.get_logger().error(
                'SimSafetyNode CANNOT be run on physical hardware! '
                'allow_simulation and use_sim_time must both be True.'
            )
            raise RuntimeError('SimSafetyNode rejected running outside simulation mode.')

        # State Variables
        # A2: E-stop starts ACTIVE (True), must be explicitly released by operator
        self.estop_active = True

        # A2: Battery default 80%, battery_low=False
        self.battery_percentage = 0.80  # 0.0 to 1.0
        self.battery_voltage = 24.0
        self.battery_low = False

        # A2: Water default 80%, water_low=False, kettle_present=True, sensor_ok=True
        self.water_percent = 80.0
        self.water_liter = 4.0
        self.water_low = False
        self.water_sensor_ok = True
        self.kettle_present = True

        # Publishers
        self.pub_battery = self.create_publisher(BatteryState, '/battery_state', 10)
        self.pub_water = self.create_publisher(WaterStatus, '/water/status', 10)
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)

        # Services for E-stop control
        self.srv_trigger_estop = self.create_service(
            Trigger, '/sim_safety/trigger_estop', self.cb_trigger_estop)
        self.srv_release_estop = self.create_service(
            Trigger, '/sim_safety/release_estop', self.cb_release_estop)
        self.srv_set_estop = self.create_service(
            SetBool, '/sim_safety/set_estop', self.cb_set_estop)

        # Services for Battery control
        self.srv_set_battery_low = self.create_service(
            SetBool, '/sim_safety/set_battery_low', self.cb_set_battery_low)

        # Services for Water control
        self.srv_set_water_low = self.create_service(
            SetBool, '/sim_safety/set_water_low', self.cb_set_water_low)
        self.srv_set_water_sensor = self.create_service(
            SetBool, '/sim_safety/set_water_sensor_ok', self.cb_set_water_sensor_ok)

        # Periodic Publisher Timer (10 Hz for continuous fresh topics)
        self.timer = self.create_timer(0.1, self.publish_all)

        self.get_logger().info(
            'SimSafetyNode initialized: E-stop=ACTIVE (True), Battery=80%, Water=80%.'
        )

    # Service Callbacks
    def cb_trigger_estop(self, request, response):
        self.estop_active = True
        self.get_logger().warn('[SIM_SAFETY] Emergency Stop ACTIVATED by operator!')
        response.success = True
        response.message = 'Emergency Stop ACTIVATED'
        return response

    def cb_release_estop(self, request, response):
        self.estop_active = False
        self.get_logger().info('[SIM_SAFETY] Emergency Stop RELEASED by operator.')
        response.success = True
        response.message = 'Emergency Stop RELEASED'
        return response

    def cb_set_estop(self, request, response):
        self.estop_active = bool(request.data)
        state_str = 'ACTIVATED' if self.estop_active else 'RELEASED'
        self.get_logger().info(f'[SIM_SAFETY] Emergency Stop set to {state_str}')
        response.success = True
        response.message = f'Emergency Stop {state_str}'
        return response

    def cb_set_battery_low(self, request, response):
        if request.data:
            self.battery_low = True
            self.battery_percentage = 0.10  # 10%
            self.battery_voltage = 20.0
            msg = 'Battery set to LOW (10%)'
        else:
            self.battery_low = False
            self.battery_percentage = 0.80  # 80%
            self.battery_voltage = 24.0
            msg = 'Battery set to NORMAL (80%)'
        self.get_logger().info(f'[SIM_SAFETY] {msg}')
        response.success = True
        response.message = msg
        return response

    def cb_set_water_low(self, request, response):
        if request.data:
            self.water_low = True
            self.water_percent = 10.0  # 10%
            self.water_liter = 0.5
            msg = 'Water set to LOW (10%)'
        else:
            self.water_low = False
            self.water_percent = 80.0  # 80%
            self.water_liter = 4.0
            msg = 'Water set to NORMAL (80%)'
        self.get_logger().info(f'[SIM_SAFETY] {msg}')
        response.success = True
        response.message = msg
        return response

    def cb_set_water_sensor_ok(self, request, response):
        self.water_sensor_ok = bool(request.data)
        state_str = 'OK' if self.water_sensor_ok else 'FAULT'
        self.get_logger().info(f'[SIM_SAFETY] Water sensor set to {state_str}')
        response.success = True
        response.message = f'Water sensor {state_str}'
        return response

    # Periodic Publication
    def publish_all(self):
        stamp = self.get_clock().now().to_msg()

        # 1. Emergency Stop
        msg_estop = Bool()
        msg_estop.data = self.estop_active
        self.pub_estop.publish(msg_estop)

        # 2. Battery State
        msg_bat = BatteryState()
        msg_bat.header.stamp = stamp
        msg_bat.header.frame_id = 'base_footprint'
        msg_bat.voltage = float(self.battery_voltage)
        msg_bat.percentage = float(self.battery_percentage)
        msg_bat.present = True
        msg_bat.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg_bat.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.pub_battery.publish(msg_bat)

        # 3. Water Status
        msg_water = WaterStatus()
        msg_water.water_percent = float(self.water_percent)
        msg_water.water_liter = float(self.water_liter)
        msg_water.water_low = bool(self.water_low)
        msg_water.sensor_ok = bool(self.water_sensor_ok)
        msg_water.kettle_present = bool(self.kettle_present)
        self.pub_water.publish(msg_water)


def main(args=None):
    rclpy.init(args=args)
    node = SimSafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
