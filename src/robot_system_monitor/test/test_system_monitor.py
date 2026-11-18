import unittest
import math
import time
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_interfaces.msg import WaterStatus, RobotStatus
from robot_system_monitor.system_monitor_node import SystemMonitorNode


class TestSystemMonitor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SystemMonitorNode(parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, False)
        ])

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_fail_closed_state(self):
        # Before any data arrives, everything must fail closed
        self.assertFalse(self.node.lidar_ok)
        self.assertFalse(self.node.odom_ok)
        self.assertFalse(self.node.localization_ok)
        self.assertTrue(self.node.battery_low)
        self.assertTrue(self.node.water_low)

    def test_lidar_validation(self):
        scan = LaserScan()
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.range_min = 0.1
        scan.range_max = 10.0

        # Empty ranges
        scan.ranges = []
        self.node.cb_scan(scan)
        self.assertFalse(self.node.scan_valid)

        # All NaNs/Infs
        scan.ranges = [float('nan'), float('inf'), -1.0]
        self.node.cb_scan(scan)
        self.assertFalse(self.node.scan_valid)

        # Valid ranges
        scan.ranges = [float('nan'), 2.5, 3.0]
        self.node.cb_scan(scan)
        self.assertTrue(self.node.scan_valid)
        self.assertIsNotNone(self.node.last_scan_time)

    def test_odom_validation(self):
        odom = Odometry()
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.1
        odom.twist.twist.angular.z = 0.05

        self.node.cb_odom(odom)
        self.assertTrue(self.node.odom_valid)

        # Invalid quaternion norm
        odom.pose.pose.orientation.w = 5.0
        self.node.cb_odom(odom)
        self.assertFalse(self.node.odom_valid)

        # NaN in position
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = float('nan')
        self.node.cb_odom(odom)
        self.assertFalse(self.node.odom_valid)

    def test_amcl_covariance_validation(self):
        amcl = PoseWithCovarianceStamped()
        amcl.pose.pose.position.x = 0.5
        amcl.pose.pose.orientation.w = 1.0
        # Set good covariance
        cov = [0.0] * 36
        cov[0] = 0.2    # var x
        cov[7] = 0.2    # var y
        cov[35] = 0.05  # var yaw
        amcl.pose.covariance = cov

        self.node.cb_amcl(amcl)
        self.assertTrue(self.node.amcl_valid)

        # Divergent covariance
        cov[0] = 10.0  # Exceeds MAX_POS_VARIANCE = 5.0
        amcl.pose.covariance = cov
        self.node.cb_amcl(amcl)
        self.assertFalse(self.node.amcl_valid)

    def test_battery_fail_closed_and_valid(self):
        bat = BatteryState()
        bat.voltage = 24.0
        bat.present = True
        bat.percentage = 0.80
        bat.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        self.node.cb_battery(bat)
        self.assertTrue(self.node.battery_valid)
        self.assertFalse(self.node.battery_low)
        self.assertAlmostEqual(self.node.battery_percent, 80.0)

        # Low battery
        bat.percentage = 0.15
        self.node.cb_battery(bat)
        self.assertTrue(self.node.battery_valid)
        self.assertTrue(self.node.battery_low)
        self.assertAlmostEqual(self.node.battery_percent, 15.0)

        # Dead battery health
        bat.percentage = 0.80
        bat.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        self.node.cb_battery(bat)
        self.assertTrue(self.node.battery_low)

    def test_water_fail_closed_and_valid(self):
        water = WaterStatus()
        water.water_percent = 80.0
        water.water_liter = 4.0
        water.water_low = False
        water.sensor_ok = True

        self.node.cb_water(water)
        self.assertTrue(self.node.water_valid)
        self.assertFalse(self.node.water_low)

        # Sensor failure -> must fail closed to water_low = True
        water.sensor_ok = False
        self.node.cb_water(water)
        self.assertTrue(self.node.water_low)

        # Low water
        water.sensor_ok = True
        water.water_percent = 10.0
        water.water_liter = 0.5
        water.water_low = True
        self.node.cb_water(water)
        self.assertTrue(self.node.water_low)

    def test_watchdog_timeout_fail_closed(self):
        # First set good battery
        bat = BatteryState()
        bat.voltage = 24.0
        bat.present = True
        bat.percentage = 0.80
        bat.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.node.cb_battery(bat)
        self.assertFalse(self.node.battery_low)

        # Fake monotonic time expiry
        self.node.last_battery_time = time.monotonic() - 5.0
        self.node.publish_status()
        self.assertTrue(self.node.battery_low)
        self.assertEqual(self.node.battery_percent, 0.0)


if __name__ == '__main__':
    unittest.main()

