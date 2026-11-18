import unittest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from robot_interfaces.msg import WaterStatus
from robot_sim_safety.sim_safety_node import SimSafetyNode


class TestSimSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SimSafetyNode(parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True),
            Parameter('allow_simulation', Parameter.Type.BOOL, True)
        ])

    def tearDown(self):
        self.node.destroy_node()

    def test_simulation_guard_rejects_hardware(self):
        # When use_sim_time is False or allow_simulation is False, must raise RuntimeError
        with self.assertRaises(RuntimeError):
            SimSafetyNode(parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, False),
                Parameter('allow_simulation', Parameter.Type.BOOL, True)
            ])
        with self.assertRaises(RuntimeError):
            SimSafetyNode(parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
                Parameter('allow_simulation', Parameter.Type.BOOL, False)
            ])

    def test_default_values(self):
        # E-stop must start ACTIVE (True)
        self.assertTrue(self.node.estop_active)
        # Battery default 80%
        self.assertAlmostEqual(self.node.battery_percentage, 0.80)
        self.assertFalse(self.node.battery_low)
        # Water default 80%
        self.assertAlmostEqual(self.node.water_percent, 80.0)
        self.assertFalse(self.node.water_low)
        self.assertTrue(self.node.water_sensor_ok)

    def test_estop_release_and_trigger(self):
        req = Trigger.Request()
        resp = Trigger.Response()

        # Release E-stop
        self.node.cb_release_estop(req, resp)
        self.assertFalse(self.node.estop_active)
        self.assertTrue(resp.success)

        # Trigger E-stop
        self.node.cb_trigger_estop(req, resp)
        self.assertTrue(self.node.estop_active)
        self.assertTrue(resp.success)

    def test_battery_control(self):
        req = SetBool.Request()
        resp = SetBool.Response()

        # Set low
        req.data = True
        self.node.cb_set_battery_low(req, resp)
        self.assertTrue(self.node.battery_low)
        self.assertAlmostEqual(self.node.battery_percentage, 0.10)

        # Set normal
        req.data = False
        self.node.cb_set_battery_low(req, resp)
        self.assertFalse(self.node.battery_low)
        self.assertAlmostEqual(self.node.battery_percentage, 0.80)

    def test_water_control(self):
        req = SetBool.Request()
        resp = SetBool.Response()

        # Set low
        req.data = True
        self.node.cb_set_water_low(req, resp)
        self.assertTrue(self.node.water_low)
        self.assertAlmostEqual(self.node.water_percent, 10.0)

        # Set normal
        req.data = False
        self.node.cb_set_water_low(req, resp)
        self.assertFalse(self.node.water_low)
        self.assertAlmostEqual(self.node.water_percent, 80.0)

        # Sensor fault
        req.data = False
        self.node.cb_set_water_sensor_ok(req, resp)
        self.assertFalse(self.node.water_sensor_ok)


if __name__ == '__main__':
    unittest.main()
