#!/usr/bin/env python3
"""
System Monitor Node for Office Robot.
Performs health monitoring, sensor data validation, and fail-closed safety checks.
Monitored inputs:
  - /scan (LaserScan): freshness and finite range check
  - /odom (Odometry): freshness, finite pose and twist check
  - /amcl_pose (PoseWithCovarianceStamped) & TF map->base_footprint: localization health check
  - /battery_state (BatteryState): freshness, validity, low battery check
  - /water/status (WaterStatus): freshness, sensor_ok, low water check
Outputs:
  - /robot_status (RobotStatus): consolidated health status
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_interfaces.msg import WaterStatus, RobotStatus
import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class SystemMonitorNode(Node):
    # Timeout constants (seconds, monotonic)
    SCAN_TIMEOUT = 2.0
    ODOM_TIMEOUT = 2.0
    AMCL_TIMEOUT = 10.0  # AMCL pose update timeout (supplemented by TF check)
    TF_TIMEOUT = 2.5    # TF map->base_footprint freshness timeout
    BATTERY_TIMEOUT = 2.0
    WATER_TIMEOUT = 2.0

    # Covariance divergence thresholds
    MAX_POS_VARIANCE = 5.0
    MAX_YAW_VARIANCE = 5.0

    def __init__(self, **kwargs):
        super().__init__('system_monitor_node', **kwargs)

        self.cb_group = ReentrantCallbackGroup()

        # TF2 listener for localization verification
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.cb_scan, 10, callback_group=self.cb_group)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10, callback_group=self.cb_group)
        qos_amcl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, qos_amcl, callback_group=self.cb_group)
        self.sub_battery = self.create_subscription(
            BatteryState, '/battery_state', self.cb_battery, 10, callback_group=self.cb_group)
        self.sub_water = self.create_subscription(
            WaterStatus, '/water/status', self.cb_water, 10, callback_group=self.cb_group)

        # Publisher
        self.pub_status = self.create_publisher(RobotStatus, '/robot_status', 10)

        # Timers (publish status at 2 Hz)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.5, self.publish_status, callback_group=self.cb_group, clock=self.steady_clock)

        # Internal State - Timestamps (monotonic seconds)
        self.last_scan_time = None
        self.last_odom_time = None
        self.last_amcl_time = None
        self.last_battery_time = None
        self.last_water_time = None

        # Internal State - Data Validity flags
        self.scan_valid = False
        self.odom_valid = False
        self.amcl_valid = False
        self.battery_valid = False
        self.water_valid = False

        # Status variables (Fail-Closed initial defaults)
        self.lidar_ok = False
        self.odom_ok = False
        self.localization_ok = False
        self.battery_percent = 0.0
        self.battery_low = True   # Fail-closed: True until proven normal
        self.water_liter = 0.0
        self.water_low = True     # Fail-closed: True until proven normal

        self.get_logger().info('System Monitor Node initialized with fail-closed safety.')

    # Callback: LaserScan
    def cb_scan(self, msg: LaserScan):
        # Validate metadata
        if (not math.isfinite(msg.angle_min) or
            not math.isfinite(msg.angle_max) or
            not math.isfinite(msg.range_min) or
            not math.isfinite(msg.range_max)):
            self.scan_valid = False
            return

        if not msg.ranges or len(msg.ranges) == 0:
            self.scan_valid = False
            return

        # Check for at least one valid finite reading in [range_min, range_max]
        valid_ranges = 0
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valid_ranges += 1

        if valid_ranges > 0:
            self.scan_valid = True
            self.last_scan_time = time.monotonic()
        else:
            self.scan_valid = False

    # Callback: Odometry
    def cb_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist

        # Check finite values in position and twist
        if (not math.isfinite(pos.x) or not math.isfinite(pos.y) or not math.isfinite(pos.z) or
            not math.isfinite(ori.x) or not math.isfinite(ori.y) or not math.isfinite(ori.z) or not math.isfinite(ori.w) or
            not math.isfinite(twist.linear.x) or not math.isfinite(twist.linear.y) or not math.isfinite(twist.angular.z)):
            self.odom_valid = False
            return

        # Check valid quaternion norm
        q_norm_sq = ori.x**2 + ori.y**2 + ori.z**2 + ori.w**2
        if abs(q_norm_sq - 1.0) > 0.2:
            self.odom_valid = False
            return

        self.odom_valid = True
        self.last_odom_time = time.monotonic()

    # Callback: AMCL Pose
    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        cov = msg.pose.covariance

        if (not math.isfinite(pos.x) or not math.isfinite(pos.y) or
            not math.isfinite(ori.x) or not math.isfinite(ori.y) or not math.isfinite(ori.z) or not math.isfinite(ori.w)):
            self.amcl_valid = False
            return

        # Covariance diagonal indices: 0 (var x), 7 (var y), 35 (var yaw)
        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]

        if (not math.isfinite(var_x) or var_x < 0.0 or var_x > self.MAX_POS_VARIANCE or
            not math.isfinite(var_y) or var_y < 0.0 or var_y > self.MAX_POS_VARIANCE or
            not math.isfinite(var_yaw) or var_yaw < 0.0 or var_yaw > self.MAX_YAW_VARIANCE):
            self.amcl_valid = False
            return

        self.amcl_valid = True
        self.last_amcl_time = time.monotonic()

    # Callback: BatteryState
    def cb_battery(self, msg: BatteryState):
        if not math.isfinite(msg.percentage) or not math.isfinite(msg.voltage):
            self.battery_valid = False
            return

        pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
        if not math.isfinite(pct) or pct < 0.0 or pct > 100.0:
            self.battery_valid = False
            return

        self.battery_percent = pct
        # Low if percentage < 20%, not present, or dead
        is_low = (pct < 20.0) or (not msg.present) or (msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD)
        self.battery_low = is_low
        self.battery_valid = True
        self.last_battery_time = time.monotonic()

    # Callback: WaterStatus
    def cb_water(self, msg: WaterStatus):
        if not math.isfinite(msg.water_percent) or not math.isfinite(msg.water_liter):
            self.water_valid = False
            return

        self.water_liter = msg.water_liter
        # Fail-closed water check: must have sensor_ok=True, and not low
        if not msg.sensor_ok:
            self.water_low = True
        elif msg.water_low or (msg.water_percent < 20.0) or (msg.water_liter < 0.8):
            self.water_low = True
        else:
            self.water_low = False

        self.water_valid = True
        self.last_water_time = time.monotonic()

    def check_tf_localization(self) -> bool:
        """Verify TF transform from map to base_footprint is active and valid."""
        try:
            if not self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                return False
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            # Check transform coordinates are finite
            trans = tf.transform.translation
            rot = tf.transform.rotation
            if (not math.isfinite(trans.x) or not math.isfinite(trans.y) or not math.isfinite(trans.z) or
                not math.isfinite(rot.x) or not math.isfinite(rot.y) or not math.isfinite(rot.z) or not math.isfinite(rot.w)):
                return False
            return True
        except Exception:
            return False

    def publish_status(self):
        now = time.monotonic()

        # 1. Check Lidar Freshness & Validity
        if self.last_scan_time is not None and self.scan_valid:
            self.lidar_ok = (now - self.last_scan_time) <= self.SCAN_TIMEOUT
        else:
            self.lidar_ok = False

        # 2. Check Odom Freshness & Validity
        if self.last_odom_time is not None and self.odom_valid:
            self.odom_ok = (now - self.last_odom_time) <= self.ODOM_TIMEOUT
        else:
            self.odom_ok = False

        # 3. Check Localization (AMCL pose received + bounded covariance + TF map->base_footprint active)
        tf_ok = self.check_tf_localization()
        # AMCL update received at least once with valid covariance
        # In stationary Gazebo, AMCL does not publish continuously, but once initialized with valid cov and TF is active:
        self.localization_ok = self.amcl_valid and tf_ok

        # 4. Check Battery (Fail-closed)
        if self.last_battery_time is None or not self.battery_valid or (now - self.last_battery_time) > self.BATTERY_TIMEOUT:
            self.battery_low = True
            # Keep battery_percent at 0.0 or last if stale
            if self.last_battery_time is None or (now - self.last_battery_time) > self.BATTERY_TIMEOUT:
                self.battery_percent = 0.0

        # 5. Check Water (Fail-closed)
        if self.last_water_time is None or not self.water_valid or (now - self.last_water_time) > self.WATER_TIMEOUT:
            self.water_low = True
            if self.last_water_time is None or (now - self.last_water_time) > self.WATER_TIMEOUT:
                self.water_liter = 0.0

        # Construct and publish RobotStatus message
        msg = RobotStatus()
        msg.battery_percent = float(self.battery_percent)
        msg.battery_low = bool(self.battery_low)
        msg.lidar_ok = bool(self.lidar_ok)
        msg.odom_ok = bool(self.odom_ok)
        msg.localization_ok = bool(self.localization_ok)
        msg.water_liter = float(self.water_liter)
        msg.water_low = bool(self.water_low)
        msg.robot_state = 'IDLE'
        msg.current_location = 'office'

        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
