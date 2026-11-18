"""Office Delivery Integration v1 - Dedicated ROS 2 Adapter (Thread-Safe with Signals)."""

import json
import math
import os
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from robot_interfaces.msg import NavigationStatus, RobotStatus, WaterStatus
from office_delivery_models import DeliveryExecutionState, SystemDashboardState
from PyQt5.QtCore import QObject, pyqtSignal


class DeliveryRosBridge(QObject):
    """Bridge between ROS 2 Humble callbacks and PyQt5 GUI thread."""

    status_updated = pyqtSignal(object)  # SystemDashboardState
    execution_updated = pyqtSignal(object)  # DeliveryExecutionState
    ros_log_signal = pyqtSignal(str)
    service_result_signal = pyqtSignal(str, bool, str)  # name, success, message

    def __init__(self, ros_node: Node, parent=None):
        super().__init__(parent)
        self.node = ros_node
        self.lock = threading.RLock()
        self.cb_group = ReentrantCallbackGroup()

        # State representations
        self.dashboard_state = SystemDashboardState()
        self.execution_state = DeliveryExecutionState()

        # Quality of Service profiles
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 1. Subscriptions
        self.sub_nav_status = self.node.create_subscription(
            NavigationStatus, "/navigation/status", self._cb_nav_status, 10, callback_group=self.cb_group
        )
        self.sub_authority = self.node.create_subscription(
            String, "/navigation/authority", self._cb_authority, 1, callback_group=self.cb_group
        )
        self.sub_delivery_auth = self.node.create_subscription(
            String, "/office_robot/delivery_authorized", self._cb_delivery_auth, 1, callback_group=self.cb_group
        )
        self.sub_delivery_req = self.node.create_subscription(
            String, "/office_robot/delivery_request", self._cb_delivery_req, 1, callback_group=self.cb_group
        )
        self.sub_safe_stopped = self.node.create_subscription(
            Bool, "/office_robot/safe_stopped", self._cb_safe_stopped, 1, callback_group=self.cb_group
        )
        self.sub_robot_status = self.node.create_subscription(
            RobotStatus, "/robot_status", self._cb_robot_status, 10, callback_group=self.cb_group
        )
        self.sub_estop = self.node.create_subscription(
            Bool, "/emergency_stop", self._cb_estop, 1, callback_group=self.cb_group
        )
        self.sub_battery = self.node.create_subscription(
            BatteryState, "/battery_state", self._cb_battery, 10, callback_group=self.cb_group
        )
        self.sub_water = self.node.create_subscription(
            WaterStatus, "/water/status", self._cb_water, 10, callback_group=self.cb_group
        )
        self.sub_odom = self.node.create_subscription(
            Odometry, "/odom", self._cb_odom, qos_profile_sensor_data, callback_group=self.cb_group
        )

        # 2. Publisher for QR scanning
        self.pub_qr_scan = self.node.create_publisher(String, "/office_robot/qr_scan", 10)

        # 3. Service Clients
        self.cli_trigger_estop = self.node.create_client(Trigger, "/sim_safety/trigger_estop", callback_group=self.cb_group)
        self.cli_release_estop = self.node.create_client(Trigger, "/sim_safety/release_estop", callback_group=self.cb_group)
        self.cli_set_battery_low = self.node.create_client(SetBool, "/sim_safety/set_battery_low", callback_group=self.cb_group)
        self.cli_set_water_low = self.node.create_client(SetBool, "/sim_safety/set_water_low", callback_group=self.cb_group)
        self.cli_set_water_sensor = self.node.create_client(SetBool, "/sim_safety/set_water_sensor_ok", callback_group=self.cb_group)
        self.cli_reconcile = self.node.create_client(Trigger, "/navigation_manager/reconcile", callback_group=self.cb_group)

        # Periodic health & freshness check timer (every 0.5s)
        self.health_timer = self.node.create_timer(0.5, self._check_health_and_emit, callback_group=self.cb_group)

    def destroy(self):
        try:
            if hasattr(self, "health_timer") and self.health_timer is not None:
                self.node.destroy_timer(self.health_timer)
                self.health_timer = None
        except Exception:
            pass

    # Callbacks
    def _cb_nav_status(self, msg: NavigationStatus):
        with self.lock:
            self.dashboard_state.last_nav_status_time = time.monotonic()
            self.execution_state.navigation_state = msg.status
            self.execution_state.distance_remaining = float(msg.distance_remaining)
            self.execution_state.goal_id = msg.current_goal_name

            # Determine leg from goal name
            if "pickup" in msg.current_goal_name.lower():
                self.execution_state.current_leg = "PICKUP"
                if msg.status == "MOVING":
                    self.execution_state.pipeline_stage = "NAVIGATING_PICKUP"
                elif msg.status == "REACHED_GOAL":
                    self.execution_state.pipeline_stage = "WAITING_PICKUP_QR"
            elif msg.current_goal_name != "None" and msg.current_goal_name != "":
                self.execution_state.current_leg = "DELIVERY"
                if msg.status == "MOVING":
                    self.execution_state.pipeline_stage = "NAVIGATING_DELIVERY"
                elif msg.status == "REACHED_GOAL":
                    self.execution_state.pipeline_stage = "WAITING_DELIVERY_QR"

        self.execution_updated.emit(self.execution_state)

    def _cb_authority(self, msg: String):
        with self.lock:
            self.dashboard_state.last_authority_time = time.monotonic()
            try:
                pkt = json.loads(msg.data)
                self.dashboard_state.authority_mode = pkt.get("mode", "UNKNOWN")
                asgn = pkt.get("assignmentId")
                if asgn:
                    self.execution_state.assignment_id = asgn
                boot = pkt.get("bootId")
                if boot:
                    self.execution_state.boot_id = boot
            except Exception:
                self.dashboard_state.authority_mode = "UNKNOWN"

    def _cb_delivery_auth(self, msg: String):
        with self.lock:
            try:
                pkt = json.loads(msg.data)
                self.dashboard_state.delivery_authorized = bool(pkt.get("authorized", False))
            except Exception:
                self.dashboard_state.delivery_authorized = False

    def _cb_delivery_req(self, msg: String):
        with self.lock:
            try:
                pkt = json.loads(msg.data)
                asgn = pkt.get("assignmentId")
                if asgn:
                    self.execution_state.assignment_id = asgn
            except Exception:
                pass

    def _cb_safe_stopped(self, msg: Bool):
        with self.lock:
            self.dashboard_state.last_safe_stopped_time = time.monotonic()
            self.dashboard_state.safe_stopped = bool(msg.data)

    def _cb_robot_status(self, msg: RobotStatus):
        with self.lock:
            self.dashboard_state.last_robot_status_time = time.monotonic()
            self.dashboard_state.battery_percent = float(msg.battery_percent)
            self.dashboard_state.battery_low = bool(msg.battery_low)
            self.dashboard_state.water_liter = float(msg.water_liter)
            self.dashboard_state.water_low = bool(msg.water_low)
            self.dashboard_state.lidar_ok = bool(msg.lidar_ok)
            self.dashboard_state.odom_ok = bool(msg.odom_ok)
            self.dashboard_state.localization_ok = bool(msg.localization_ok)

    def _cb_estop(self, msg: Bool):
        with self.lock:
            self.dashboard_state.estop_active = bool(msg.data)

    def _cb_battery(self, msg: BatteryState):
        with self.lock:
            if math.isfinite(msg.percentage):
                self.dashboard_state.battery_percent = float(msg.percentage)

    def _cb_water(self, msg: WaterStatus):
        with self.lock:
            self.dashboard_state.water_liter = float(msg.water_liter)
            self.dashboard_state.water_low = bool(msg.water_low) or not bool(msg.sensor_ok)

    def _cb_odom(self, msg: Odometry):
        with self.lock:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.execution_state.robot_pose = (float(p.x), float(p.y), float(yaw))

    def _check_health_and_emit(self):
        now = time.monotonic()
        with self.lock:
            # Check node presences in ROS graph
            node_names = self.node.get_node_names()
            self.dashboard_state.gazebo_running = "/ros_gz_bridge" in node_names
            self.dashboard_state.nav2_active = "/bt_navigator" in node_names
            self.dashboard_state.amcl_active = "/amcl" in node_names
            self.dashboard_state.mission_controller_online = "/mission_controller_node" in node_names
            self.dashboard_state.navigation_manager_online = "/navigation_manager_node" in node_names
            self.dashboard_state.gateway_online = "/office_robot_edge_gateway" in node_names

            # Stale checks (TTL 2.5s)
            if now - self.dashboard_state.last_authority_time > 2.5:
                self.dashboard_state.authority_mode = "UNKNOWN"
            if now - self.dashboard_state.last_safe_stopped_time > 2.5:
                self.dashboard_state.safe_stopped = False
            if now - self.dashboard_state.last_robot_status_time > 2.5:
                self.dashboard_state.lidar_ok = False
                self.dashboard_state.odom_ok = False
                self.dashboard_state.localization_ok = False

            # TF validity
            self.dashboard_state.tf_valid = self.dashboard_state.localization_ok and self.dashboard_state.odom_ok

        self.status_updated.emit(self.dashboard_state)

    # Actions: Publish QR Scan
    def publish_qr_scan(self, challenge_id: str, token: str):
        payload = {"challengeId": challenge_id, "token": token}
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_qr_scan.publish(msg)
        self.ros_log_signal.emit(f"Đã gửi thông điệp quét QR tới /office_robot/qr_scan (challengeId={challenge_id})")

    # Safe asynchronous service calls
    def call_service_async(self, client, request, service_name: str, timeout: float = 5.0):
        def _worker():
            try:
                if not client.wait_for_service(timeout_sec=timeout):
                    self.ros_log_signal.emit(f"Dịch vụ {service_name} không khả dụng (timeout={timeout}s)")
                    self.service_result_signal.emit(service_name, False, "Service unavailable")
                    return

                future = client.call_async(request)
                start = time.time()
                while not future.done() and time.time() - start < timeout:
                    time.sleep(0.05)

                if future.done():
                    try:
                        res = future.result()
                        msg = getattr(res, "message", "OK")
                        success = getattr(res, "success", True)
                        self.ros_log_signal.emit(f"Dịch vụ {service_name}: success={success}, msg='{msg}'")
                        self.service_result_signal.emit(service_name, success, msg)
                    except Exception as e:
                        self.ros_log_signal.emit(f"Dịch vụ {service_name} lỗi: {e}")
                        self.service_result_signal.emit(service_name, False, str(e))
                else:
                    self.ros_log_signal.emit(f"Dịch vụ {service_name} timeout sau {timeout}s")
                    self.service_result_signal.emit(service_name, False, "Deadline exceeded")
            except Exception:
                pass

        threading.Thread(target=_worker, daemon=True).start()

    def trigger_estop(self):
        self.call_service_async(self.cli_trigger_estop, Trigger.Request(), "/sim_safety/trigger_estop")

    def release_estop(self):
        self.call_service_async(self.cli_release_estop, Trigger.Request(), "/sim_safety/release_estop")

    def set_battery_low(self, value: bool):
        req = SetBool.Request()
        req.data = bool(value)
        self.call_service_async(self.cli_set_battery_low, req, "/sim_safety/set_battery_low")

    def set_water_low(self, value: bool):
        req = SetBool.Request()
        req.data = bool(value)
        self.call_service_async(self.cli_set_water_low, req, "/sim_safety/set_water_low")

    def set_water_sensor_ok(self, value: bool):
        req = SetBool.Request()
        req.data = bool(value)
        self.call_service_async(self.cli_set_water_sensor, req, "/sim_safety/set_water_sensor_ok")

    def reconcile_nav2(self):
        self.call_service_async(self.cli_reconcile, Trigger.Request(), "/navigation_manager/reconcile")
