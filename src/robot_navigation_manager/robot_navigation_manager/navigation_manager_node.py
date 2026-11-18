"""Single local Nav2 owner for DirectGo and scoped delivery goals."""

import math
import os
import threading
import time
from uuid import UUID

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from action_msgs.msg import GoalStatus, GoalStatusArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from robot_interfaces.action import DeliveryGo, DirectGo
from robot_interfaces.msg import NavigationStatus
from robot_interfaces.srv import GetLocation
from unique_identifier_msgs.msg import UUID as GoalUUID

from robot_edge_gateway.authority import NavigationAuthority
from robot_edge_gateway.navigation_ledger import DeliveryGoalLedger
import json


class NavigationManagerNode(Node):
    def __init__(self):
        super().__init__("navigation_manager_node")
        self.cb_group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.authority = NavigationAuthority()
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.nav2_goal_handle = None
        self.outer_handle = None
        self.cancel_requested = False
        self.current_goal_name, self.status_string, self.distance_remaining = "None", "IDLE", -1.0
        self.declare_parameter("nav2_action", "navigate_to_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("auto_reconcile_on_startup", False)
        self.declare_parameter("operator_reconcile_enabled", True)
        self.declare_parameter("delivery_goal_ledger_path", os.path.expanduser("~/.ros/office_robot_navigation.sqlite3"))
        self.goal_ledger = DeliveryGoalLedger(self.get_parameter("delivery_goal_ledger_path").value, acquire_owner=True)
        action = self.get_parameter("nav2_action").value
        self.nav2_client = ActionClient(self, NavigateToPose, action, callback_group=self.cb_group)
        self.reconcile_service = self.create_service(Trigger, "/navigation_manager/reconcile", self.handle_reconcile_service, callback_group=self.cb_group)
        self.get_loc_client = self.create_client(GetLocation, "/config/get_location", callback_group=self.cb_group)
        self._action_server = ActionServer(self, DirectGo, "/navigation/direct_go", self.execute_callback,
            goal_callback=self.direct_goal, cancel_callback=self.cancel_callback, callback_group=self.cb_group)
        self._delivery_server = ActionServer(self, DeliveryGo, "/navigation/delivery_go", self.delivery_callback,
            goal_callback=self.delivery_goal, cancel_callback=self.cancel_callback, callback_group=self.cb_group)
        self.status_pub = self.create_publisher(NavigationStatus, "/navigation/status", 10)
        self.stopped_pub = self.create_publisher(Bool, "/office_robot/safe_stopped", 1)
        self.authorized_pub = self.create_publisher(String, "/office_robot/delivery_authorized", 1)
        self.create_subscription(String, "/navigation/authority", self.grant, 1, callback_group=self.cb_group)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odometry,
                                 qos_profile_sensor_data, callback_group=self.cb_group)
        self.create_subscription(GoalStatusArray, action.rstrip("/") + "/_action/status", self.nav_status,
                                 qos_profile_action_status_default, callback_group=self.cb_group)
        self.create_timer(0.1, self.watchdog, callback_group=self.cb_group, clock=self.watchdog_clock)

    def grant(self, message):
        try:
            packet = json.loads(message.data)
        except (ValueError, TypeError):
            packet = {}
        with self.lock:
            self.authority.grant(packet)
            if self.authority.needs_cancel():
                self._cancel()

    def odometry(self, message):
        twist = message.twist.twist
        self.authority.odometry([getattr(vector, axis) for vector in (twist.linear, twist.angular) for axis in ("x", "y", "z")])

    def nav_status(self, message):
        active_goals = [
            bytes(item.goal_info.goal_id.uuid).hex()
            for item in message.status_list
            if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
        ]
        with self.lock:
            self.authority.nav_status(len(active_goals) > 0, goal_ids=active_goals)
            if self.authority.needs_cancel():
                self._cancel()

    def handle_reconcile_service(self, request, response):
        with self.lock:
            if not self.get_parameter("operator_reconcile_enabled").value:
                response.success = False
                response.message = "Reconciliation rejected: Operator reconciliation is disabled by parameter."
                return response
            if not self.nav2_client.server_is_ready():
                response.success = False
                response.message = "Reconciliation rejected: Nav2 action server is not ready."
                return response
            if (self.authority.nav_status_known and self.authority.nav_active) or len(self.authority.active_goal_ids) > 0:
                response.success = False
                response.message = f"Reconciliation rejected: Nav2 has active goal(s) ({len(self.authority.active_goal_ids)} active)."
                return response
            success = self.authority.operator_reconcile(clear_uncertain=True, verified_clean=True)
            if success:
                response.success = True
                response.message = "Controlled reconciliation successful: Nav2 verified clean and robot stationary."
                self.get_logger().info("Controlled reconciliation confirmed by operator.")
            else:
                response.success = False
                response.message = "Reconciliation rejected: Robot is not physically stationary or odometry is stale."
            return response

    def watchdog(self):
        ready = self.nav2_client.server_is_ready()
        with self.lock:
            if not ready:
                if self.authority.nav_server_ready:
                    self.authority.server_status(False)
                    self._cancel()
            else:
                if not self.authority.nav_server_ready:
                    self.authority.server_status(True)

            if self.authority.needs_cancel() or not ready:
                self._cancel()

            stopped = self.authority.safe_stopped() and ready
            self.stopped_pub.publish(Bool(data=bool(stopped)))
            packet = self.authority.packet or {}
            scope = (packet.get("assignmentId"), packet.get("bootId"))
            allowed = bool(self.authority.delivery_allowed(scope) and ready and stopped)
            self.authorized_pub.publish(String(data=json.dumps({"assignmentId": scope[0], "bootId": scope[1], "authorized": allowed})))
            status = NavigationStatus()
            status.status, status.current_goal_name, status.distance_remaining = self.status_string, self.current_goal_name, self.distance_remaining
            self.status_pub.publish(status)

    def direct_goal(self, request):
        with self.lock:
            accepted = bool(request.target_location.strip()) and self.nav2_client.server_is_ready() and self.authority.reserve("LOCAL")
            if accepted:
                self.cancel_requested = False
            return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT

    def delivery_goal(self, request):
        try:
            UUID(request.goal_id)
            pose = request.pose.pose
            values = (pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            valid = (request.pose.header.frame_id == self.get_parameter("map_frame").value
                     and all(math.isfinite(value) for value in values)
                     and abs(sum(v*v for v in values[3:]) - 1) < 1e-3)
        except (ValueError, AttributeError):
            valid = False
        with self.lock:
            accepted = (valid and self.nav2_client.server_is_ready() and
                        self.authority.reserve("DELIVERY", (request.assignment_id, request.boot_id), request.goal_id))
            if accepted:
                try:
                    accepted = self.goal_ledger.claim(request.goal_id, {"assignmentId": request.assignment_id,
                        "bootId": request.boot_id, "frameId": request.pose.header.frame_id, "pose": list(values)})
                except Exception:
                    accepted = False
                if not accepted:
                    self.authority.finish()
                else:
                    self.cancel_requested = False
            return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT

    def _cancel(self):
        self.cancel_requested = True
        if self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()

    def cancel_callback(self, goal_handle):
        with self.lock:
            self._cancel()
        return CancelResponse.ACCEPT

    async def _bounded(self, future, timeout):
        """Timeout the waiter, not the action request; retain late goal replies."""
        from rclpy.task import Future
        waiter = Future()

        def done(completed):
            if not waiter.done():
                try:
                    waiter.set_result(completed.result())
                except Exception:
                    waiter.set_exception(RuntimeError("ROS request failed"))

        def expired():
            if not waiter.done():
                waiter.set_exception(TimeoutError("ROS request deadline exceeded"))

        future.add_done_callback(done)
        timer = self.create_timer(timeout, expired, callback_group=self.cb_group, clock=self.watchdog_clock)
        try:
            return await waiter
        finally:
            self.destroy_timer(timer)

    async def execute_callback(self, goal_handle):
        result = DirectGo.Result()
        try:
            if not self.get_loc_client.service_is_ready():
                raise ValueError("Location service unavailable")
            request = GetLocation.Request()
            request.name = goal_handle.request.target_location
            location = await self._bounded(self.get_loc_client.call_async(request), 5.0)
            if location is None or not location.success or not all(math.isfinite(v) for v in (location.x, location.y, location.yaw)):
                raise ValueError("Location unavailable")
            pose = PoseStamped()
            pose.header.frame_id = self.get_parameter("map_frame").value
            pose.pose.position.x, pose.pose.position.y = float(location.x), float(location.y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(location.yaw / 2), math.cos(location.yaw / 2)
            return await self._navigate(goal_handle, pose, result)
        except Exception:
            self.authority.finish()
            result.success, result.finish_message = False, "Location validation failed; no navigation dispatched."
            goal_handle.abort()
            return result

    async def delivery_callback(self, goal_handle):
        return await self._navigate(goal_handle, goal_handle.request.pose, DeliveryGo.Result(), goal_handle.request.goal_id)

    async def _navigate(self, outer, pose, result, goal_id=None):
        sent = False
        try:
            with self.lock:
                if outer.is_cancel_requested or self.cancel_requested or self.authority.needs_cancel():
                    raise ValueError("Local authority revoked")
                self.outer_handle = outer
                self.status_string = "MOVING"
                self.current_goal_name = goal_id or outer.request.target_location
                goal = NavigateToPose.Goal()
                goal.pose = pose
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                kwargs = {"goal_uuid": GoalUUID(uuid=list(UUID(goal_id).bytes))} if goal_id else {}
                future = self.nav2_client.send_goal_async(goal, **kwargs)
                sent = True

            def accepted_late(completed):
                try:
                    handle = completed.result()
                    with self.lock:
                        self.nav2_goal_handle = handle if handle.accepted else None
                        if handle.accepted and (self.cancel_requested or self.authority.needs_cancel()):
                            handle.cancel_goal_async()
                except Exception:
                    pass  # Waiter below retains uncertainty; no new owner is admitted.

            future.add_done_callback(accepted_late)
            handle = await self._bounded(future, 5.0)
            if not handle.accepted:
                sent = False
                raise ValueError("Nav2 rejected goal")
            response = await self._bounded(handle.get_result_async(), 600.0)
            with self.lock:
                success = (response.status == GoalStatus.STATUS_SUCCEEDED and not self.cancel_requested
                           and not outer.is_cancel_requested and not self.authority.needs_cancel())
                self.nav2_goal_handle, self.outer_handle = None, None
                self.authority.finish()
            result.success = bool(success)
            result.finish_message = "Arrived at navigation goal." if success else "Navigation failed or authority was revoked."
            if outer.is_cancel_requested:
                outer.canceled()
            elif success:
                outer.succeed()
            else:
                outer.abort()
            self.status_string = "REACHED_GOAL" if success else "CANCELLED"
        except Exception:
            with self.lock:
                self._cancel()
                self.authority.finish(uncertain=sent)
                self.outer_handle = None
            result.success, result.finish_message = False, "Navigation failed; reconcile uncertain execution before reuse."
            self.status_string = "UNKNOWN" if sent else "FAILED"
            if outer.is_cancel_requested:
                outer.canceled()
            else:
                outer.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        with node.lock:
            node._cancel()
        executor.shutdown()
        node.goal_ledger.close()
        node.destroy_node()
        rclpy.shutdown()
