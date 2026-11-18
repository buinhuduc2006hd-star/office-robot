#!/usr/bin/env python3
"""Automated Test Suite for Office Delivery Integration v1 GUI.

Verifies:
1. Navigation Control Isolation (Fail-safe Nav2 segregation).
2. Direct Goal & Global Cancellation Prevention in Delivery Mode.
3. Fail-closed Motion Safety Gate (Operator Authorization).
4. NLP Command Processing & Deserialization.
5. Delivery Order Management (DRAFT -> CONFIRMED -> QUEUED).
6. QR Code Challenge Issuance and ROS 2 Publication.
7. Complete 12-step Test Case A workflow execution.
"""

import os
import sys
import time
import unittest

os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["ROS_DOMAIN_ID"] = "21"

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMessageBox

# Ensure workspace modules are importable
sys.path.insert(0, "/home/duc/office_robot_ws/src")
sys.path.insert(0, "/home/duc/office_robot_nlp_integration")

from gui_v2 import MainWindow
from office_delivery_models import DeliveryOrder, NLPResult, NLPTaskItem


class TestOfficeDeliveryGUI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not QApplication.instance():
            cls.app = QApplication(sys.argv)
        else:
            cls.app = QApplication.instance()

        # Mock interactive modal dialogs for headless testing
        QMessageBox.question = staticmethod(lambda *args, **kwargs: QMessageBox.Yes)
        QMessageBox.information = staticmethod(lambda *args, **kwargs: None)
        QMessageBox.warning = staticmethod(lambda *args, **kwargs: None)
        QMessageBox.critical = staticmethod(lambda *args, **kwargs: None)

        cls.win = MainWindow()
        cls.tab = cls.win.delivery_tab

    @classmethod
    def tearDownClass(cls):
        try:
            cls.win.close()
            cls.app.processEvents()
        except Exception:
            pass

    def pump_events(self, duration_sec: float = 0.1, step_sec: float = 0.02):
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            self.app.processEvents()
            time.sleep(step_sec)

    def test_01_nav2_control_isolation(self):
        """Test strict Nav2 control isolation in Office Delivery mode vs Legacy mode."""
        # 1. Delivery Mode (Default on startup)
        self.assertEqual(self.win.tab_widget.currentIndex(), 0)
        self.assertTrue(self.win.ros_node.delivery_mode)
        self.assertTrue(self.win.map_view.read_only)
        self.assertIsNone(self.win.ros_node.cmd_vel_pub)
        self.assertIsNone(self.win.ros_node.nav_to_pose_client)
        self.assertIsNone(self.win.ros_node.goal_pub)

        # 2. Switch to Legacy Tab (Manual / SLAM)
        self.win.tab_widget.setCurrentIndex(1)
        self.pump_events(0.1)
        self.assertFalse(self.win.ros_node.delivery_mode)
        self.assertFalse(self.win.map_view.read_only)
        self.assertIsNotNone(self.win.ros_node.cmd_vel_pub)
        self.assertIsNotNone(self.win.ros_node.nav_to_pose_client)
        self.assertIsNotNone(self.win.ros_node.goal_pub)

        # 3. Switch back to Delivery Tab
        self.win.tab_widget.setCurrentIndex(0)
        self.pump_events(0.1)
        self.assertTrue(self.win.ros_node.delivery_mode)
        self.assertTrue(self.win.map_view.read_only)
        self.assertIsNone(self.win.ros_node.cmd_vel_pub)
        self.assertIsNone(self.win.ros_node.nav_to_pose_client)
        self.assertIsNone(self.win.ros_node.goal_pub)

    def test_02_direct_nav_commands_rejected_in_delivery_mode(self):
        """Verify that direct send_goal and cancel_nav2_goals cannot interfere in delivery mode."""
        self.win.tab_widget.setCurrentIndex(0)
        self.pump_events(0.1)

        # Direct send_goal must return False and not execute
        result = self.win.ros_node.send_goal(1.0, 2.0, 0.0)
        self.assertFalse(result)

        # cancel_nav2_goals must log warning and not call global CancelGoal
        self.win.ros_node.cancel_nav2_goals()
        self.assertIsNone(self.win.ros_node.cancel_nav2_client)

    def test_03_fail_closed_motion_safety_gate(self):
        """Verify that motion is fail-closed by default and requires operator toggle."""
        self.assertFalse(self.tab.allow_motion)
        self.assertIn("FAIL-CLOSED", self.tab.lbl_motion_auth.text())

        # Authorize motion via operator toggle
        self.tab._toggle_motion_clicked()
        self.assertTrue(self.tab.allow_motion)
        self.assertIn("OPERATOR AUTHORIZED", self.tab.lbl_motion_auth.text())

        # Toggle back to fail-closed
        self.tab._toggle_motion_clicked()
        self.assertFalse(self.tab.allow_motion)
        self.assertIn("FAIL-CLOSED", self.tab.lbl_motion_auth.text())

    def test_04_nlp_command_handling(self):
        """Verify NLP task creation UI logic and model deserialization."""
        mock_tasks = [
            NLPTaskItem(
                task_id="task_02_mock",
                sequence_index=0,
                document_text="tài liệu",
                destination_text="Phòng 02",
                destination_id="02",
                priority="NORMAL",
                lifecycle="READY",
                handoff="DIRECT",
                document_load="LOADED",
            )
        ]
        mock_result = NLPResult(
            session_id="mock_sess_123",
            message_id="mock_msg_456",
            dialogue_status="TASK_READY",
            reply="Đã nhận lệnh: Giao tài liệu đến phòng 02.",
            tasks=mock_tasks,
        )

        self.tab._on_nlp_result(mock_result)
        self.pump_events(0.1)

        self.assertIsNotNone(self.tab.current_nlp_result)
        self.assertEqual(len(self.tab.current_nlp_result.tasks), 1)
        self.assertIn("TASK_READY", self.tab.lbl_nlp_status.text())
        self.assertIn("phòng 02", self.tab.lbl_nlp_reply.text().lower())

    def test_05_order_lifecycle_management(self):
        """Verify DRAFT creation, CONFIRM transition, and Table updates."""
        # Ensure an NLP result is present
        self.test_04_nlp_command_handling()

        # 1. Provide Draft to verify UI table and state updates
        mock_order = DeliveryOrder(
            request_id="req_test_01",
            task_id="task_02_mock",
            pickup="pickup-a",
            delivery="02",
            priority="NORMAL",
            request_state="DRAFT",
            task_state="READY",
            assignment_id="asgn_test_123",
            last_update="Just now",
            idempotency_key="key_123",
        )
        self.tab._on_draft_result(mock_order)
        self.pump_events(0.1)

        order = self.tab.current_order
        self.assertIsNotNone(order)
        self.assertEqual(order.request_state, "DRAFT")
        self.assertEqual(order.delivery, "02")
        self.assertEqual(self.tab.table_orders.rowCount(), 1)
        self.assertEqual(self.tab.table_orders.item(0, 5).text(), "DRAFT")

        # 2. Confirm Order
        self.tab._on_confirm_result(order.request_id, True, "Confirmed in Staging")
        self.pump_events(0.1)

        self.assertEqual(order.request_state, "CONFIRMED")
        self.assertEqual(self.tab.table_orders.item(0, 5).text(), "CONFIRMED")
        self.assertEqual(self.tab.current_execution.pipeline_stage, "CONFIRMED")

    def test_06_qr_code_issuance_and_scan(self):
        """Verify issuing QR challenge and publishing to /office_robot/qr_scan."""
        # Test QR Pickup
        self.tab._on_qr_issued("PICKUP", "ch_pickup_123", "token_test_p")
        self.pump_events(0.1)
        self.assertIn("TEST QR (PICKUP)", self.tab.lbl_qr_feedback.text())
        self.assertEqual(self.tab.test_a_status["9. CONFIRM QR PICKUP"], "PASS")

        # Test QR Delivery
        self.tab._on_qr_issued("DELIVERY", "ch_delivery_456", "token_test_d")
        self.pump_events(0.1)
        self.assertIn("TEST QR (DELIVERY)", self.tab.lbl_qr_feedback.text())
        self.assertEqual(self.tab.test_a_status["11. CONFIRM QR DELIVERY"], "PASS")
        self.assertEqual(self.tab.test_a_status["12. VERIFY COMPLETED"], "PASS")

    def test_07_complete_test_a_runner_steps(self):
        """Run all 12 steps in Test Case A and verify all reach PASS status."""
        self.tab._reset_test_a()
        self.pump_events(0.1)

        # Step 1: Connect Backend
        self.tab._on_backend_health(True, "HTTP 200 OK")
        # Step 2: Check ROS 2
        self.tab._set_step_status("2. CHECK ROS 2 / NAV2", "PASS")
        # Step 3: Check Safety
        self.tab._set_step_status("3. CHECK SAFETY", "PASS")
        # Step 4: Controlled Reconcile
        self.tab._on_service_result("/navigation_manager/reconcile", True, "Reconciled")
        # Step 5: Create NLP
        self.test_04_nlp_command_handling()
        self.tab._set_step_status("5. CREATE NLP TASK", "PASS")
        # Step 6: Create Draft
        mock_order = DeliveryOrder(
            request_id="req_test_01",
            task_id="task_02_mock",
            pickup="pickup-a",
            delivery="02",
            priority="NORMAL",
            request_state="DRAFT",
            task_state="READY",
            assignment_id="asgn_test_123",
            last_update="Just now",
            idempotency_key="key_123",
        )
        self.tab._on_draft_result(mock_order)
        self.tab._set_step_status("6. CREATE DRAFT", "PASS")
        # Step 7: Confirm Order
        self.tab._on_confirm_result(self.tab.current_order.request_id, True, "OK")
        # Step 8: Monitor Pickup Navigation
        self.tab._advance_test_a_step()
        # Step 9: Confirm QR Pickup
        self.tab._on_qr_issued("PICKUP", "ch_p_test", "tok_p")
        # Step 10: Monitor Delivery Navigation
        self.tab._advance_test_a_step()
        # Step 11: Confirm QR Delivery
        self.tab._on_qr_issued("DELIVERY", "ch_d_test", "tok_d")
        self.pump_events(0.1)

        # Verify all 12 steps reached PASS
        for step, status in self.tab.test_a_status.items():
            self.assertEqual(status, "PASS", f"Step '{step}' did not pass!")

        self.assertEqual(self.tab.current_execution.pipeline_stage, "COMPLETED")
        self.assertIn("COMPLETED", self.tab.lbl_active_stage.text())


if __name__ == "__main__":
    unittest.main()
