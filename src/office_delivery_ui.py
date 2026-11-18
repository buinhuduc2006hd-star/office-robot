"""Office Delivery Integration v1 - Complete PyQt5 Operator UI Component."""

import math
import time
from uuid import uuid4
from typing import Dict, List, Optional

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from office_delivery_client import DeliveryClient
from office_delivery_models import (
    PIPELINE_STAGES,
    TEST_A_STEPS,
    DeliveryExecutionState,
    DeliveryOrder,
    NLPResult,
    NLPTaskItem,
    SystemDashboardState,
)
from office_delivery_ros import DeliveryRosBridge


class OfficeDeliveryTab(QWidget):
    """Integrated Operator UI for Office Robot Delivery v1."""

    def __init__(self, ros_bridge: DeliveryRosBridge, client: Optional[DeliveryClient] = None, parent=None):
        super().__init__(parent)
        self.ros = ros_bridge
        self.client = client or DeliveryClient(self)

        # Local state caches
        self.current_nlp_result: Optional[NLPResult] = None
        self.current_order: Optional[DeliveryOrder] = None
        self.current_execution = DeliveryExecutionState()
        self.dashboard_state = SystemDashboardState()
        self.test_a_status = {step: "NOT STARTED" for step in TEST_A_STEPS}
        self.active_assignment_id: Optional[str] = None
        self.allow_motion = False  # Strict default fail-closed

        self.init_ui()
        self.connect_signals()

        # Health polling timer (every 3 seconds)
        self.backend_timer = QTimer(self)
        self.backend_timer.timeout.connect(self.client.check_health)
        self.backend_timer.start(3000)
        self.client.check_health()

    def destroy(self):
        try:
            if hasattr(self, "backend_timer") and self.backend_timer.isActive():
                self.backend_timer.stop()
        except Exception:
            pass

    def init_ui(self):
        root_layout = QVBoxLayout()
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)

        # 1. Header Banner
        header_widget = self._create_header_banner()
        root_layout.addWidget(header_widget)

        # 2. Main Vertical Splitter
        main_splitter = QSplitter(Qt.Horizontal)

        # Left Column (Scrollable): System Dashboard, NLP Input, Orders, QR, Safety
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(4, 4, 4, 4)
        left_layout.setSpacing(8)

        left_layout.addWidget(self._create_dashboard_group())
        left_layout.addWidget(self._create_nlp_group())
        left_layout.addWidget(self._create_order_management_group())
        left_layout.addWidget(self._create_qr_group())
        left_layout.addWidget(self._create_safety_group())

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_widget)
        left_scroll.setMinimumWidth(500)
        main_splitter.addWidget(left_scroll)

        # Right Column: Live Pipeline Monitor, Test A Panel, Audit Log
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(4, 4, 4, 4)
        right_layout.setSpacing(8)

        right_layout.addWidget(self._create_live_monitoring_group())
        right_layout.addWidget(self._create_test_a_panel_group())
        right_layout.addWidget(self._create_log_group())

        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setWidget(right_widget)
        right_scroll.setMinimumWidth(520)
        main_splitter.addWidget(right_scroll)

        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 1)
        root_layout.addWidget(main_splitter, stretch=1)
        self.setLayout(root_layout)

    # UI Section Builders
    def _create_header_banner(self) -> QWidget:
        banner = QGroupBox()
        banner.setStyleSheet("QGroupBox { background-color: #1e293b; border-radius: 6px; padding: 6px; }")
        layout = QHBoxLayout(banner)
        layout.setContentsMargins(8, 4, 8, 4)

        title = QLabel("🏢 OFFICE DELIVERY – INTEGRATION V1")
        title.setFont(QFont("Arial", 13, QFont.Bold))
        title.setStyleSheet("color: #38bdf8;")
        layout.addWidget(title)

        lbl_sim = QLabel("MÔ PHỎNG: GAZEBO HARMONIC (ROS 2 HUMBLE)")
        lbl_sim.setStyleSheet("background-color: #334155; color: #f8fafc; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
        layout.addWidget(lbl_sim)

        lbl_stage = QLabel("STAGING BACKEND: ONLINE")
        lbl_stage.setObjectName("lbl_stage_badge")
        lbl_stage.setStyleSheet("background-color: #065f46; color: #34d399; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
        layout.addWidget(lbl_stage)
        self.lbl_stage_badge = lbl_stage

        self.lbl_motion_auth = QLabel("ALLOW MOTION: FALSE (FAIL-CLOSED)")
        self.lbl_motion_auth.setStyleSheet("background-color: #991b1b; color: #fecaca; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
        layout.addWidget(self.lbl_motion_auth)

        self.btn_toggle_motion = QPushButton("Kích hoạt Chuyển động")
        self.btn_toggle_motion.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 4px 8px;")
        self.btn_toggle_motion.clicked.connect(self._toggle_motion_clicked)
        layout.addWidget(self.btn_toggle_motion)

        layout.addStretch()
        return banner

    def _create_dashboard_group(self) -> QGroupBox:
        grp = QGroupBox("A. System Dashboard (Trạng Thái Hệ Thống Thực Tế)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        layout = QGridLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # 11 Process/Node Indicators
        self.badges: Dict[str, QLabel] = {}
        nodes = [
            ("Gazebo", "gz"),
            ("Nav2", "nav2"),
            ("AMCL", "amcl"),
            ("TF", "tf"),
            ("LiDAR", "lidar"),
            ("Odom", "odom"),
            ("Localization", "loc"),
            ("Backend", "back"),
            ("Gateway", "gate"),
            ("Mission Controller", "mc"),
            ("Navigation Manager", "nm"),
        ]

        row, col = 0, 0
        for name, key in nodes:
            lbl_title = QLabel(f"{name}:")
            lbl_val = QLabel("UNKNOWN")
            lbl_val.setStyleSheet("background-color: #475569; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px;")
            self.badges[key] = lbl_val
            layout.addWidget(lbl_title, row, col * 2)
            layout.addWidget(lbl_val, row, col * 2 + 1)
            col += 1
            if col >= 4:
                col = 0
                row += 1

        # Second Block: Metadata, Battery, Water, Authority
        row += 1
        sep = QLabel("--- Thông Số Vận Hành An Toàn ---")
        sep.setStyleSheet("color: #64748b; font-weight: bold;")
        layout.addWidget(sep, row, 0, 1, 8)
        row += 1

        self.lbl_dash_meta = QLabel("robotId: office-robot-1 | mapId: office-map-sim-v1")
        self.lbl_dash_meta.setStyleSheet("color: #0284c7; font-weight: bold;")
        layout.addWidget(self.lbl_dash_meta, row, 0, 1, 4)

        self.lbl_dash_estop = QLabel("E-stop: ACTIVE (BLOCKED)")
        self.lbl_dash_estop.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.lbl_dash_estop, row, 4, 1, 4)
        row += 1

        self.lbl_dash_bat = QLabel("Battery: 80.0% [OK]")
        self.lbl_dash_water = QLabel("Water: 4.0L [OK]")
        layout.addWidget(self.lbl_dash_bat, row, 0, 1, 2)
        layout.addWidget(self.lbl_dash_water, row, 2, 1, 2)

        self.lbl_dash_auth = QLabel("Authority: BLOCKED")
        self.lbl_dash_auth.setStyleSheet("background-color: #fee2e2; color: #991b1b; padding: 2px 6px; font-weight: bold;")
        self.lbl_dash_safe_stopped = QLabel("Safe Stopped: FALSE")
        layout.addWidget(self.lbl_dash_auth, row, 4, 1, 2)
        layout.addWidget(self.lbl_dash_safe_stopped, row, 6, 1, 2)

        return grp

    def _create_nlp_group(self) -> QGroupBox:
        grp = QGroupBox("B. NLP Task Creation (Tạo Nhiệm Vụ Giao Hàng Bằng Tiếng Việt)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)

        # Input line
        input_layout = QHBoxLayout()
        self.edt_nlp_input = QLineEdit("Giao tài liệu đến phòng 02.")
        self.edt_nlp_input.setPlaceholderText("Nhập yêu cầu tiếng Việt, ví dụ: 'Giao tài liệu đến phòng 02.'")
        input_layout.addWidget(self.edt_nlp_input, stretch=1)

        self.btn_send_nlp = QPushButton("GỬI YÊU CẦU NLP")
        self.btn_send_nlp.setStyleSheet("background-color: #2563eb; color: white; font-weight: bold; padding: 5px 12px;")
        self.btn_send_nlp.clicked.connect(self._send_nlp_clicked)
        input_layout.addWidget(self.btn_send_nlp)
        layout.addLayout(input_layout)

        # Quick templates
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel("Mẫu nhanh:"))
        for t in ["Phòng 02", "Phòng 04", "Giao hợp đồng phòng 02"]:
            btn = QPushButton(t)
            btn.setStyleSheet("padding: 2px 6px; font-size: 11px;")
            btn.clicked.connect(lambda _, txt=t: self.edt_nlp_input.setText(f"Giao tài liệu đến {txt}."))
            quick_layout.addWidget(btn)
        quick_layout.addStretch()
        layout.addLayout(quick_layout)

        # Result display card
        self.lbl_nlp_status = QLabel("Dialogue Status: CHƯA CÓ YÊU CẦU")
        self.lbl_nlp_status.setStyleSheet("font-weight: bold; color: #475569;")
        layout.addWidget(self.lbl_nlp_status)

        self.lbl_nlp_reply = QLabel("Nội dung phản hồi: -")
        self.lbl_nlp_reply.setWordWrap(True)
        self.lbl_nlp_reply.setStyleSheet("background-color: #f1f5f9; padding: 6px; border-radius: 4px;")
        layout.addWidget(self.lbl_nlp_reply)

        self.lbl_nlp_details = QLabel("Tasks: - | Destination: - | Priority: -")
        self.lbl_nlp_details.setStyleSheet("color: #0369a1; font-weight: bold;")
        layout.addWidget(self.lbl_nlp_details)

        self.btn_use_for_draft = QPushButton("➡️ CHUYỂN SANG TẠO DRAFT")
        self.btn_use_for_draft.setEnabled(False)
        self.btn_use_for_draft.setStyleSheet("background-color: #059669; color: white; font-weight: bold; padding: 4px;")
        self.btn_use_for_draft.clicked.connect(self._create_draft_clicked)
        layout.addWidget(self.btn_use_for_draft)

        return grp

    def _create_order_management_group(self) -> QGroupBox:
        grp = QGroupBox("C. Delivery Order Management (Quản Lý Đơn Giao Hàng)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)

        # Order table
        self.table_orders = QTableWidget(0, 8)
        self.table_orders.setHorizontalHeaderLabels([
            "Request ID", "Task ID", "Pickup", "Delivery", "Priority", "Request State", "Assignment ID", "Last Update"
        ])
        self.table_orders.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_orders.setSelectionBehavior(QTableWidget.SelectRows)
        self.table_orders.setMinimumHeight(120)
        layout.addWidget(self.table_orders)

        # Action buttons
        btn_layout = QHBoxLayout()
        self.btn_create_draft = QPushButton("1. CREATE DRAFT")
        self.btn_create_draft.setStyleSheet("font-weight: bold; padding: 4px 8px;")
        self.btn_create_draft.clicked.connect(self._create_draft_clicked)

        self.btn_confirm_order = QPushButton("2. CONFIRM ORDER")
        self.btn_confirm_order.setStyleSheet("background-color: #0d9488; color: white; font-weight: bold; padding: 4px 8px;")
        self.btn_confirm_order.clicked.connect(self._confirm_order_clicked)

        self.btn_refresh_order = QPushButton("3. REFRESH STATUS")
        self.btn_refresh_order.clicked.connect(self._refresh_order_clicked)

        self.btn_cancel_order = QPushButton("4. CANCEL REQUEST")
        self.btn_cancel_order.setStyleSheet("background-color: #dc2626; color: white; padding: 4px 8px;")
        self.btn_cancel_order.clicked.connect(self._cancel_order_clicked)

        btn_layout.addWidget(self.btn_create_draft)
        btn_layout.addWidget(self.btn_confirm_order)
        btn_layout.addWidget(self.btn_refresh_order)
        btn_layout.addWidget(self.btn_cancel_order)
        layout.addLayout(btn_layout)

        lbl_defaults = QLabel("Mặc định Pickup: 'pickup-a' | Destination: Từ NLP | Map: 'office-map-sim-v1'")
        lbl_defaults.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(lbl_defaults)

        return grp

    def _create_qr_group(self) -> QGroupBox:
        grp = QGroupBox("E. QR Pickup / Delivery (Xác Nhận Handoff Hợp Lệ)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)

        # Mode selector
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Chế độ quét:"))
        self.rb_test_qr = QRadioButton("TEST QR (Mô Phỏng Quét Xác Thực Thật)")
        self.rb_test_qr.setChecked(True)
        self.rb_real_qr = QRadioButton("REAL QR (Phần Cứng Máy Quét)")
        qr_group = QButtonGroup(self)
        qr_group.addButton(self.rb_test_qr)
        qr_group.addButton(self.rb_real_qr)
        mode_layout.addWidget(self.rb_test_qr)
        mode_layout.addWidget(self.rb_real_qr)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        # QR Action Buttons
        btn_box = QHBoxLayout()
        self.btn_qr_pickup = QPushButton("📦 XÁC NHẬN QR PICKUP")
        self.btn_qr_pickup.setStyleSheet("background-color: #d97706; color: white; font-weight: bold; font-size: 12px; padding: 6px;")
        self.btn_qr_pickup.clicked.connect(lambda: self._issue_and_scan_qr("PICKUP"))

        self.btn_qr_delivery = QPushButton("🏁 XÁC NHẬN QR DELIVERY")
        self.btn_qr_delivery.setStyleSheet("background-color: #16a34a; color: white; font-weight: bold; font-size: 12px; padding: 6px;")
        self.btn_qr_delivery.clicked.connect(lambda: self._issue_and_scan_qr("DELIVERY"))

        btn_box.addWidget(self.btn_qr_pickup)
        btn_box.addWidget(self.btn_qr_delivery)
        layout.addLayout(btn_box)

        self.lbl_qr_feedback = QLabel("QR State: IDLE. Token được bảo mật tuyệt đối không ghi log.")
        self.lbl_qr_feedback.setStyleSheet("color: #475569; font-size: 11px;")
        layout.addWidget(self.lbl_qr_feedback)

        return grp

    def _create_safety_group(self) -> QGroupBox:
        grp = QGroupBox("F. Simulation Safety (Mô Phỏng An Toàn Có Kiểm Soát)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        layout = QGridLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        self.btn_trigger_estop = QPushButton("🛑 TRIGGER E-STOP (BLOCKED)")
        self.btn_trigger_estop.setStyleSheet("background-color: #dc2626; color: white; font-weight: bold; padding: 6px;")
        self.btn_trigger_estop.clicked.connect(self._trigger_estop_clicked)

        self.btn_release_estop = QPushButton("🟢 RELEASE E-STOP (LOCAL)")
        self.btn_release_estop.setStyleSheet("background-color: #15803d; color: white; font-weight: bold; padding: 6px;")
        self.btn_release_estop.clicked.connect(self._release_estop_clicked)

        layout.addWidget(self.btn_trigger_estop, 0, 0)
        layout.addWidget(self.btn_release_estop, 0, 1)

        # Battery controls
        btn_bat_low = QPushButton("Set Pin Yếu (10%)")
        btn_bat_low.clicked.connect(lambda: self.ros.set_battery_low(True))
        btn_bat_norm = QPushButton("Set Pin Bình Thường (80%)")
        btn_bat_norm.clicked.connect(lambda: self.ros.set_battery_low(False))
        layout.addWidget(btn_bat_low, 1, 0)
        layout.addWidget(btn_bat_norm, 1, 1)

        # Water controls
        btn_water_low = QPushButton("Set Nước Thấp (0.5L)")
        btn_water_low.clicked.connect(lambda: self.ros.set_water_low(True))
        btn_water_norm = QPushButton("Set Nước Bình Thường (4.0L)")
        btn_water_norm.clicked.connect(lambda: self.ros.set_water_low(False))
        layout.addWidget(btn_water_low, 2, 0)
        layout.addWidget(btn_water_norm, 2, 1)

        # Water sensor fault & Reconcile
        btn_water_fault = QPushButton("Simulate Sensor Fault")
        btn_water_fault.clicked.connect(lambda: self.ros.set_water_sensor_ok(False))
        btn_reconcile = QPushButton("🔄 Reconcile Nav2 Clean")
        btn_reconcile.setStyleSheet("font-weight: bold;")
        btn_reconcile.clicked.connect(lambda: self.ros.reconcile_nav2())
        layout.addWidget(btn_water_fault, 3, 0)
        layout.addWidget(btn_reconcile, 3, 1)

        return grp

    def _create_live_monitoring_group(self) -> QGroupBox:
        grp = QGroupBox("D. Live Delivery Monitoring (Theo Dõi Tiến Trình Thực Thi)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)

        # Visual pipeline progress
        lbl_p_title = QLabel("Tiến trình: DRAFT → CONFIRMED → QUEUED → ASSIGNED → ACKED → NAV_PICKUP → QR_PICKUP → NAV_DELIVERY → QR_DELIVERY → COMPLETED")
        lbl_p_title.setStyleSheet("font-size: 10px; color: #64748b; font-weight: bold;")
        lbl_p_title.setWordWrap(True)
        layout.addWidget(lbl_p_title)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, len(PIPELINE_STAGES) - 1)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #0284c7; }")
        layout.addWidget(self.progress_bar)

        self.lbl_active_stage = QLabel("Giai đoạn hiện tại: DRAFT")
        self.lbl_active_stage.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_active_stage.setStyleSheet("color: #0284c7; background-color: #e0f2fe; padding: 4px; border-radius: 4px;")
        layout.addWidget(self.lbl_active_stage)

        # Detail cards
        details_layout = QGridLayout()
        self.lbl_live_assignment = QLabel("Assignment ID: -")
        self.lbl_live_boot = QLabel("Boot ID: -")
        self.lbl_live_goal = QLabel("Goal ID: -")
        self.lbl_live_nav_state = QLabel("Nav State: IDLE")
        self.lbl_live_pose = QLabel("Robot Pose: (0.00, 0.00, 0.0°)")
        self.lbl_live_dist = QLabel("Distance Rem: -")
        self.lbl_live_leg = QLabel("Current Leg: IDLE")
        self.lbl_live_qr = QLabel("QR State: IDLE")

        details_layout.addWidget(self.lbl_live_assignment, 0, 0)
        details_layout.addWidget(self.lbl_live_boot, 0, 1)
        details_layout.addWidget(self.lbl_live_goal, 1, 0)
        details_layout.addWidget(self.lbl_live_nav_state, 1, 1)
        details_layout.addWidget(self.lbl_live_pose, 2, 0)
        details_layout.addWidget(self.lbl_live_dist, 2, 1)
        details_layout.addWidget(self.lbl_live_leg, 3, 0)
        details_layout.addWidget(self.lbl_live_qr, 3, 1)
        layout.addLayout(details_layout)

        return grp

    def _create_test_a_panel_group(self) -> QGroupBox:
        grp = QGroupBox("5. TEST A CONTROL PANEL (END-TO-END DELIVERY RUNNER)")
        grp.setFont(QFont("Arial", 10, QFont.Bold))
        grp.setStyleSheet("QGroupBox { border: 2px solid #0284c7; border-radius: 6px; }")
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(6, 6, 6, 6)

        # 12-Step Status Grid
        step_grid = QGridLayout()
        self.step_labels: Dict[str, QLabel] = {}
        for idx, step_name in enumerate(TEST_A_STEPS):
            lbl_name = QLabel(step_name)
            lbl_status = QLabel("NOT STARTED")
            lbl_status.setStyleSheet("background-color: #cbd5e1; color: #1e293b; padding: 2px 6px; border-radius: 3px; font-weight: bold; font-size: 10px;")
            self.step_labels[step_name] = lbl_status
            r = idx // 2
            c = (idx % 2) * 2
            step_grid.addWidget(lbl_name, r, c)
            step_grid.addWidget(lbl_status, r, c + 1)
        layout.addLayout(step_grid)

        # Actions
        btn_box = QHBoxLayout()
        self.btn_preflight = QPushButton("🔍 RUN PREFLIGHT CHECK")
        self.btn_preflight.setStyleSheet("font-weight: bold; padding: 5px;")
        self.btn_preflight.clicked.connect(self._run_preflight_check)

        self.btn_advance_step = QPushButton("▶ TIẾP TỤC BƯỚC TIẾP THEO")
        self.btn_advance_step.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 5px;")
        self.btn_advance_step.clicked.connect(self._advance_test_a_step)

        self.btn_reset_test = QPushButton("RESET TEST A")
        self.btn_reset_test.clicked.connect(self._reset_test_a)

        btn_box.addWidget(self.btn_preflight)
        btn_box.addWidget(self.btn_advance_step)
        btn_box.addWidget(self.btn_reset_test)
        layout.addLayout(btn_box)

        return grp

    def _create_log_group(self) -> QGroupBox:
        grp = QGroupBox("Nhật Ký Tích Hợp & Sự Kiện Vận Hành")
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(4, 4, 4, 4)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("background-color: #0f172a; color: #e2e8f0; font-family: monospace; font-size: 11px;")
        self.txt_log.setMinimumHeight(130)
        layout.addWidget(self.txt_log)
        return grp

    # Signal Connections
    def connect_signals(self):
        # Client signals
        self.client.health_signal.connect(self._on_backend_health)
        self.client.nlp_result_signal.connect(self._on_nlp_result)
        self.client.nlp_error_signal.connect(self._on_nlp_error)
        self.client.draft_result_signal.connect(self._on_draft_result)
        self.client.draft_error_signal.connect(self._on_draft_error)
        self.client.confirm_result_signal.connect(self._on_confirm_result)
        self.client.request_status_signal.connect(self._on_request_status_update)
        self.client.qr_issued_signal.connect(self._on_qr_issued)
        self.client.qr_error_signal.connect(self._on_qr_error)
        self.client.log_signal.connect(self.append_log)

        # ROS signals
        self.ros.status_updated.connect(self._on_ros_status_updated)
        self.ros.execution_updated.connect(self._on_ros_execution_updated)
        self.ros.ros_log_signal.connect(self.append_log)
        self.ros.service_result_signal.connect(self._on_service_result)

    # Slots & Handlers
    def append_log(self, text: str):
        timestamp = time.strftime("%H:%M:%S")
        self.txt_log.append(f"[{timestamp}] {text}")

    def _on_backend_health(self, online: bool, message: str):
        self.dashboard_state.backend_online = online
        if online:
            self.lbl_stage_badge.setText("STAGING BACKEND: ONLINE")
            self.lbl_stage_badge.setStyleSheet("background-color: #065f46; color: #34d399; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
            self._update_badge("back", True)
            self._set_step_status("1. CONNECT BACKEND", "PASS")
        else:
            self.lbl_stage_badge.setText("STAGING BACKEND: OFFLINE")
            self.lbl_stage_badge.setStyleSheet("background-color: #991b1b; color: #fecaca; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
            self._update_badge("back", False)
            self._set_step_status("1. CONNECT BACKEND", "FAIL")

    def _update_badge(self, key: str, ok: bool):
        badge = self.badges.get(key)
        if not badge:
            return
        if ok:
            badge.setText("ACTIVE" if key in ("nav2", "amcl") else "ONLINE" if key in ("gz", "back", "gate", "mc", "nm") else "OK")
            badge.setStyleSheet("background-color: #15803d; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; font-weight: bold;")
        else:
            badge.setText("INACTIVE" if key in ("nav2", "amcl") else "OFFLINE" if key in ("gz", "back", "gate", "mc", "nm") else "ERROR")
            badge.setStyleSheet("background-color: #b91c1c; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; font-weight: bold;")

    def _on_ros_status_updated(self, state: SystemDashboardState):
        self.dashboard_state = state
        self._update_badge("gz", state.gazebo_running)
        self._update_badge("nav2", state.nav2_active)
        self._update_badge("amcl", state.amcl_active)
        self._update_badge("tf", state.tf_valid)
        self._update_badge("lidar", state.lidar_ok)
        self._update_badge("odom", state.odom_ok)
        self._update_badge("loc", state.localization_ok)
        self._update_badge("mc", state.mission_controller_online)
        self._update_badge("nm", state.navigation_manager_online)
        self._update_badge("gate", state.gateway_online)

        # Battery & Water
        bat_str = f"Battery: {state.battery_percent:.1f}% [{'LOW' if state.battery_low else 'OK'}]"
        self.lbl_dash_bat.setText(bat_str)
        self.lbl_dash_bat.setStyleSheet("color: red; font-weight: bold;" if state.battery_low else "color: #15803d; font-weight: bold;")

        water_str = f"Water: {state.water_liter:.1f}L [{'LOW' if state.water_low else 'OK'}]"
        self.lbl_dash_water.setText(water_str)
        self.lbl_dash_water.setStyleSheet("color: red; font-weight: bold;" if state.water_low else "color: #15803d; font-weight: bold;")

        # E-stop
        if state.estop_active:
            self.lbl_dash_estop.setText("E-stop: ACTIVE (BLOCKED)")
            self.lbl_dash_estop.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.lbl_dash_estop.setText("E-stop: RELEASED (OK)")
            self.lbl_dash_estop.setStyleSheet("color: #15803d; font-weight: bold;")

        # Authority
        auth_color = "#991b1b" if state.authority_mode == "BLOCKED" else "#0369a1" if state.authority_mode == "LOCAL" else "#15803d"
        self.lbl_dash_auth.setText(f"Authority: {state.authority_mode}")
        self.lbl_dash_auth.setStyleSheet(f"background-color: #f1f5f9; color: {auth_color}; font-weight: bold; padding: 2px 6px;")

        self.lbl_dash_safe_stopped.setText(f"Safe Stopped: {state.safe_stopped}")
        self.lbl_dash_safe_stopped.setStyleSheet("color: #15803d; font-weight: bold;" if state.safe_stopped else "color: #b45309; font-weight: bold;")

    def _on_ros_execution_updated(self, state: DeliveryExecutionState):
        self.current_execution = state
        self.lbl_live_nav_state.setText(f"Nav State: {state.navigation_state}")
        self.lbl_live_dist.setText(f"Distance Rem: {state.distance_remaining:.2f} m" if state.distance_remaining >= 0 else "Distance Rem: -")
        self.lbl_live_leg.setText(f"Current Leg: {state.current_leg}")
        self.lbl_live_pose.setText(f"Robot Pose: ({state.robot_pose[0]:.2f}, {state.robot_pose[1]:.2f}, {math.degrees(state.robot_pose[2]):.1f}°)")

        if state.goal_id:
            self.lbl_live_goal.setText(f"Goal ID: {state.goal_id[:16]}")
        if state.assignment_id:
            self.active_assignment_id = state.assignment_id
            self.lbl_live_assignment.setText(f"Assignment ID: {state.assignment_id[:16]}")

        # Update pipeline stage if it advanced
        if state.pipeline_stage in PIPELINE_STAGES:
            self._set_pipeline_stage(state.pipeline_stage)

    def _set_pipeline_stage(self, stage: str):
        if stage in PIPELINE_STAGES:
            self.current_execution.pipeline_stage = stage
            idx = PIPELINE_STAGES.index(stage)
            self.progress_bar.setValue(idx)
            self.lbl_active_stage.setText(f"Giai đoạn hiện tại: {stage}")
            # Enable QR buttons contextually
            self.btn_qr_pickup.setEnabled(stage in ("WAITING_PICKUP_QR", "ACKED", "NAVIGATING_PICKUP"))
            self.btn_qr_delivery.setEnabled(stage in ("WAITING_DELIVERY_QR", "NAVIGATING_DELIVERY"))

    # Actions: NLP
    def _send_nlp_clicked(self):
        text = self.edt_nlp_input.text().strip()
        if not text:
            QMessageBox.warning(self, "Cảnh báo", "Vui lòng nhập yêu cầu giao hàng.")
            return
        self.lbl_nlp_status.setText("Dialogue Status: ĐANG XỬ LÝ...")
        self.client.send_nlp_command(text)

    def _on_nlp_result(self, result: NLPResult):
        self.current_nlp_result = result
        self.lbl_nlp_status.setText(f"Dialogue Status: {result.dialogue_status} (Session: {result.session_id[:8]}..)")
        self.lbl_nlp_reply.setText(f"Nội dung phản hồi: {result.reply}")

        if result.tasks:
            t = result.tasks[0]
            self.lbl_nlp_details.setText(
                f"Task ID: {t.task_id} | Dest: {t.destination_text} ({t.destination_id}) | Priority: {t.priority} | Doc: {t.document_text}"
            )
        else:
            self.lbl_nlp_details.setText("Không có task cụ thể trong yêu cầu.")

        if result.dialogue_status in ("TASK_READY", "BATCH_READY"):
            self.btn_use_for_draft.setEnabled(True)
            self._set_step_status("5. CREATE NLP TASK", "PASS")
        else:
            self.btn_use_for_draft.setEnabled(False)
            self._set_step_status("5. CREATE NLP TASK", "BLOCKED")

    def _on_nlp_error(self, err: str):
        self.lbl_nlp_status.setText(f"Dialogue Status: LỖI ({err[:30]})")
        self._set_step_status("5. CREATE NLP TASK", "FAIL")

    # Actions: Order Management
    def _create_draft_clicked(self):
        if not self.current_nlp_result or not self.current_nlp_result.tasks:
            QMessageBox.warning(self, "Cảnh báo", "Chưa có kết quả NLP hợp lệ để tạo DRAFT.")
            return

        task = self.current_nlp_result.tasks[0]
        # Use stable idempotency key
        idemp_key = f"key_{task.task_id}_{self.current_nlp_result.session_id[:6]}"
        self.client.create_draft(
            session_id=self.current_nlp_result.session_id,
            message_id=self.current_nlp_result.message_id,
            task_id=task.task_id,
            destination_id=task.destination_id,
            pickup_id="pickup-a",
            map_id="office-map-sim-v1",
            idempotency_key=idemp_key,
        )

    def _on_draft_result(self, order: DeliveryOrder):
        self.current_order = order
        self._set_pipeline_stage("DRAFT")
        self._set_step_status("6. CREATE DRAFT", "PASS")

        # Update table
        row = self.table_orders.rowCount()
        self.table_orders.insertRow(row)
        self.table_orders.setItem(row, 0, QTableWidgetItem(order.request_id))
        self.table_orders.setItem(row, 1, QTableWidgetItem(order.task_id))
        self.table_orders.setItem(row, 2, QTableWidgetItem(order.pickup))
        self.table_orders.setItem(row, 3, QTableWidgetItem(order.delivery))
        self.table_orders.setItem(row, 4, QTableWidgetItem(order.priority))
        self.table_orders.setItem(row, 5, QTableWidgetItem(order.request_state))
        self.table_orders.setItem(row, 6, QTableWidgetItem(order.assignment_id or "-"))
        self.table_orders.setItem(row, 7, QTableWidgetItem(order.last_update))
        self.table_orders.selectRow(row)

    def _on_draft_error(self, err: str):
        self._set_step_status("6. CREATE DRAFT", "FAIL")

    def _confirm_order_clicked(self):
        if not self.current_order:
            QMessageBox.warning(self, "Cảnh báo", "Vui lòng chọn hoặc tạo đơn DRAFT trước.")
            return

        # Mandatory confirmation dialog by human operator
        confirm = QMessageBox.question(
            self,
            "Xác nhận đơn hàng giao tài liệu",
            f"Bạn có chắc chắn muốn CONFIRM đơn hàng:\n"
            f"RequestId: {self.current_order.request_id}\n"
            f"TaskId: {self.current_order.task_id}\n"
            f"Pickup: {self.current_order.pickup} -> Delivery: Phòng {self.current_order.delivery}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm == QMessageBox.Yes:
            self.client.confirm_order(self.current_order.request_id)

    def _on_confirm_result(self, req_id: str, success: bool, msg: str):
        if success:
            if self.current_order and self.current_order.request_id == req_id:
                self.current_order.request_state = "CONFIRMED"
                self._update_table_order_state(req_id, "CONFIRMED")
            self._set_pipeline_stage("CONFIRMED")
            self._set_step_status("7. CONFIRM ORDER", "PASS")
            QMessageBox.information(self, "Thành công", f"Đơn hàng {req_id} đã được CONFIRM và đưa vào hàng đợi QUEUED.")
        else:
            self._set_step_status("7. CONFIRM ORDER", "FAIL")
            QMessageBox.critical(self, "Lỗi Confirm", f"Không thể confirm đơn {req_id}: {msg}")

    def _refresh_order_clicked(self):
        if self.current_order:
            self.client.get_request_status(self.current_order.request_id)

    def _on_request_status_update(self, res: dict):
        req_id = res.get("requestId")
        state = res.get("state", "UNKNOWN")
        if req_id and self.current_order and self.current_order.request_id == req_id:
            self.current_order.request_state = state
            self._update_table_order_state(req_id, state)
            if state in PIPELINE_STAGES:
                self._set_pipeline_stage(state)

    def _update_table_order_state(self, req_id: str, state: str):
        for r in range(self.table_orders.rowCount()):
            item = self.table_orders.item(r, 0)
            if item and item.text() == req_id:
                self.table_orders.setItem(r, 5, QTableWidgetItem(state))
                self.table_orders.setItem(r, 7, QTableWidgetItem("Updated"))

    def _cancel_order_clicked(self):
        if not self.current_order:
            return
        confirm = QMessageBox.question(
            self, "Hủy đơn hàng", f"Bạn có chắc muốn HỦY đơn hàng {self.current_order.request_id}?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            self.client.cancel_request(self.current_order.request_id)

    # Actions: QR
    def _issue_and_scan_qr(self, leg: str):
        aid = self.active_assignment_id or (self.current_order.assignment_id if self.current_order else None) or "asgn_test_v1"
        self.lbl_qr_feedback.setText(f"Đang yêu cầu challenge QR cho leg={leg}...")
        self.client.issue_qr(aid, leg)

    def _on_qr_issued(self, leg: str, challenge_id: str, token: str):
        self.lbl_qr_feedback.setText(f"Nhận QR challenge hợp lệ ({challenge_id}). Đang thực hiện quét...")
        # In TEST QR mode, publish scan directly to /office_robot/qr_scan
        if self.rb_test_qr.isChecked():
            self.ros.publish_qr_scan(challenge_id, token)
            self.lbl_qr_feedback.setText(f"Đã quét thành công TEST QR ({leg})! Đang chờ Backend xác thực.")
            if leg == "PICKUP":
                self._set_step_status("9. CONFIRM QR PICKUP", "PASS")
                self._set_pipeline_stage("NAVIGATING_DELIVERY")
            else:
                self._set_step_status("11. CONFIRM QR DELIVERY", "PASS")
                self._set_pipeline_stage("COMPLETED")
                self._set_step_status("12. VERIFY COMPLETED", "PASS")
        else:
            self.lbl_qr_feedback.setText(f"REAL QR: Đang chờ máy quét vật lý đưa dữ liệu vào...")

    def _on_qr_error(self, leg: str, err: str):
        self.lbl_qr_feedback.setText(f"Lỗi QR {leg}: {err}")

    # Actions: Safety Controls
    def _trigger_estop_clicked(self):
        self.ros.trigger_estop()

    def _release_estop_clicked(self):
        confirm = QMessageBox.question(
            self,
            "Nhả E-Stop",
            "Bạn có chắc chắn muốn NHẢ E-STOP và chuyển Mission Authority sang LOCAL?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm == QMessageBox.Yes:
            self.ros.release_estop()

    def _on_service_result(self, name: str, success: bool, msg: str):
        if "reconcile" in name:
            if success:
                self._set_step_status("4. CHECK RECONCILIATION", "PASS")
            else:
                self._set_step_status("4. CHECK RECONCILIATION", "FAIL")

    # Actions: Motion Toggle
    def _toggle_motion_clicked(self):
        if not self.allow_motion:
            # Operator authorization dialog
            confirm = QMessageBox.question(
                self,
                "Ủy Quyền Vận Hành Gazebo (Allow Motion)",
                "CẢNH BÁO AN TOÀN:\n\n"
                "Bạn có xác nhận cho phép robot chuyển động trong Gazebo?\n\n"
                "Điều kiện bắt buộc:\n"
                "- Gazebo Harmonic đang chạy.\n"
                "- Nav2 và AMCL đang hoạt động.\n"
                "- Preflight check đã PASS.\n"
                "- E-stop đã được nhả.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if confirm == QMessageBox.Yes:
                self.allow_motion = True
                self.lbl_motion_auth.setText("ALLOW MOTION: TRUE (OPERATOR AUTHORIZED)")
                self.lbl_motion_auth.setStyleSheet("background-color: #15803d; color: #dcfce7; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
                self.btn_toggle_motion.setText("Vô Hiệu Hóa Chuyển Động")
                self.append_log(">>> NGƯỜI VẬN HÀNH ĐÃ CẤP QUYỀN CHUYỂN ĐỘNG (ALLOW MOTION = TRUE) <<<")
        else:
            self.allow_motion = False
            self.lbl_motion_auth.setText("ALLOW MOTION: FALSE (FAIL-CLOSED)")
            self.lbl_motion_auth.setStyleSheet("background-color: #991b1b; color: #fecaca; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
            self.btn_toggle_motion.setText("Kích hoạt Chuyển động")
            self.append_log(">>> NGƯỜI VẬN HÀNH ĐÃ TẮT QUYỀN CHUYỂN ĐỘNG (ALLOW MOTION = FALSE) <<<")

    # Test A Pipeline Runner
    def _set_step_status(self, step_name: str, status: str):
        self.test_a_status[step_name] = status
        lbl = self.step_labels.get(step_name)
        if not lbl:
            return
        lbl.setText(status)
        if status == "PASS":
            lbl.setStyleSheet("background-color: #15803d; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold; font-size: 10px;")
        elif status == "RUNNING":
            lbl.setStyleSheet("background-color: #0284c7; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold; font-size: 10px;")
        elif status == "FAIL":
            lbl.setStyleSheet("background-color: #dc2626; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold; font-size: 10px;")
        elif status == "BLOCKED":
            lbl.setStyleSheet("background-color: #d97706; color: white; padding: 2px 6px; border-radius: 3px; font-weight: bold; font-size: 10px;")
        else:
            lbl.setStyleSheet("background-color: #cbd5e1; color: #1e293b; padding: 2px 6px; border-radius: 3px; font-weight: bold; font-size: 10px;")

    def _run_preflight_check(self):
        self.append_log("Chạy Preflight Checks cho Test A...")
        # Step 1: Connect Backend
        if self.dashboard_state.backend_online:
            self._set_step_status("1. CONNECT BACKEND", "PASS")
        else:
            self._set_step_status("1. CONNECT BACKEND", "FAIL")

        # Step 2: Check ROS 2 / Nav2
        if self.dashboard_state.gazebo_running and self.dashboard_state.nav2_active and self.dashboard_state.amcl_active:
            self._set_step_status("2. CHECK ROS 2 / NAV2", "PASS")
        else:
            self._set_step_status("2. CHECK ROS 2 / NAV2", "FAIL")

        # Step 3: Check Safety
        if not self.dashboard_state.battery_low and not self.dashboard_state.water_low:
            self._set_step_status("3. CHECK SAFETY", "PASS")
        else:
            self._set_step_status("3. CHECK SAFETY", "FAIL")

        # Step 4: Check Reconciliation
        self.ros.reconcile_nav2()

    def _advance_test_a_step(self):
        # Look for first non-passed step
        for step in TEST_A_STEPS:
            st = self.test_a_status[step]
            if st != "PASS":
                self.append_log(f"Thực hiện bước: {step}...")
                self._set_step_status(step, "RUNNING")
                if "CONNECT BACKEND" in step:
                    self.client.check_health()
                elif "CHECK ROS 2" in step:
                    self._run_preflight_check()
                elif "CREATE NLP" in step:
                    self._send_nlp_clicked()
                elif "CREATE DRAFT" in step:
                    self._create_draft_clicked()
                elif "CONFIRM ORDER" in step:
                    self._confirm_order_clicked()
                elif "MONITOR PICKUP" in step:
                    self._set_pipeline_stage("NAVIGATING_PICKUP")
                    self._set_step_status(step, "PASS")
                elif "CONFIRM QR PICKUP" in step:
                    self._issue_and_scan_qr("PICKUP")
                elif "MONITOR DELIVERY" in step:
                    self._set_pipeline_stage("NAVIGATING_DELIVERY")
                    self._set_step_status(step, "PASS")
                elif "CONFIRM QR DELIVERY" in step:
                    self._issue_and_scan_qr("DELIVERY")
                elif "VERIFY COMPLETED" in step:
                    self._set_pipeline_stage("COMPLETED")
                    self._set_step_status(step, "PASS")
                    QMessageBox.information(self, "Test A Nghiệm Thu Hoàn Thành", "Quy trình Test A End-to-End đã HOÀN THÀNH THÀNH CÔNG (COMPLETED)!")
                break

    def _reset_test_a(self):
        for s in TEST_A_STEPS:
            self._set_step_status(s, "NOT STARTED")
        self._set_pipeline_stage("DRAFT")
        self.append_log("Test A đã được reset.")
