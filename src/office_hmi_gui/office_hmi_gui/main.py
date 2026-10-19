import sys
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QListWidget, QListWidgetItem, QGroupBox,
                             QInputDialog, QMessageBox, QFormLayout, QLineEdit, QSplitter,
                             QScrollArea, QFrame, QComboBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPointF
from PyQt5.QtGui import QPixmap, QImage, QColor, QPen, QBrush, QPolygonF, QFont
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene

from nav_msgs.msg import OccupancyGrid
from office_interfaces.msg import RoomList, RoomState, RoomConfig, DoorConfig, KeepoutZone, ActiveKeepoutZones
from office_interfaces.srv import SetRoomState, SaveRoom, DeleteRoom
from geometry_msgs.msg import Point, Pose2D, Twist, Polygon, Point32
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MapViewer(QGraphicsView):
    clicked_signal = pyqtSignal(float, float)
    dragged_signal = pyqtSignal(float, float, float, float)
    
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.setMouseTracking(True)
        self.map_item = None
        self.map_data = None
        
        self.mode = "IDLE" # IDLE, ANCHOR, DOOR_P1, DOOR_P2, STAGING
        self.temp_p1 = None
        
        self.drag_start = None
        
        # Scaling
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        
    def set_map(self, msg: OccupancyGrid):
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        
        img = QImage(w, h, QImage.Format_RGB888)
        for y in range(h):
            for x in range(w):
                val = msg.data[y * w + x]
                if val == -1:
                    c = QColor(200, 200, 200)
                elif val > 50:
                    c = QColor(0, 0, 0)
                else:
                    c = QColor(255, 255, 255)
                # In ROS, y=0 is bottom, but QImage y=0 is top. We flip Y.
                img.setPixelColor(x, h - 1 - y, c)
                
        pixmap = QPixmap.fromImage(img)
        if not self.map_item:
            self.map_item = self.scene.addPixmap(pixmap)
        else:
            self.map_item.setPixmap(pixmap)
            
    def map_to_world(self, mx, my):
        if not self.map_item: return 0, 0
        h = self.map_item.pixmap().height()
        ros_y = h - 1 - my
        wx = self.origin_x + mx * self.resolution
        wy = self.origin_y + ros_y * self.resolution
        return wx, wy
        
    def world_to_map(self, wx, wy):
        if not self.map_item: return 0, 0
        mx = (wx - self.origin_x) / self.resolution
        ros_y = (wy - self.origin_y) / self.resolution
        h = self.map_item.pixmap().height()
        my = h - 1 - ros_y
        return mx, my

    def mousePressEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        wx, wy = self.map_to_world(scene_pos.x(), scene_pos.y())
        
        if self.mode == "STAGING":
            self.drag_start = (wx, wy)
        else:
            self.clicked_signal.emit(wx, wy)
            
        super().mousePressEvent(event)
        
    def mouseReleaseEvent(self, event):
        if self.mode == "STAGING" and self.drag_start:
            scene_pos = self.mapToScene(event.pos())
            wx, wy = self.map_to_world(scene_pos.x(), scene_pos.y())
            self.dragged_signal.emit(self.drag_start[0], self.drag_start[1], wx, wy)
            self.drag_start = None
        super().mouseReleaseEvent(event)


class ROSNode(Node):
    def __init__(self):
        super().__init__('office_hmi_gui')
        
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_profile)
        self.rooms_sub = self.create_subscription(RoomList, 'office/rooms', self.rooms_cb, 10)
        self.state_sub = self.create_subscription(RoomState, 'office/room_states', self.state_cb, 10)
        
        self.set_state_cli = self.create_client(SetRoomState, 'office/set_room_state')
        self.save_room_cli = self.create_client(SaveRoom, 'office/save_room')
        self.delete_room_cli = self.create_client(DeleteRoom, 'office/delete_room')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Navigation Action Client & Vel Publisher
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_goal_handle = None

        self.map_data = None
        self.rooms = {}
        self.room_states = {}
        self.robot_pose = None

        # Keepout Layer Publisher để GUI có thể đồng bộ trực tiếp tới Nav2 Costmap
        self.keepout_pub = self.create_publisher(ActiveKeepoutZones, '/office/keepout_updates', 10)
        
    def map_cb(self, msg):
        self.map_data = msg
        
    def rooms_cb(self, msg):
        self.rooms = {r.room_id: r for r in msg.rooms}
        self.publish_keepout_zones()
        
    def state_cb(self, msg):
        self.room_states[msg.room_id] = msg.state
        self.publish_keepout_zones()

    def calculate_door_polygon(self, door, room_anchor):
        ax, ay = door.p1.x, door.p1.y
        bx, by = door.p2.x, door.p2.y
        rx, ry = room_anchor.x, room_anchor.y
        vx, vy = bx - ax, by - ay
        dist = math.hypot(vx, vy)
        if dist < 0.01:
            return None
        ux, uy = vx / dist, vy / dist
        nx, ny = -uy, ux
        mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
        mrx, mry = rx - mx, ry - my
        if (nx * mrx + ny * mry) < 0:
            nx, ny = -nx, -ny
        margin = 0.05
        d_out = 0.15
        d_in = max(float(door.zone_depth), 0.25)
        a_ext_x = ax - ux * margin
        a_ext_y = ay - uy * margin
        b_ext_x = bx + ux * margin
        b_ext_y = by + uy * margin
        p1_out_x = a_ext_x - nx * d_out
        p1_out_y = a_ext_y - ny * d_out
        p2_out_x = b_ext_x - nx * d_out
        p2_out_y = b_ext_y - ny * d_out
        p2_in_x = b_ext_x + nx * d_in
        p2_in_y = b_ext_y + ny * d_in
        p1_in_x = a_ext_x + nx * d_in
        p1_in_y = a_ext_y + ny * d_in
        poly = Polygon()
        poly.points = [
            Point32(x=float(p1_out_x), y=float(p1_out_y), z=0.0),
            Point32(x=float(p2_out_x), y=float(p2_out_y), z=0.0),
            Point32(x=float(p2_in_x), y=float(p2_in_y), z=0.0),
            Point32(x=float(p1_in_x), y=float(p1_in_y), z=0.0)
        ]
        return poly

    def publish_keepout_zones(self):
        zones = []
        for room_id, state in self.room_states.items():
            if state == 1:  # BUSY
                room = self.rooms.get(room_id)
                if room:
                    for door in room.doors:
                        poly = self.calculate_door_polygon(door, room.room_anchor)
                        if poly:
                            kz = KeepoutZone()
                            kz.room_id = room_id
                            kz.door_id = door.door_id
                            kz.polygon = poly
                            zones.append(kz)
        msg = ActiveKeepoutZones()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.zones = zones
        self.keepout_pub.publish(msg)

    def send_nav_goal(self, x, y, yaw=0.0, result_callback=None):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return False, "Nav2 action server 'navigate_to_pose' chưa sẵn sàng!"

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
            except Exception as e:
                if result_callback:
                    result_callback(False, str(e))
                return
            if not goal_handle.accepted:
                if result_callback:
                    result_callback(False, "Goal bị từ chối bởi Nav2!")
                return
            self.current_goal_handle = goal_handle
            res_future = goal_handle.get_result_async()
            def res_cb(rf):
                self.current_goal_handle = None
                try:
                    res = rf.result()
                    success = (res.status == 4)  # 4 = STATUS_SUCCEEDED
                    if result_callback:
                        result_callback(success, f"Status: {res.status}")
                except Exception as e:
                    if result_callback:
                        result_callback(False, str(e))
            res_future.add_done_callback(res_cb)

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(goal_response_callback)
        return True, "Goal đã được gửi thành công."

    def cancel_nav_goal(self):
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


class MainWindow(QMainWindow):
    save_completed_signal = pyqtSignal(bool, str)
    delete_completed_signal = pyqtSignal(bool, str)
    sig_mission_log = pyqtSignal(str, object)
    sig_mission_status = pyqtSignal(str)
    sig_goal_done = pyqtSignal(bool, str)
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Office HMI Control Panel")
        self.resize(1300, 850)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #f5f6fa; }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #dcdde1;
                border-radius: 6px;
                margin-top: 12px;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
                color: #2f3640;
            }
            QPushButton {
                background-color: #3498db; color: white;
                border-radius: 4px; padding: 6px; font-weight: bold;
            }
            QPushButton:hover { background-color: #2980b9; }
            QPushButton:disabled { background-color: #bdc3c7; }
            QPushButton#btn_free { background-color: #2ecc71; }
            QPushButton#btn_free:hover { background-color: #27ae60; }
            QPushButton#btn_busy { background-color: #e74c3c; }
            QPushButton#btn_busy:hover { background-color: #c0392b; }
            QPushButton#btn_delete_room { background-color: #e74c3c; }
            QPushButton#btn_delete_room:hover { background-color: #c0392b; }
            QPushButton#btn_delete_room:disabled { background-color: #bdc3c7; }
            QPushButton#btn_start_mission { background-color: #27ae60; font-size: 13px; font-weight: bold; padding: 8px; }
            QPushButton#btn_start_mission:hover { background-color: #219653; }
            QPushButton#btn_cancel_mission { background-color: #c0392b; font-size: 13px; font-weight: bold; padding: 8px; }
            QPushButton#btn_cancel_mission:hover { background-color: #962d22; }
            QLineEdit, QListWidget, QComboBox {
                border: 1px solid #dcdde1; border-radius: 4px; padding: 4px;
            }
        """)
        
        self.viewer = MapViewer()
        self.viewer.clicked_signal.connect(self.on_map_clicked)
        self.viewer.dragged_signal.connect(self.on_map_dragged)
        
        left_panel = QWidget()
        vbox = QVBoxLayout(left_panel)
        vbox.setSpacing(12)
        vbox.setContentsMargins(10, 10, 10, 10)
        
        # --- Room Management ---
        grp_manage = QGroupBox("Room Management")
        lo_manage = QVBoxLayout(grp_manage)
        
        h_add = QHBoxLayout()
        self.edit_new_room = QLineEdit()
        self.edit_new_room.setPlaceholderText("New Room ID")
        self.btn_add_room = QPushButton("➕ Add")
        self.btn_add_room.clicked.connect(self.add_room)
        h_add.addWidget(self.edit_new_room)
        h_add.addWidget(self.btn_add_room)
        lo_manage.addLayout(h_add)
        
        self.list_widget = QListWidget()
        self.list_widget.currentRowChanged.connect(self.on_room_selected)
        lo_manage.addWidget(self.list_widget)

        self.btn_delete_room = QPushButton("🗑 Xóa phòng đang chọn")
        self.btn_delete_room.setObjectName("btn_delete_room")
        self.btn_delete_room.setEnabled(False)
        self.btn_delete_room.clicked.connect(self.delete_room)
        lo_manage.addWidget(self.btn_delete_room)

        vbox.addWidget(grp_manage, stretch=1)
        
        # --- Room Editor ---
        self.grp_editor = QGroupBox("Room Editor")
        form = QFormLayout(self.grp_editor)
        form.setSpacing(8)
        self.lbl_id = QLabel("-")
        self.lbl_id.setStyleSheet("color: #e1b12c; font-weight: bold;")
        self.edit_name = QLineEdit()
        self.edit_desc = QLineEdit()
        self.btn_anchor = QPushButton("Set Anchor")
        self.btn_anchor.clicked.connect(lambda: self.set_mode("ANCHOR"))
        self.btn_door = QPushButton("Set Door")
        self.btn_door.clicked.connect(lambda: self.set_mode("DOOR_P1"))
        self.btn_staging = QPushButton("Set Staging")
        self.btn_staging.clicked.connect(lambda: self.set_mode("STAGING"))
        self.btn_save = QPushButton("Save Config")
        self.btn_save.clicked.connect(self.save_room)
        
        form.addRow("Room ID:", self.lbl_id)
        form.addRow("Name:", self.edit_name)
        form.addRow("Desc:", self.edit_desc)
        form.addRow("Action:", self.btn_anchor)
        form.addRow("", self.btn_door)
        form.addRow("", self.btn_staging)
        form.addRow("", self.btn_save)
        
        self.lbl_details = QLabel("-")
        self.lbl_details.setStyleSheet("font-family: monospace; font-size: 11px; color: #34495e;")
        form.addRow("Details:", self.lbl_details)
        
        self.is_dirty = False
        self.edit_name.textChanged.connect(self.mark_dirty)
        self.edit_desc.textChanged.connect(self.mark_dirty)
        self.save_completed_signal.connect(self.on_save_completed)
        self.delete_completed_signal.connect(self.on_delete_completed)
        
        vbox.addWidget(self.grp_editor, stretch=0)
        self.grp_editor.setEnabled(False)
        
        # --- Room State Control ---
        self.grp_state = QGroupBox("Room State Control")
        h_state = QHBoxLayout(self.grp_state)
        self.btn_free = QPushButton("FREE (Available)")
        self.btn_free.setObjectName("btn_free")
        self.btn_free.clicked.connect(lambda: self.set_state(0))
        self.btn_busy = QPushButton("BUSY (Occupied)")
        self.btn_busy.setObjectName("btn_busy")
        self.btn_busy.clicked.connect(lambda: self.set_state(1))
        h_state.addWidget(self.btn_free)
        h_state.addWidget(self.btn_busy)
        vbox.addWidget(self.grp_state, stretch=0)
        self.grp_state.setEnabled(False)
        
        # --- Mission Dispatch (Giao nhiệm vụ đa phòng) ---
        self.grp_mission = QGroupBox("Mission Dispatch (Giao nhiệm vụ)")
        lo_mission = QVBoxLayout(self.grp_mission)
        lo_mission.setSpacing(6)

        h_rooms = QHBoxLayout()
        self.edit_mission_rooms = QLineEdit()
        self.edit_mission_rooms.setPlaceholderText("ID phòng (vd: 01, 02 hoặc 001, 002)")
        self.btn_add_to_mission = QPushButton("+ Thêm phòng")
        self.btn_add_to_mission.setStyleSheet("background-color: #7f8c8d; padding: 4px; font-weight: normal;")
        self.btn_add_to_mission.clicked.connect(self.add_selected_room_to_mission)
        h_rooms.addWidget(self.edit_mission_rooms, stretch=3)
        h_rooms.addWidget(self.btn_add_to_mission, stretch=2)
        lo_mission.addLayout(h_rooms)

        h_target = QHBoxLayout()
        h_target.addWidget(QLabel("Mục tiêu đến:"))
        self.cmb_target_type = QComboBox()
        self.cmb_target_type.addItems(["Điểm chờ ngoài cửa (Staging Pose)", "Trong phòng (Room Anchor)"])
        h_target.addWidget(self.cmb_target_type)
        lo_mission.addLayout(h_target)

        h_btns = QHBoxLayout()
        self.btn_start_mission = QPushButton("🚀 Bắt đầu nhiệm vụ")
        self.btn_start_mission.setObjectName("btn_start_mission")
        self.btn_start_mission.clicked.connect(self.start_mission)

        self.btn_cancel_mission = QPushButton("🛑 Dừng / Hủy")
        self.btn_cancel_mission.setObjectName("btn_cancel_mission")
        self.btn_cancel_mission.clicked.connect(self.cancel_mission)
        self.btn_cancel_mission.setEnabled(False)

        h_btns.addWidget(self.btn_start_mission)
        h_btns.addWidget(self.btn_cancel_mission)
        lo_mission.addLayout(h_btns)

        self.lbl_mission_status = QLabel("Trạng thái: Sẵn sàng")
        self.lbl_mission_status.setStyleSheet("color: #2c3e50; font-weight: bold; font-size: 11px;")
        lo_mission.addWidget(self.lbl_mission_status)

        vbox.addWidget(self.grp_mission, stretch=0)
        
        # --- Notifications ---
        grp_notify = QGroupBox("System Notifications")
        lo_notify = QVBoxLayout(grp_notify)
        self.log_widget = QListWidget()
        self.log_widget.setMaximumHeight(150)
        lo_notify.addWidget(self.log_widget)
        vbox.addWidget(grp_notify, stretch=1)
        
        # --- Main Splitter ---
        splitter = QSplitter(Qt.Horizontal)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(left_panel)
        scroll.setFrameShape(QFrame.NoFrame)
        splitter.addWidget(scroll)
        
        map_frame = QWidget()
        map_lo = QVBoxLayout(map_frame)
        map_lo.setContentsMargins(0, 10, 10, 10)
        map_lo.setSpacing(6)
        
        # Instruction Banner
        self.lbl_instruction = QLabel("💡 Hướng dẫn: Chọn phòng từ danh sách hoặc tạo phòng mới, sau đó bấm Set Anchor / Door / Staging để định vị.")
        self.lbl_instruction.setStyleSheet("""
            background-color: #2c3e50;
            color: #ffffff;
            font-size: 13px;
            font-weight: bold;
            padding: 8px 14px;
            border-radius: 6px;
            border-left: 5px solid #3498db;
        """)
        self.lbl_instruction.setAlignment(Qt.AlignCenter)
        map_lo.addWidget(self.lbl_instruction)
        
        map_lo.addWidget(self.viewer)
        splitter.addWidget(map_frame)
        
        splitter.setSizes([420, 880])
        self.setCentralWidget(splitter)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(500)
        
        self.current_room = None
        self.editing_room = None
        self.overlay_items = []
        self.force_refresh_list = False  # Flag để force refresh list sau khi save

        # Mission state variables
        self.pending_mission_rooms = []
        self.is_mission_running = False
        self.current_mission_room_id = None

        # Connect mission signals
        self.sig_mission_log.connect(self.log_msg)
        self.sig_mission_status.connect(self.lbl_mission_status.setText)
        self.sig_goal_done.connect(self.on_nav_goal_completed)
        
    def log_msg(self, msg, color=Qt.black):
        item = QListWidgetItem(msg)
        item.setForeground(color)
        self.log_widget.addItem(item)
        self.log_widget.scrollToBottom()
        
    def mark_dirty(self, *args):
        if self.current_room:
            self.is_dirty = True
            
    def update_details_panel(self):
        if not self.editing_room: 
            self.lbl_details.setText("-")
            return
        r = self.editing_room
        text = f"Anchor:\n  x = {r.room_anchor.x:.3f}\n  y = {r.room_anchor.y:.3f}\n\n"
        if r.doors:
            d = r.doors[0]
            text += f"Door: {d.door_id}\n"
            text += f"  p1 = ({d.p1.x:.3f}, {d.p1.y:.3f})\n"
            text += f"  p2 = ({d.p2.x:.3f}, {d.p2.y:.3f})\n"
            text += f"  zone_depth = {d.zone_depth:.3f}\n\n"
            text += f"Staging:\n"
            text += f"  x = {d.staging_pose.x:.3f}\n  y = {d.staging_pose.y:.3f}\n  yaw = {d.staging_pose.theta:.3f} rad"
        else:
            text += "Door: None\nStaging: None"
        self.lbl_details.setText(text)
        
    def set_mode(self, mode):
        self.viewer.mode = mode
        if mode == "DOOR_P1":
            self.viewer.temp_p1 = None
        self.statusBar().showMessage(f"Mode: {mode}")
        
        if mode == "ANCHOR":
            self.lbl_instruction.setText("📍 CHẾ ĐỘ ĐẶT TÂM PHÒNG (ANCHOR): Vui lòng click chuột lên vị trí tâm phòng trên bản đồ!")
            self.lbl_instruction.setStyleSheet("""
                background-color: #d35400; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #f39c12;
            """)
            self.log_msg("📍 Đang bật chế độ đặt Anchor: Hãy click chuột vào vị trí tâm phòng trên bản đồ...", Qt.darkBlue)
        elif mode == "DOOR_P1":
            self.lbl_instruction.setText("🚪 CHẾ ĐỘ ĐẶT CỬA (BƯỚC 1/2): Click chuột vào điểm mép thứ nhất (P1) của cửa...")
            self.lbl_instruction.setStyleSheet("""
                background-color: #2980b9; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #3498db;
            """)
            self.log_msg("🚪 Đang đặt cửa: Click chuột vào điểm mép thứ nhất (P1) của cửa...", Qt.darkBlue)
        elif mode == "DOOR_P2":
            self.lbl_instruction.setText("🚪 CHẾ ĐỘ ĐẶT CỬA (BƯỚC 2/2): Điểm P1 đã chọn! Click tiếp vào điểm mép thứ hai (P2) của cửa...")
            self.lbl_instruction.setStyleSheet("""
                background-color: #16a085; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #1abc9c;
            """)
            self.log_msg("🚪 Điểm P1 đã chọn! Click tiếp vào điểm mép thứ 2 (P2) của cửa...", Qt.darkBlue)
        elif mode == "STAGING":
            self.lbl_instruction.setText("🚩 CHẾ ĐỘ ĐẶT ĐIỂM CHỜ (STAGING): Nhấn giữ và KÉO chuột trước cửa để chọn vị trí + hướng xe đỗ...")
            self.lbl_instruction.setStyleSheet("""
                background-color: #8e44ad; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #9b59b6;
            """)
            self.log_msg("🚩 Đang đặt Staging: Nhấn giữ và KÉO chuột trước cửa để đặt vị trí và hướng xe đỗ...", Qt.darkBlue)
        else: # IDLE
            self.lbl_instruction.setText("💡 Sẵn sàng: Chọn phòng để chỉnh sửa hoặc nhập ID vào ô bên dưới để giao nhiệm vụ.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #2c3e50; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #3498db;
            """)
        self.draw_overlays()
        
    def update_gui(self):
        try:
            trans = self.ros_node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.ros_node.robot_pose = (trans.transform.translation.x, trans.transform.translation.y)
        except TransformException:
            pass
            
        if self.ros_node.map_data:
            self.viewer.set_map(self.ros_node.map_data)
            self.ros_node.map_data = None
            
        names = [f"{r.room_id}: {r.display_name}" for r in self.ros_node.rooms.values()]
        current_ids = [r.room_id for r in self.ros_node.rooms.values()]
        list_needs_refresh = (
            self.list_widget.count() != len(names) or
            self.force_refresh_list
        )
        self.force_refresh_list = False

        if list_needs_refresh:
            # Lưu lại row đang chọn để restore sau khi refresh
            prev_row = self.list_widget.currentRow()
            prev_room_id = self.current_room

            self.list_widget.blockSignals(True)
            self.list_widget.clear()
            self.list_widget.addItems(names)
            self.list_widget.blockSignals(False)

            # Restore selection: ưu tiên restore về phòng vừa lưu
            target_id = prev_room_id
            if target_id and target_id in current_ids:
                new_row = current_ids.index(target_id)
            elif prev_row >= 0 and prev_row < len(names):
                new_row = prev_row
            else:
                new_row = -1

            if new_row >= 0:
                self.list_widget.setCurrentRow(new_row)
            
        self.draw_overlays()
            
    def on_room_selected(self, row):
        if row < 0: return
        room_id = list(self.ros_node.rooms.keys())[row]
        room = self.ros_node.rooms[room_id]
        
        self.current_room = room_id
        self.editing_room = RoomConfig()
        self.editing_room.room_id = room.room_id
        self.editing_room.display_name = room.display_name
        self.editing_room.description = room.description
        self.editing_room.room_anchor = room.room_anchor
        self.editing_room.doors = room.doors
        
        self.lbl_id.setText(room.room_id)
        self.edit_name.blockSignals(True)
        self.edit_desc.blockSignals(True)
        self.edit_name.setText(room.display_name)
        self.edit_desc.setText(room.description)
        self.edit_name.blockSignals(False)
        self.edit_desc.blockSignals(False)
        
        self.grp_editor.setEnabled(True)
        self.grp_state.setEnabled(True)
        self.btn_delete_room.setEnabled(True)
        self.is_dirty = False
        self.set_mode("IDLE")
        self.update_details_panel()
        
    def add_room(self):
        rid = self.edit_new_room.text().strip()
        if rid:
            if rid in self.ros_node.rooms:
                QMessageBox.warning(self, "Warning", "Room ID already exists!")
                return
            r = RoomConfig()
            r.room_id = rid
            self.ros_node.rooms[rid] = r
            self.on_room_selected(list(self.ros_node.rooms.keys()).index(rid))
            self.log_msg(f"Đã thêm phòng mới: {rid}", Qt.blue)
            self.edit_new_room.clear()

    def on_map_clicked(self, wx, wy):
        if not self.editing_room: return
        
        if self.viewer.mode == "ANCHOR":
            self.editing_room.room_anchor.x = wx
            self.editing_room.room_anchor.y = wy
            self.set_mode("IDLE")
            self.mark_dirty()
            self.update_details_panel()
            self.lbl_instruction.setText(f"✅ Đã đặt Anchor tại: ({wx:.2f}, {wy:.2f}) thành công! Đừng quên bấm 'Save Config'.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.log_msg(f"✅ Đã đặt tọa độ Neo (Anchor) tại: x={wx:.2f}, y={wy:.2f}", Qt.darkGreen)
            self.draw_overlays()
            QMessageBox.information(
                self, 
                "Thông báo Anchor", 
                f"✅ Đã đặt tâm phòng (Anchor) cho phòng '{self.current_room}' thành công!\n\n"
                f"• Tọa độ: X = {wx:.2f} m, Y = {wy:.2f} m\n\n"
                f"Vị trí Anchor đã hiển thị trực tiếp trên bản đồ.\nHãy bấm 'Save Config' để lưu cấu hình!"
            )
            
        elif self.viewer.mode == "DOOR_P1":
            self.viewer.temp_p1 = (wx, wy)
            self.set_mode("DOOR_P2")
            self.log_msg(f"📍 Đã chọn điểm P1 của cửa tại ({wx:.2f}, {wy:.2f}). Hãy click tiếp để chọn điểm P2.", Qt.blue)
            self.draw_overlays()
            
        elif self.viewer.mode == "DOOR_P2":
            p1 = self.viewer.temp_p1
            p2 = (wx, wy)
            door = DoorConfig()
            door.door_id = "main_door"
            door.p1.x, door.p1.y = p1[0], p1[1]
            door.p2.x, door.p2.y = p2[0], p2[1]
            door.zone_depth = 0.3
            self.editing_room.doors = [door]
            self.set_mode("IDLE")
            self.mark_dirty()
            self.update_details_panel()
            d_len = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            self.lbl_instruction.setText(f"✅ Đã tạo cửa phòng (Rộng: {d_len:.2f}m)! Tiếp theo hãy bấm 'Set Staging' để đặt điểm đỗ.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.log_msg(f"✅ Đã thiết lập Cửa (P1-P2, rộng {d_len:.2f}m) thành công.", Qt.darkGreen)
            self.draw_overlays()
            QMessageBox.information(
                self,
                "Thông báo Cửa phòng",
                f"✅ Đã thiết lập Cửa cho phòng '{self.current_room}' thành công!\n\n"
                f"• Điểm P1: ({p1[0]:.2f}, {p1[1]:.2f})\n"
                f"• Điểm P2: ({p2[0]:.2f}, {p2[1]:.2f})\n"
                f"• Bề rộng cửa: {d_len:.2f} m\n\n"
                f"Cửa (mép A-B) và vùng cửa (Keepout zone) đã hiển thị trên bản đồ!\n"
                f"Tiếp theo, hãy bấm 'Set Staging' để chọn vị trí và hướng xe dừng đỗ."
            )
            
    def on_map_dragged(self, sx, sy, ex, ey):
        if not self.editing_room: return
        if self.viewer.mode == "STAGING":
            if not self.editing_room.doors:
                QMessageBox.warning(self, "Warning", "Hãy đặt cửa (Set Door) trước khi đặt điểm đỗ Staging!")
                self.set_mode("IDLE")
                return
                
            dx = ex - sx
            dy = ey - sy
            door = self.editing_room.doors[0]
            mx = (door.p1.x + door.p2.x)/2
            my = (door.p1.y + door.p2.y)/2
            
            if math.hypot(dx, dy) < 0.1:
                yaw = math.atan2(my - sy, mx - sx)
            else:
                yaw = math.atan2(dy, dx)
                
            door.staging_pose.x = sx
            door.staging_pose.y = sy
            door.staging_pose.theta = yaw
            
            self.set_mode("IDLE")
            self.mark_dirty()
            self.update_details_panel()
            self.lbl_instruction.setText(f"✅ Đã đặt Staging tại ({sx:.2f}, {sy:.2f}, yaw={yaw:.2f} rad)! Bấm 'Save Config' để lưu lại.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.log_msg(f"✅ Đã thiết lập xong điểm Chờ (Staging) tại ({sx:.2f}, {sy:.2f}).", Qt.darkGreen)
            self.draw_overlays()
            QMessageBox.information(
                self,
                "Thông báo Điểm chờ (Staging)",
                f"✅ Đã đặt điểm chờ Staging cho phòng '{self.current_room}' thành công!\n\n"
                f"• Tọa độ: X = {sx:.2f} m, Y = {sy:.2f} m\n"
                f"• Hướng xe (Yaw): {math.degrees(yaw):.1f}° ({yaw:.2f} rad)\n\n"
                f"Điểm Staging (ký hiệu 'S' và mũi tên hướng xe) đã hiển thị trên bản đồ.\n"
                f"Đừng quên bấm 'Save Config' để lưu cấu hình vào hệ thống!"
            )
            
    def save_room(self):
        if not self.editing_room: return
        self.editing_room.display_name = self.edit_name.text()
        self.editing_room.description = self.edit_desc.text()
        
        req = SaveRoom.Request()
        req.room = self.editing_room
        
        future = self.ros_node.save_room_cli.call_async(req)
        future.add_done_callback(self.save_room_done)
        self.statusBar().showMessage("Room saved request sent.", 3000)
        
    def save_room_done(self, future):
        try:
            res = future.result()
            self.save_completed_signal.emit(res.success, res.message)
        except Exception as e:
            self.save_completed_signal.emit(False, str(e))
            
    def on_save_completed(self, success, message):
        if success:
            self.is_dirty = False
            self.ros_node.rooms[self.current_room] = self.editing_room
            self.log_msg(f"💾 Lưu thành công thông tin phòng {self.current_room}!", Qt.darkGreen)
            self.lbl_instruction.setText(f"💾 Đã lưu cấu hình phòng {self.current_room} thành công!")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.force_refresh_list = True  # Ép refresh list để hiển thị đúng phòng vừa lưu
            self.draw_overlays()
            QMessageBox.information(
                self,
                "Lưu thành công",
                f"💾 Đã lưu cấu hình phòng '{self.current_room}' thành công vào file hệ thống!"
            )
        else:
            self.log_msg(f"❌ Lỗi khi lưu: {message}", Qt.red)
            self.lbl_instruction.setText(f"❌ Lỗi khi lưu: {message}")
            self.lbl_instruction.setStyleSheet("""
                background-color: #c0392b; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #e74c3c;
            """)
            QMessageBox.critical(
                self,
                "Lỗi lưu cấu hình",
                f"❌ Không thể lưu cấu hình phòng '{self.current_room}':\n{message}"
            )
        
    def set_state(self, state):
        if not self.current_room: return
        # Cập nhật ngay state cục bộ & phát keepout layer lập tức để GUI phản hồi tức thì
        self.ros_node.room_states[self.current_room] = state
        self.ros_node.publish_keepout_zones()
        self.draw_overlays()
        
        req = SetRoomState.Request()
        req.room_id = self.current_room
        req.state = state
        self.ros_node.set_state_cli.call_async(req)
        
        if state == 1:
            s_text = "BUSY (Cửa Đóng / Kích hoạt Layer Cản Keepout)"
            self.log_msg(f"🔒 Đã khóa phòng [{self.current_room}]: Đường cửa P1-P2 đã biến thành Keepout Layer!", QColor(192, 57, 43))
        else:
            s_text = "FREE (Cửa Mở / Giải phóng Keepout Layer)"
            self.log_msg(f"🔓 Đã mở phòng [{self.current_room}]: Cửa thông thoáng, giải phóng Keepout Layer.", Qt.darkGreen)

    def delete_room(self):
        if not self.current_room:
            return

        room_name = ""
        if self.editing_room and self.editing_room.display_name:
            room_name = f" ({self.editing_room.display_name})"

        reply = QMessageBox.question(
            self,
            "Xác nhận xóa phòng",
            f"⚠️ Bạn có chắc muốn XÓA VĨNH VIỄN phòng '{self.current_room}'{room_name} không?\n\n"
            f"Hành động này sẽ xóa toàn bộ cấu hình (Anchor, Cửa, Staging) của phòng khỏi file YAML và KHÔNG THỂ KHÔI PHỤC!",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        req = DeleteRoom.Request()
        req.room_id = self.current_room
        future = self.ros_node.delete_room_cli.call_async(req)
        future.add_done_callback(self.delete_room_done)
        self.btn_delete_room.setEnabled(False)
        self.statusBar().showMessage(f"Đang xóa phòng {self.current_room}...", 3000)

    def delete_room_done(self, future):
        try:
            res = future.result()
            self.delete_completed_signal.emit(res.success, res.message)
        except Exception as e:
            self.delete_completed_signal.emit(False, str(e))

    def on_delete_completed(self, success, message):
        if success:
            deleted_id = self.current_room
            # Reset UI state
            self.current_room = None
            self.editing_room = None
            self.is_dirty = False
            self.grp_editor.setEnabled(False)
            self.grp_state.setEnabled(False)
            self.btn_delete_room.setEnabled(False)
            self.lbl_id.setText("-")
            self.edit_name.blockSignals(True)
            self.edit_desc.blockSignals(True)
            self.edit_name.clear()
            self.edit_desc.clear()
            self.edit_name.blockSignals(False)
            self.edit_desc.blockSignals(False)
            self.lbl_details.setText("-")
            # Xóa khỏi dict local và refresh list
            if deleted_id in self.ros_node.rooms:
                del self.ros_node.rooms[deleted_id]
            self.force_refresh_list = True
            self.set_mode("IDLE")
            self.log_msg(f"🗑 Đã xóa phòng '{deleted_id}' thành công!", Qt.darkRed)
            self.lbl_instruction.setText(f"🗑 Đã xóa phòng '{deleted_id}' thành công!")
            self.lbl_instruction.setStyleSheet("""
                background-color: #922b21; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #e74c3c;
            """)
            QMessageBox.information(
                self,
                "Xóa thành công",
                f"🗑 Đã xóa phòng '{deleted_id}' thành công khỏi hệ thống!"
            )
        else:
            self.btn_delete_room.setEnabled(True)
            self.log_msg(f"❌ Lỗi khi xóa phòng: {message}", Qt.red)
            self.lbl_instruction.setText(f"❌ Lỗi xóa phòng: {message}")
            self.lbl_instruction.setStyleSheet("""
                background-color: #c0392b; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #e74c3c;
            """)
            QMessageBox.critical(
                self,
                "Lỗi xóa phòng",
                f"❌ Không thể xóa phòng '{self.current_room}':\n{message}"
            )

    def resolve_room_id(self, raw_id: str):
        raw_id = raw_id.strip()
        if not raw_id:
            return None
        rooms = self.ros_node.rooms
        if raw_id in rooms:
            return raw_id
        if raw_id.isdigit():
            val = int(raw_id)
            for cand in [str(val), f"{val:02d}", f"{val:03d}", f"room_{val:02d}", f"room_{val:03d}"]:
                if cand in rooms:
                    return cand
        if f"room_{raw_id}" in rooms:
            return f"room_{raw_id}"
        for rid, r in rooms.items():
            if r.display_name.strip().lower() == raw_id.lower() or rid.lower() == raw_id.lower():
                return rid
        return None

    def add_selected_room_to_mission(self):
        if not self.current_room:
            return
        current_text = self.edit_mission_rooms.text().strip()
        if current_text:
            self.edit_mission_rooms.setText(f"{current_text}, {self.current_room}")
        else:
            self.edit_mission_rooms.setText(self.current_room)

    def start_mission(self):
        raw_text = self.edit_mission_rooms.text().strip()
        if not raw_text:
            QMessageBox.warning(self, "Chưa nhập phòng", "Vui lòng nhập ID phòng cần đến (ví dụ: 01, 02)!")
            return

        raw_list = [r.strip() for r in raw_text.replace(';', ',').split(',') if r.strip()]
        resolved_rooms = []
        for r in raw_list:
            m_id = self.resolve_room_id(r)
            if m_id:
                resolved_rooms.append(m_id)
            else:
                self.log_msg(f"⚠️ Cảnh báo: Không tìm thấy ID phòng tương ứng với '{r}'! Đã bỏ qua.", Qt.darkYellow)

        if not resolved_rooms:
            QMessageBox.warning(self, "Không tìm thấy phòng", "Không có ID phòng nào hợp lệ trong cấu hình hiện tại!")
            return

        self.pending_mission_rooms = resolved_rooms
        self.is_mission_running = True
        self.btn_start_mission.setEnabled(False)
        self.btn_cancel_mission.setEnabled(True)
        self.log_msg(f"📋 Bắt đầu lộ trình giao việc gồm {len(self.pending_mission_rooms)} phòng: {', '.join(self.pending_mission_rooms)}", Qt.darkBlue)
        self.process_next_mission_room()

    def process_next_mission_room(self):
        if not self.is_mission_running:
            return

        if not self.pending_mission_rooms:
            self.log_msg("🎉 ĐÃ HOÀN THÀNH TẤT CẢ CÁC NHIỆM VỤ GIAO PHÒNG!", Qt.darkGreen)
            self.lbl_mission_status.setText("Trạng thái: Hoàn thành nhiệm vụ!")
            self.is_mission_running = False
            self.btn_start_mission.setEnabled(True)
            self.btn_cancel_mission.setEnabled(False)
            return

        current_rid = self.pending_mission_rooms.pop(0)
        room = self.ros_node.rooms.get(current_rid)
        if not room:
            self.log_msg(f"❌ Không tìm thấy thông tin phòng {current_rid}. Bỏ qua.", Qt.red)
            QTimer.singleShot(1000, self.process_next_mission_room)
            return

        # KIỂM TRA TRẠNG THÁI BUSY
        state = self.ros_node.room_states.get(current_rid, 0)
        if state == 1:
            self.log_msg(
                f"⚠️ Phòng [{current_rid} - {room.display_name}] đang BẬN (BUSY - Cửa khóa/Keepout đang bật)! ➔ TỰ ĐỘNG BỎ QUA và chuyển sang phòng kế tiếp...",
                QColor(211, 84, 0)
            )
            self.lbl_mission_status.setText(f"Phòng {current_rid} bận ➔ Chuyển tiếp...")
            QTimer.singleShot(2000, self.process_next_mission_room)
            return

        # XÁC ĐỊNH TỌA ĐỘ ĐÍCH
        target_is_staging = (self.cmb_target_type.currentIndex() == 0)
        tx, ty, tyaw = None, None, 0.0
        target_desc = ""

        if target_is_staging:
            if room.doors and (room.doors[0].staging_pose.x != 0.0 or room.doors[0].staging_pose.y != 0.0):
                d = room.doors[0]
                tx, ty, tyaw = d.staging_pose.x, d.staging_pose.y, d.staging_pose.theta
                target_desc = f"Điểm chờ ngoài cửa ({tx:.2f}, {ty:.2f})"
            elif room.room_anchor and (room.room_anchor.x != 0.0 or room.room_anchor.y != 0.0):
                tx, ty, tyaw = room.room_anchor.x, room.room_anchor.y, 0.0
                target_desc = f"Tâm phòng Anchor ({tx:.2f}, {ty:.2f}) [Fallback]"
        else:
            if room.room_anchor and (room.room_anchor.x != 0.0 or room.room_anchor.y != 0.0):
                tx, ty, tyaw = room.room_anchor.x, room.room_anchor.y, 0.0
                target_desc = f"Trong phòng Anchor ({tx:.2f}, {ty:.2f})"
            elif room.doors and (room.doors[0].staging_pose.x != 0.0 or room.doors[0].staging_pose.y != 0.0):
                d = room.doors[0]
                tx, ty, tyaw = d.staging_pose.x, d.staging_pose.y, d.staging_pose.theta
                target_desc = f"Điểm chờ ngoài cửa ({tx:.2f}, {ty:.2f}) [Fallback]"

        if tx is None:
            self.log_msg(f"❌ Phòng [{current_rid}] chưa được cài đặt tọa độ Staging hoặc Anchor! Bỏ qua.", Qt.red)
            QTimer.singleShot(1000, self.process_next_mission_room)
            return

        self.current_mission_room_id = current_rid
        self.log_msg(f"🚗 [Nhiệm vụ] Phòng [{current_rid} - {room.display_name}] RẢNH. Đang di chuyển đến {target_desc}...", Qt.blue)
        self.lbl_mission_status.setText(f"Đang đi đến phòng {current_rid}...")

        # Gửi lệnh Nav2
        def on_result(success, message):
            self.sig_goal_done.emit(success, message)

        ok, err_msg = self.ros_node.send_nav_goal(tx, ty, tyaw, result_callback=on_result)
        if not ok:
            self.log_msg(f"❌ Lỗi gửi Nav2: {err_msg}", Qt.red)
            self.lbl_mission_status.setText("Lỗi kết nối Nav2!")
            self.is_mission_running = False
            self.btn_start_mission.setEnabled(True)
            self.btn_cancel_mission.setEnabled(False)

    def on_nav_goal_completed(self, success, message):
        if not self.is_mission_running:
            return

        rid = getattr(self, 'current_mission_room_id', 'Unknown')
        if success:
            self.log_msg(f"✅ Đã đến phòng [{rid}] an toàn! Dừng phục vụ 3 giây trước khi đi phòng tiếp theo...", Qt.darkGreen)
            self.lbl_mission_status.setText(f"Đã đến phòng {rid}. Đang chờ 3s...")
            QTimer.singleShot(3000, self.process_next_mission_room)
        else:
            self.log_msg(f"⚠️ Di chuyển đến phòng [{rid}] không thành công ({message})! Chuyển sang phòng tiếp theo...", Qt.darkYellow)
            self.lbl_mission_status.setText(f"Thất bại phòng {rid} ➔ Chuyển tiếp...")
            QTimer.singleShot(2000, self.process_next_mission_room)

    def cancel_mission(self):
        self.is_mission_running = False
        self.pending_mission_rooms = []
        self.ros_node.cancel_nav_goal()
        self.log_msg("🛑 Đã hủy toàn bộ nhiệm vụ và phát lệnh dừng xe khẩn cấp!", Qt.red)
        self.lbl_mission_status.setText("Trạng thái: Đã dừng / Hủy")
        self.btn_start_mission.setEnabled(True)
        self.btn_cancel_mission.setEnabled(False)
        
    def draw_overlays(self):
        for item in self.overlay_items:
            self.viewer.scene.removeItem(item)
        self.overlay_items.clear()
        
        # 1. Preview temp_p1 (A pending) if in DOOR_P2 mode
        if self.viewer.mode == "DOOR_P2" and self.viewer.temp_p1:
            px, py = self.viewer.world_to_map(self.viewer.temp_p1[0], self.viewer.temp_p1[1])
            ca = self.viewer.scene.addEllipse(px-10, py-10, 20, 20, QPen(QColor(41, 128, 185), 3), QBrush(QColor(52, 152, 219, 180)))
            ca.setZValue(5.0)
            ta = self.viewer.scene.addText("📍 P1 (Mép 1) - Click tiếp điểm P2")
            ta.setDefaultTextColor(QColor(41, 128, 185))
            f_p1 = QFont()
            f_p1.setBold(True)
            f_p1.setPointSize(10)
            ta.setFont(f_p1)
            ta.setPos(px+12, py-20)
            ta.setZValue(5.1)
            self.overlay_items.extend([ca, ta])
        
        for rid, room in self.ros_node.rooms.items():
            is_editing = (rid == self.current_room and self.editing_room)
            r = self.editing_room if is_editing else room
            
            # --- Anchor ---
            if r.room_anchor.x != 0.0 or r.room_anchor.y != 0.0:
                ax, ay = self.viewer.world_to_map(r.room_anchor.x, r.room_anchor.y)
                state = self.ros_node.room_states.get(rid, 0)
                color = Qt.red if state == 1 else Qt.darkGreen
                
                # Outer highlight ring if editing
                if is_editing:
                    ring = self.viewer.scene.addEllipse(ax-18, ay-18, 36, 36, QPen(QColor(230, 126, 34), 2, Qt.DashLine), QBrush(Qt.transparent))
                    ring.setZValue(5.0)
                    self.overlay_items.append(ring)

                # Anchor target circle
                pen_a = QPen(QColor(230, 126, 34) if is_editing else QColor(41, 128, 185), 2)
                brush_a = QBrush(QColor(243, 156, 18, 160) if is_editing else QColor(52, 152, 219, 120))
                c_anchor = self.viewer.scene.addEllipse(ax-8, ay-8, 16, 16, pen_a, brush_a)
                c_anchor.setZValue(5.0)
                
                # Crosshair
                pen_cross = QPen(Qt.black if is_editing else Qt.darkBlue, 2)
                l1 = self.viewer.scene.addLine(ax-12, ay, ax+12, ay, pen_cross)
                l2 = self.viewer.scene.addLine(ax, ay-12, ax, ay+12, pen_cross)
                c_dot = self.viewer.scene.addEllipse(ax-3, ay-3, 6, 6, QPen(Qt.red, 1), QBrush(Qt.red))
                l1.setZValue(5.0); l2.setZValue(5.0); c_dot.setZValue(5.1)
                self.overlay_items.extend([c_anchor, l1, l2, c_dot])
                
                # Label with background badge
                display_text = f"📍 {r.display_name if r.display_name else rid}"
                text = self.viewer.scene.addText(display_text)
                font = QFont()
                font.setPointSize(10 if is_editing else 9)
                font.setBold(True)
                text.setFont(font)
                text.setDefaultTextColor(color)
                
                br = text.boundingRect()
                tx = ax - br.width() / 2
                ty = ay - br.height() - 10
                text.setPos(tx, ty)
                text.setZValue(5.2)
                badge = self.viewer.scene.addRect(
                    text.boundingRect().translated(tx, ty),
                    QPen(QColor(200, 200, 200), 1),
                    QBrush(QColor(255, 255, 255, 220))
                )
                badge.setZValue(5.1)
                self.overlay_items.extend([badge, text])
            
            # --- Doors (Hiển thị Cửa & Layer Rào Chắn Khi Phòng Bận) ---
            for d in r.doors:
                p1x, p1y = self.viewer.world_to_map(d.p1.x, d.p1.y)
                p2x, p2y = self.viewer.world_to_map(d.p2.x, d.p2.y)
                state = self.ros_node.room_states.get(rid, 0)

                vx = d.p2.x - d.p1.x
                vy = d.p2.y - d.p1.y
                d_len = math.hypot(vx, vy)
                
                # Tính toán vùng rào chắn / keepout layer bao trọn đường nối P1-P2
                if d_len > 0:
                    ux = vx / d_len
                    uy = vy / d_len
                    nx = -uy
                    ny = ux
                    mx = (d.p1.x + d.p2.x) / 2.0
                    my = (d.p1.y + d.p2.y) / 2.0
                    mrx = r.room_anchor.x - mx
                    mry = r.room_anchor.y - my
                    if (nx * mrx + ny * mry) < 0:
                        nx, ny = -nx, -ny

                    margin = 0.05
                    d_out = 0.15
                    d_in = max(float(d.zone_depth), 0.25)

                    a_ext_x = d.p1.x - ux * margin
                    a_ext_y = d.p1.y - uy * margin
                    b_ext_x = d.p2.x + ux * margin
                    b_ext_y = d.p2.y + uy * margin

                    p1_out_x, p1_out_y = a_ext_x - nx * d_out, a_ext_y - ny * d_out
                    p2_out_x, p2_out_y = b_ext_x - nx * d_out, b_ext_y - ny * d_out
                    p2_in_x, p2_in_y = b_ext_x + nx * d_in, b_ext_y + ny * d_in
                    p1_in_x, p1_in_y = a_ext_x + nx * d_in, a_ext_y + ny * d_in

                    m_p1_out = self.viewer.world_to_map(p1_out_x, p1_out_y)
                    m_p2_out = self.viewer.world_to_map(p2_out_x, p2_out_y)
                    m_p2_in = self.viewer.world_to_map(p2_in_x, p2_in_y)
                    m_p1_in = self.viewer.world_to_map(p1_in_x, p1_in_y)

                    barrier_poly = QPolygonF([
                        QPointF(m_p1_out[0], m_p1_out[1]),
                        QPointF(m_p2_out[0], m_p2_out[1]),
                        QPointF(m_p2_in[0], m_p2_in[1]),
                        QPointF(m_p1_in[0], m_p1_in[1])
                    ])

                    if state == 1:
                        # --- BUSY: Layer Vùng cản Keepout Đỏ Nổi Bật ---
                        b_brush = QBrush(QColor(231, 76, 60, 160))
                        b_pen = QPen(QColor(192, 57, 43), 2, Qt.DashLine)
                        pitem = self.viewer.scene.addPolygon(barrier_poly, b_pen, b_brush)
                        pitem.setZValue(3.0)
                        self.overlay_items.append(pitem)
                    else:
                        # --- FREE: Vùng tiếp cận xanh nhạt ---
                        b_brush = QBrush(QColor(46, 204, 113, 80))
                        b_pen = QPen(QColor(39, 174, 96), 1, Qt.DashLine)
                        pitem = self.viewer.scene.addPolygon(barrier_poly, b_pen, b_brush)
                        pitem.setZValue(3.0)
                        self.overlay_items.append(pitem)

                # --- ĐƯỜNG NỐI GIỮA 2 ĐIỂM CỦA CỬA THÀNH 1 LAYER TƯỜNG CẢN ---
                if state == 1:
                    # 1. Hiệu ứng viền phát sáng cảnh báo (Outer Glow)
                    pen_glow = QPen(QColor(231, 76, 60, 120), 10, Qt.SolidLine)
                    pen_glow.setCapStyle(Qt.RoundCap)
                    dline_glow = self.viewer.scene.addLine(p1x, p1y, p2x, p2y, pen_glow)
                    dline_glow.setZValue(4.1)
                    self.overlay_items.append(dline_glow)

                    # 2. Đường cửa đỏ đậm (Virtual Wall / Closed Barrier)
                    pen_door = QPen(QColor(192, 57, 43), 6, Qt.SolidLine)
                    pen_door.setCapStyle(Qt.RoundCap)
                    dline = self.viewer.scene.addLine(p1x, p1y, p2x, p2y, pen_door)
                    dline.setZValue(4.2)
                    self.overlay_items.append(dline)

                    # 3. Điểm mút P1 (A) và P2 (B) màu đỏ đậm
                    cr = 8 if is_editing else 6
                    ca = self.viewer.scene.addEllipse(p1x - cr, p1y - cr, cr * 2, cr * 2, QPen(QColor(146, 43, 33), 2), QBrush(QColor(231, 76, 60)))
                    cb = self.viewer.scene.addEllipse(p2x - cr, p2y - cr, cr * 2, cr * 2, QPen(QColor(146, 43, 33), 2), QBrush(QColor(231, 76, 60)))
                    ca.setZValue(4.5); cb.setZValue(4.5)
                    self.overlay_items.extend([ca, cb])

                    # 4. Badge "🔒 CỬA KHÓA" tại trung điểm cửa
                    mmx, mmy = (p1x + p2x) / 2.0, (p1y + p2y) / 2.0
                    badge_text = self.viewer.scene.addText("🔒 CỬA KHÓA")
                    f_busy = QFont(); f_busy.setBold(True); f_busy.setPointSize(8)
                    badge_text.setFont(f_busy)
                    badge_text.setDefaultTextColor(Qt.white)
                    b_rect = badge_text.boundingRect()
                    bt_x = mmx - b_rect.width() / 2.0
                    bt_y = mmy - b_rect.height() / 2.0
                    badge_text.setPos(bt_x, bt_y)
                    badge_text.setZValue(4.8)
                    badge_bg = self.viewer.scene.addRect(
                        badge_text.boundingRect().translated(bt_x, bt_y),
                        QPen(QColor(146, 43, 33), 1),
                        QBrush(QColor(192, 57, 43, 230))
                    )
                    badge_bg.setZValue(4.7)
                    self.overlay_items.extend([badge_bg, badge_text])

                else:
                    # FREE: Cửa mở màu xanh thông thoáng
                    pen_door = QPen(QColor(41, 128, 185) if is_editing else QColor(39, 174, 96), 4 if is_editing else 3)
                    dline = self.viewer.scene.addLine(p1x, p1y, p2x, p2y, pen_door)
                    dline.setZValue(4.0)
                    self.overlay_items.append(dline)

                    # Điểm mút A, B màu xanh
                    cr = 8 if is_editing else 5
                    ca = self.viewer.scene.addEllipse(p1x - cr, p1y - cr, cr * 2, cr * 2, pen_door, QBrush(QColor(52, 152, 219) if is_editing else QColor(46, 204, 113)))
                    cb = self.viewer.scene.addEllipse(p2x - cr, p2y - cr, cr * 2, cr * 2, pen_door, QBrush(QColor(52, 152, 219) if is_editing else QColor(46, 204, 113)))
                    ca.setZValue(4.5); cb.setZValue(4.5)
                    ta = self.viewer.scene.addText("A")
                    tb = self.viewer.scene.addText("B")
                    ta.setDefaultTextColor(Qt.white); tb.setDefaultTextColor(Qt.white)
                    f_ab = QFont(); f_ab.setBold(True); f_ab.setPointSize(8)
                    ta.setFont(f_ab); tb.setFont(f_ab)
                    ta.setPos(p1x - cr + 2, p1y - cr - 1)
                    tb.setPos(p2x - cr + 2, p2y - cr - 1)
                    ta.setZValue(4.6); tb.setZValue(4.6)
                    self.overlay_items.extend([ca, cb, ta, tb])
                    
                # --- Staging Pose ---
                if d.staging_pose.x != 0.0 or d.staging_pose.y != 0.0:
                    sx, sy = self.viewer.world_to_map(d.staging_pose.x, d.staging_pose.y)
                    sr = 8 if is_editing else 6
                    pen_stage = QPen(QColor(142, 68, 173), 3 if is_editing else 2)
                    sitem = self.viewer.scene.addEllipse(sx-sr, sy-sr, sr*2, sr*2, pen_stage, QBrush(QColor(155, 89, 182, 180)))
                    sitem.setZValue(5.0)
                    ts = self.viewer.scene.addText("S")
                    ts.setDefaultTextColor(Qt.white)
                    f_s = QFont(); f_s.setBold(True); f_s.setPointSize(8)
                    ts.setFont(f_s)
                    ts.setPos(sx-sr+3, sy-sr-1)
                    ts.setZValue(5.1)
                    
                    ex = d.staging_pose.x + (0.8 if is_editing else 0.5) * math.cos(d.staging_pose.theta)
                    ey = d.staging_pose.y + (0.8 if is_editing else 0.5) * math.sin(d.staging_pose.theta)
                    mex, mey = self.viewer.world_to_map(ex, ey)
                    sline = self.viewer.scene.addLine(sx, sy, mex, mey, pen_stage)
                    sline.setZValue(5.0)
                    self.overlay_items.extend([sitem, ts, sline])
                
        if self.ros_node.robot_pose:
            rx, ry = self.viewer.world_to_map(self.ros_node.robot_pose[0], self.ros_node.robot_pose[1])
            robot_item = self.viewer.scene.addEllipse(rx-6, ry-6, 12, 12, QPen(Qt.darkRed, 2), QBrush(Qt.red))
            robot_item.setZValue(6.0)
            self.overlay_items.append(robot_item)

def main(args=None):
    rclpy.init(args=args)
    
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    
    node = ROSNode()
    
    # Spin ROS node in a separate background thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    win = MainWindow(node)
    win.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
