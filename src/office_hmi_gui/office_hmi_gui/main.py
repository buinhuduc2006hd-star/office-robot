import sys
import os
import math
import threading
import time
import json
import yaml
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QListWidget, QListWidgetItem, QGroupBox,
                             QInputDialog, QMessageBox, QFormLayout, QLineEdit, QSplitter,
                             QScrollArea, QFrame, QComboBox, QTabWidget, QTableWidget,
                             QTableWidgetItem, QHeaderView, QTextEdit, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPointF, QObject
from PyQt5.QtGui import QPixmap, QImage, QColor, QPen, QBrush, QPolygonF, QFont
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene

try:
    from office_hmi_gui.nlp_client import (
        NlpHttpClient, MessageResponse, TaskDTO, BatchDTO, PreviewDTO, TaskSnapshot
    )
except ImportError:
    from .nlp_client import (
        NlpHttpClient, MessageResponse, TaskDTO, BatchDTO, PreviewDTO, TaskSnapshot
    )

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

        # Nạp sẵn cấu hình phòng từ file rooms.yaml để sẵn sàng điều phối
        self.load_initial_rooms_from_config()

        # Keepout Layer Publisher để GUI có thể đồng bộ trực tiếp tới Nav2 Costmap
        self.keepout_pub = self.create_publisher(ActiveKeepoutZones, '/office/keepout_updates', 10)
        
    def load_initial_rooms_from_config(self):
        """Khởi tạo danh sách phòng từ config/rooms.yaml để GUI luôn có sẵn dữ liệu Anchor."""
        paths = [
            "/home/duc/office_robot_ws/config/rooms.yaml",
            os.path.join(os.path.dirname(__file__), "../../../config/rooms.yaml"),
            os.path.join(os.path.expanduser("~"), "office_robot_ws", "config", "rooms.yaml"),
        ]
        for p in paths:
            if os.path.exists(p):
                try:
                    with open(p, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f)
                    if data and "rooms" in data:
                        for rid, rdata in data["rooms"].items():
                            rc = RoomConfig()
                            rc.room_id = str(rid)
                            rc.display_name = rdata.get("display_name", str(rid))
                            rc.description = rdata.get("description", "")
                            anchor = rdata.get("room_anchor") or {}
                            rc.room_anchor.x = float(anchor.get("x", 0.0))
                            rc.room_anchor.y = float(anchor.get("y", 0.0))
                            for ddata in rdata.get("doors", []):
                                dc = DoorConfig()
                                dc.door_id = ddata.get("id", "main_door")
                                dc.p1.x = float(ddata.get("p1", {}).get("x", 0.0))
                                dc.p1.y = float(ddata.get("p1", {}).get("y", 0.0))
                                dc.p2.x = float(ddata.get("p2", {}).get("x", 0.0))
                                dc.p2.y = float(ddata.get("p2", {}).get("y", 0.0))
                                dc.zone_depth = float(ddata.get("zone_depth", 0.3))
                                sp = ddata.get("staging_pose", {})
                                dc.staging_pose.x = float(sp.get("x", 0.0))
                                dc.staging_pose.y = float(sp.get("y", 0.0))
                                dc.staging_pose.theta = float(sp.get("yaw", 0.0))
                                rc.doors.append(dc)
                            self.rooms[str(rid)] = rc
                    self.get_logger().info(f"Đã nạp sẵn {len(self.rooms)} phòng từ {p}")
                    break
                except Exception as e:
                    self.get_logger().debug(f"Không thể nạp cấu hình {p}: {e}")

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
        
        # NLP Client & State
        self.nlp_client = NlpHttpClient(self)
        self.nlp_client.use_mock_mode = True  # Mặc định bật Mock Contract để dùng ngay
        self.nlp_client.response_received.connect(self.on_nlp_response)
        self.nlp_client.request_failed.connect(self.on_nlp_error)
        self.nlp_client.status_updated.connect(self.update_nlp_status)
        self.nlp_client.session_created.connect(self.on_session_created)
        self.nlp_client.health_checked.connect(self.on_health_checked)
        self.current_nlp_batch = None
        self.current_ordered_tasks = []
        
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
            QPushButton:disabled { background-color: #f1f5f9; color: #94a3b8; border: 1px solid #cbd5e1; }
            
            QPushButton#btn_anchor { background-color: #e67e22; color: #ffffff; }
            QPushButton#btn_anchor:hover { background-color: #d35400; }
            QPushButton#btn_anchor:disabled { background-color: #ffedd5; color: #c2410c; border: 1px solid #fed7aa; }
            
            QPushButton#btn_door { background-color: #2980b9; color: #ffffff; }
            QPushButton#btn_door:hover { background-color: #1f618d; }
            QPushButton#btn_door:disabled { background-color: #e0f2fe; color: #0284c7; border: 1px solid #bae6fd; }
            
            QPushButton#btn_staging { background-color: #8e44ad; color: #ffffff; }
            QPushButton#btn_staging:hover { background-color: #71368a; }
            QPushButton#btn_staging:disabled { background-color: #f3e8ff; color: #7e22ce; border: 1px solid #e9d5ff; }
            
            QPushButton#btn_save { background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold; padding: 8px; }
            QPushButton#btn_save:hover { background-color: #219653; }
            QPushButton#btn_save:disabled { background-color: #dcfce7; color: #15803d; border: 1px solid #bbf7d0; font-size: 13px; font-weight: bold; padding: 8px; }
            
            QPushButton#btn_free { background-color: #2ecc71; color: #ffffff; padding: 7px; font-size: 12px; }
            QPushButton#btn_free:hover { background-color: #27ae60; }
            QPushButton#btn_free:disabled { background-color: #dcfce7; color: #166534; border: 1px solid #bbf7d0; padding: 7px; font-size: 12px; }
            
            QPushButton#btn_busy { background-color: #e74c3c; color: #ffffff; padding: 7px; font-size: 12px; }
            QPushButton#btn_busy:hover { background-color: #c0392b; }
            QPushButton#btn_busy:disabled { background-color: #fee2e2; color: #991b1b; border: 1px solid #fecaca; padding: 7px; font-size: 12px; }
            
            QPushButton#btn_delete_room { background-color: #95a5a6; color: #ffffff; font-size: 11px; padding: 5px; }
            QPushButton#btn_delete_room:hover { background-color: #c0392b; }
            QPushButton#btn_delete_room:disabled { background-color: #f1f5f9; color: #94a3b8; border: 1px solid #e2e8f0; font-size: 11px; padding: 5px; }
            
            QPushButton#btn_start_mission { background-color: #16a085; color: #ffffff; font-size: 13px; font-weight: bold; padding: 8px; }
            QPushButton#btn_start_mission:hover { background-color: #1abc9c; }
            QPushButton#btn_start_mission:disabled { background-color: #f1f5f9; color: #94a3b8; border: 1px solid #e2e8f0; font-size: 13px; font-weight: bold; padding: 8px; }
            
            QPushButton#btn_cancel_mission { background-color: #c0392b; color: #ffffff; font-size: 13px; font-weight: bold; padding: 8px; }
            QPushButton#btn_cancel_mission:hover { background-color: #962d22; }
            QPushButton#btn_cancel_mission:disabled { background-color: #fee2e2; color: #b91c1c; border: 1px solid #fecaca; font-size: 13px; font-weight: bold; padding: 8px; }
            
            QLineEdit, QListWidget, QComboBox {
                border: 1px solid #dcdde1; border-radius: 4px; padding: 4px;
            }
            QLineEdit:disabled {
                background-color: #f8fafc; color: #94a3b8;
            }
        """)
        
        self.viewer = MapViewer()
        self.viewer.clicked_signal.connect(self.on_map_clicked)
        self.viewer.dragged_signal.connect(self.on_map_dragged)
        
        left_panel = QWidget()
        vbox = QVBoxLayout(left_panel)
        vbox.setSpacing(12)
        vbox.setContentsMargins(10, 10, 10, 10)
        
        # --- Khung 1: Danh sách phòng ban ---
        grp_manage = QGroupBox("🏢 Danh sách phòng ban")
        lo_manage = QVBoxLayout(grp_manage)
        
        h_add = QHBoxLayout()
        self.edit_new_room = QLineEdit()
        self.edit_new_room.setPlaceholderText("Nhập mã phòng mới (vd: 08)...")
        self.btn_add_room = QPushButton("➕ Tạo phòng")
        self.btn_add_room.setToolTip("Tạo thêm một phòng mới vào hệ thống")
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
        
        # --- Khung 2: Cài đặt vị trí phòng ---
        self.grp_editor = QGroupBox("🛠 Cài đặt vị trí phòng")
        form = QFormLayout(self.grp_editor)
        form.setSpacing(8)
        self.lbl_id = QLabel("-")
        self.lbl_id.setStyleSheet("color: #e67e22; font-weight: bold; font-size: 13px;")
        
        self.edit_name = QLineEdit()
        self.edit_name.setPlaceholderText("Tên phòng (vd: Phòng Kế Toán)...")
        self.edit_desc = QLineEdit()
        self.edit_desc.setPlaceholderText("Ghi chú (vd: Tầng 2, Dãy A)...")
        
        self.btn_anchor = QPushButton("📍 1. Đặt vị trí trong phòng")
        self.btn_anchor.setObjectName("btn_anchor")
        self.btn_anchor.setToolTip("Click một điểm bên trong phòng trên bản đồ")
        self.btn_anchor.clicked.connect(lambda: self.set_mode("ANCHOR"))
        
        self.btn_door = QPushButton("🚪 2. Đánh dấu cửa phòng")
        self.btn_door.setObjectName("btn_door")
        self.btn_door.setToolTip("Click 2 điểm liên tiếp trên bản đồ để đánh dấu 2 bên mép cửa")
        self.btn_door.clicked.connect(lambda: self.set_mode("DOOR_P1"))
        
        self.btn_staging = QPushButton("🚩 3. Đặt chỗ dừng trước cửa")
        self.btn_staging.setObjectName("btn_staging")
        self.btn_staging.setToolTip("Nhấn giữ và kéo chuột ở hành lang trước cửa để đặt chỗ xe đỗ và hướng quay mặt")
        self.btn_staging.clicked.connect(lambda: self.set_mode("STAGING"))
        
        self.btn_save = QPushButton("💾 Lưu cài đặt phòng này")
        self.btn_save.setObjectName("btn_save")
        self.btn_save.setToolTip("Lưu thông tin phòng vào hệ thống")
        self.btn_save.clicked.connect(self.save_room)
        
        form.addRow("Mã phòng:", self.lbl_id)
        form.addRow("Tên phòng:", self.edit_name)
        form.addRow("Ghi chú:", self.edit_desc)
        form.addRow("Bước 1:", self.btn_anchor)
        form.addRow("Bước 2:", self.btn_door)
        form.addRow("Bước 3:", self.btn_staging)
        form.addRow("", self.btn_save)
        
        self.lbl_details = QLabel("<i>Chọn một phòng để xem tiến độ cài đặt.</i>")
        self.lbl_details.setStyleSheet("""
            QLabel {
                background-color: #f8fafc;
                border: 1px solid #e2e8f0;
                border-radius: 5px;
                padding: 6px 8px;
                font-size: 11px;
                color: #1e293b;
            }
        """)
        form.addRow("Tiến độ:", self.lbl_details)
        
        self.is_dirty = False
        self.edit_name.textChanged.connect(self.mark_dirty)
        self.edit_desc.textChanged.connect(self.mark_dirty)
        self.save_completed_signal.connect(self.on_save_completed)
        self.delete_completed_signal.connect(self.on_delete_completed)
        
        vbox.addWidget(self.grp_editor, stretch=0)
        self.grp_editor.setEnabled(False)
        
        # --- Khung 3: Báo bận / Trạng thái hoạt động ---
        self.grp_state = QGroupBox("🔔 Trạng thái phòng (Báo bận)")
        lo_state = QVBoxLayout(self.grp_state)
        lo_state.setSpacing(6)
        
        h_state = QHBoxLayout()
        self.btn_free = QPushButton("🟢 Phòng Rảnh (Mở cửa)")
        self.btn_free.setObjectName("btn_free")
        self.btn_free.setToolTip("Đặt phòng ở trạng thái rảnh, sẵn sàng nhận đồ từ robot")
        self.btn_free.clicked.connect(lambda: self.set_state(0))
        
        self.btn_busy = QPushButton("🔴 Báo Bận (Đang họp / Đóng)")
        self.btn_busy.setObjectName("btn_busy")
        self.btn_busy.setToolTip("Đặt phòng ở trạng thái bận. Robot sẽ tự động bỏ qua phòng này khi giao hàng")
        self.btn_busy.clicked.connect(lambda: self.set_state(1))
        
        h_state.addWidget(self.btn_free)
        h_state.addWidget(self.btn_busy)
        lo_state.addLayout(h_state)
        
        lbl_state_hint = QLabel("💡 <i>Khi báo bận, Robot sẽ tự động bỏ qua phòng này khi giao hàng để không làm phiền.</i>")
        lbl_state_hint.setStyleSheet("color: #64748b; font-size: 11px;")
        lbl_state_hint.setWordWrap(True)
        lo_state.addWidget(lbl_state_hint)
        
        vbox.addWidget(self.grp_state, stretch=0)
        self.grp_state.setEnabled(False)
        
        # --- Khung 4: Điều khiển Robot giao hàng ---
        self.grp_mission = QGroupBox("🚀 Điều khiển Robot giao hàng (Thủ công)")
        lo_mission = QVBoxLayout(self.grp_mission)
        lo_mission.setSpacing(6)

        h_rooms = QHBoxLayout()
        self.edit_mission_rooms = QLineEdit()
        self.edit_mission_rooms.setPlaceholderText("Các phòng cần giao (vd: 01, 02)...")
        self.btn_add_to_mission = QPushButton("+ Thêm phòng chọn")
        self.btn_add_to_mission.setStyleSheet("background-color: #64748b; padding: 5px; font-weight: normal;")
        self.btn_add_to_mission.setToolTip("Thêm phòng đang chọn vào danh sách lộ trình giao hàng")
        self.btn_add_to_mission.clicked.connect(self.add_selected_room_to_mission)
        h_rooms.addWidget(self.edit_mission_rooms, stretch=3)
        h_rooms.addWidget(self.btn_add_to_mission, stretch=2)
        lo_mission.addLayout(h_rooms)

        h_target = QHBoxLayout()
        h_target.addWidget(QLabel("Nơi robot dừng:"))
        self.cmb_target_type = QComboBox()
        self.cmb_target_type.addItems(["🚩 Dừng trước cửa phòng (Khuyên dùng)", "📍 Đi vào bên trong phòng"])
        h_target.addWidget(self.cmb_target_type)
        lo_mission.addLayout(h_target)

        h_btns = QHBoxLayout()
        self.btn_start_mission = QPushButton("🚀 Bắt đầu giao hàng")
        self.btn_start_mission.setObjectName("btn_start_mission")
        self.btn_start_mission.clicked.connect(self.start_mission)

        self.btn_cancel_mission = QPushButton("🛑 Dừng khẩn cấp / Hủy")
        self.btn_cancel_mission.setObjectName("btn_cancel_mission")
        self.btn_cancel_mission.clicked.connect(self.cancel_mission)
        self.btn_cancel_mission.setEnabled(False)

        h_btns.addWidget(self.btn_start_mission)
        h_btns.addWidget(self.btn_cancel_mission)
        lo_mission.addLayout(h_btns)

        self.lbl_mission_status = QLabel("Trạng thái: Robot đang sẵn sàng phục vụ")
        self.lbl_mission_status.setStyleSheet("color: #1e293b; font-weight: bold; font-size: 11px;")
        lo_mission.addWidget(self.lbl_mission_status)

        vbox.addWidget(self.grp_mission, stretch=0)
        
        # --- Khung 5: Nhật ký hoạt động ---
        grp_notify = QGroupBox("📋 Nhật ký hoạt động của Robot")
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
        self.lbl_instruction = QLabel("💡 Hướng dẫn nhanh: Chọn một phòng bên trái để xem vị trí, hoặc nhập mã phòng rồi bấm '➕ Tạo phòng' để thiết lập mới.")
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

        # Thanh chú giải sơ đồ trực quan (Legend)
        legend_frame = QFrame()
        legend_frame.setStyleSheet("""
            QFrame {
                background-color: #ffffff;
                border: 1px solid #dcdde1;
                border-radius: 6px;
            }
            QLabel {
                font-size: 11px;
                color: #2c3e50;
            }
        """)
        legend_lo = QHBoxLayout(legend_frame)
        legend_lo.setContentsMargins(10, 5, 10, 5)
        legend_lo.addWidget(QLabel("<b>Chú thích sơ đồ:</b>"))
        legend_lo.addWidget(QLabel("🔴 <b>Vị trí Robot</b>"))
        legend_lo.addWidget(QLabel("📍 <b>Trong phòng</b>"))
        legend_lo.addWidget(QLabel("🚪 <b>Cửa ra vào</b>"))
        legend_lo.addWidget(QLabel("🚩 <b>Điểm đỗ trước cửa</b>"))
        legend_lo.addWidget(QLabel("⛔ <b>Khu vực cấm đi (Phòng bận)</b>"))
        legend_lo.addStretch()
        map_lo.addWidget(legend_frame)

        splitter.addWidget(map_frame)
        splitter.setSizes([420, 880])

        # Main Tab Widget: Tab 1 = Quản lý phòng & Map, Tab 2 = Chat & NLP
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #dcdde1;
                background-color: #f5f6fa;
            }
            QTabBar::tab {
                background: #e4e7eb;
                color: #2f3640;
                padding: 10px 22px;
                font-weight: bold;
                font-size: 13px;
                border: 1px solid #dcdde1;
                border-bottom: none;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 4px;
            }
            QTabBar::tab:selected {
                background: white;
                color: #2980b9;
                border-bottom: 3px solid #3498db;
            }
            QTabBar::tab:hover:!selected {
                background: #dfe4ea;
            }
        """)

        # Tab 1: Sơ đồ văn phòng & Cài đặt phòng
        self.tabs.addTab(splitter, "🏢 Sơ đồ văn phòng & Cài đặt phòng")

        # Tab 2: Chat & NLP
        tab_chat = self.create_chat_tab()
        self.tabs.addTab(tab_chat, "💬 Trợ lý hội thoại & Giao hàng (AI Chat)")

        self.setCentralWidget(self.tabs)
        
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
            self.lbl_details.setText("<i>Chưa chọn phòng nào</i>")
            return
        r = self.editing_room
        
        # 1. Kiểm tra vị trí trong phòng (Anchor)
        has_anchor = (abs(r.room_anchor.x) > 1e-4 or abs(r.room_anchor.y) > 1e-4)
        if has_anchor:
            s_anchor = "<b>📍 1. Vị trí trong phòng:</b> <span style='color: #27ae60; font-weight: bold;'>✔ Đã đặt</span>"
        else:
            s_anchor = "<b>📍 1. Vị trí trong phòng:</b> <span style='color: #e67e22;'>⚠️ Chưa đặt</span>"

        # 2. Kiểm tra cửa phòng và chỗ dừng
        has_door = bool(r.doors and len(r.doors) > 0)
        has_staging = False
        if has_door:
            d = r.doors[0]
            d_len = math.hypot(d.p2.x - d.p1.x, d.p2.y - d.p1.y)
            s_door = f"<b>🚪 2. Cửa ra vào:</b> <span style='color: #27ae60; font-weight: bold;'>✔ Đã đánh dấu (rộng {d_len:.1f}m)</span>"
            if abs(d.staging_pose.x) > 1e-4 or abs(d.staging_pose.y) > 1e-4 or abs(d.staging_pose.theta) > 1e-4:
                has_staging = True
                s_staging = "<b>🚩 3. Chỗ dừng trước cửa:</b> <span style='color: #27ae60; font-weight: bold;'>✔ Đã đặt</span>"
            else:
                s_staging = "<b>🚩 3. Chỗ dừng trước cửa:</b> <span style='color: #e67e22;'>⚠️ Chưa đặt</span>"
        else:
            s_door = "<b>🚪 2. Cửa ra vào:</b> <span style='color: #e67e22;'>⚠️ Chưa đánh dấu</span>"
            s_staging = "<b>🚩 3. Chỗ dừng trước cửa:</b> <span style='color: #95a5a6;'>⏳ Cần đặt cửa trước</span>"

        if has_anchor and has_door and has_staging:
            status_banner = "<span style='color: #27ae60; font-weight: bold;'>🎉 Phòng đã sẵn sàng đón Robot!</span>"
        else:
            status_banner = "<span style='color: #d35400; font-size: 10px;'><i>👉 Vui lòng hoàn thành 3 bước ở trên</i></span>"

        html = f"""
        <div style='line-height: 140%;'>
            {s_anchor}<br>
            {s_door}<br>
            {s_staging}<br>
            <hr style='border: 0; border-top: 1px solid #e2e8f0; margin: 3px 0;'>
            {status_banner}
        </div>
        """
        self.lbl_details.setText(html)
        
    def set_mode(self, mode):
        self.viewer.mode = mode
        if mode == "DOOR_P1":
            self.viewer.temp_p1 = None
        self.statusBar().showMessage(f"Chế độ: {mode}")
        
        if mode == "ANCHOR":
            self.lbl_instruction.setText("📍 BƯỚC 1: ĐẶT VỊ TRÍ TRONG PHÒNG ➜ Click chuột lên 1 vị trí bên trong phòng trên sơ đồ.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #d35400; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #f39c12;
            """)
            self.log_msg("📍 Đang thực hiện Bước 1: Hãy click chuột vào vị trí bên trong phòng trên sơ đồ...", Qt.darkBlue)
        elif mode == "DOOR_P1":
            self.lbl_instruction.setText("🚪 BƯỚC 2: ĐÁNH DẤU CỬA (Lần 1/2) ➜ Click chuột vào mép thứ nhất của cửa phòng...")
            self.lbl_instruction.setStyleSheet("""
                background-color: #2980b9; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #3498db;
            """)
            self.log_msg("🚪 Đang thực hiện Bước 2: Click vào mép thứ nhất của cửa phòng...", Qt.darkBlue)
        elif mode == "DOOR_P2":
            self.lbl_instruction.setText("🚪 BƯỚC 2: ĐÁNH DẤU CỬA (Lần 2/2) ➜ Mép thứ nhất đã nhận! Click tiếp vào mép bên kia của cửa...")
            self.lbl_instruction.setStyleSheet("""
                background-color: #16a085; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #1abc9c;
            """)
            self.log_msg("🚪 Đã nhận mép thứ nhất! Click tiếp vào mép thứ hai của cửa...", Qt.darkBlue)
        elif mode == "STAGING":
            self.lbl_instruction.setText("🚩 BƯỚC 3: ĐẶT CHỖ DỪNG NGOÀI CỬA ➜ Nhấn giữ chuột tại hành lang và KÉO hướng về phía phòng...")
            self.lbl_instruction.setStyleSheet("""
                background-color: #8e44ad; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #9b59b6;
            """)
            self.log_msg("🚩 Đang thực hiện Bước 3: Nhấn giữ và KÉO chuột ngoài hành lang để đặt chỗ xe đỗ và hướng xe...", Qt.darkBlue)
        else: # IDLE
            self.lbl_instruction.setText("💡 Sẵn sàng: Chọn phòng bên trái để xem/cài đặt, hoặc nhập mã phòng vào ô giao hàng bên dưới.")
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
                QMessageBox.warning(self, "Trùng mã phòng", f"Mã phòng '{rid}' đã tồn tại trong danh sách! Vui lòng nhập mã khác.")
                return
            r = RoomConfig()
            r.room_id = rid
            self.ros_node.rooms[rid] = r
            self.on_room_selected(list(self.ros_node.rooms.keys()).index(rid))
            self.log_msg(f"➕ Đã thêm phòng mới: {rid}", Qt.blue)
            self.edit_new_room.clear()

    def on_map_clicked(self, wx, wy):
        if not self.editing_room: return
        room_title = self.editing_room.display_name if self.editing_room.display_name else self.current_room
        
        if self.viewer.mode == "ANCHOR":
            self.editing_room.room_anchor.x = wx
            self.editing_room.room_anchor.y = wy
            self.set_mode("IDLE")
            self.mark_dirty()
            self.update_details_panel()
            self.lbl_instruction.setText(f"✅ Đã đặt vị trí bên trong phòng '{room_title}'! Hãy bấm Bước 2 để đánh dấu cửa.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.log_msg(f"✅ Đã chọn vị trí bên trong phòng '{room_title}' thành công.", Qt.darkGreen)
            self.draw_overlays()
            QMessageBox.information(
                self, 
                "Cài đặt vị trí phòng", 
                f"✅ Đã lưu vị trí bên trong phòng '{room_title}' thành công!\n\n"
                f"Robot sẽ sử dụng vị trí này khi được giao nhiệm vụ đi vào tận trong phòng.\n\n"
                f"👉 Bước tiếp theo: Bấm nút '🚪 2. Đánh dấu cửa phòng' để xác định cửa ra vào."
            )
            
        elif self.viewer.mode == "DOOR_P1":
            self.viewer.temp_p1 = (wx, wy)
            self.set_mode("DOOR_P2")
            self.log_msg(f"📍 Đã nhận mép cửa thứ nhất. Mời bạn click tiếp mép cửa thứ hai...", Qt.blue)
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
            self.lbl_instruction.setText(f"✅ Đã đánh dấu cửa phòng (Rộng {d_len:.2f}m)! Bấm Bước 3 để đặt chỗ dừng đón robot.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.log_msg(f"✅ Đã đánh dấu cửa ra vào '{room_title}' (rộng {d_len:.2f}m).", Qt.darkGreen)
            self.draw_overlays()
            QMessageBox.information(
                self,
                "Đánh dấu cửa phòng",
                f"✅ Đã đánh dấu cửa cho phòng '{room_title}' thành công!\n\n"
                f"• Bề rộng cửa đo được: {d_len:.2f} mét\n\n"
                f"Vị trí cửa đã hiển thị trên sơ đồ.\n"
                f"👉 Bước tiếp theo: Bấm nút '🚩 3. Đặt chỗ dừng trước cửa' để chọn nơi robot dừng lại khi mang đồ đến."
            )
            
    def on_map_dragged(self, sx, sy, ex, ey):
        if not self.editing_room: return
        room_title = self.editing_room.display_name if self.editing_room.display_name else self.current_room
        
        if self.viewer.mode == "STAGING":
            if not self.editing_room.doors:
                QMessageBox.warning(self, "Chưa đánh dấu cửa", "Vui lòng bấm '🚪 2. Đánh dấu cửa phòng' trước khi đặt chỗ dừng đón robot!")
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
            self.lbl_instruction.setText(f"✅ Đã đặt chỗ dừng trước cửa phòng '{room_title}'! Bấm '💾 Lưu cài đặt phòng này' để hoàn tất.")
            self.lbl_instruction.setStyleSheet("""
                background-color: #27ae60; color: #ffffff; font-size: 13px; font-weight: bold;
                padding: 8px 14px; border-radius: 6px; border-left: 5px solid #2ecc71;
            """)
            self.log_msg(f"✅ Đã thiết lập xong chỗ dừng ngoài cửa cho '{room_title}'.", Qt.darkGreen)
            self.draw_overlays()
            QMessageBox.information(
                self,
                "Chỗ dừng đón Robot",
                f"✅ Đã đặt chỗ dừng đón Robot cho phòng '{room_title}' thành công!\n\n"
                f"Khi mang tài liệu hoặc đồ đạc đến phòng, Robot sẽ tự động di chuyển đến điểm này ở hành lang và quay mặt về phía phòng để mọi người lấy đồ.\n\n"
                f"👉 Cuối cùng: Hãy bấm nút '💾 Lưu cài đặt phòng này' để hoàn tất cấu hình!"
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
        raw_id = str(raw_id).strip()
        if not raw_id:
            return None
        rooms = self.ros_node.rooms

        # 1. Khớp chính xác ID trong danh sách phòng ROS
        if raw_id in rooms:
            return raw_id

        # 2. Khớp dạng số ('1' -> '01', '02', ...)
        if raw_id.isdigit():
            val = int(raw_id)
            for cand in [f"{val:02d}", str(val), f"{val:03d}", f"room_{val:02d}", f"room_{val:03d}"]:
                if cand in rooms:
                    return cand

        # 3. Chuẩn hóa tên (bỏ dấu tiếng Việt, loại bỏ từ 'phòng' / 'phong')
        def norm(s):
            s = str(s).lower().replace("phòng", "").replace("phong", "").strip()
            repl = [("ă", "a"), ("â", "a"), ("á", "a"), ("à", "a"), ("ả", "a"), ("ã", "a"), ("ạ", "a"),
                    ("ê", "e"), ("é", "e"), ("è", "e"), ("ẻ", "e"), ("ẽ", "e"), ("ẹ", "e"),
                    ("í", "i"), ("ì", "i"), ("ỉ", "i"), ("ĩ", "i"), ("ị", "i"),
                    ("ô", "o"), ("ơ", "o"), ("ó", "o"), ("ò", "o"), ("ỏ", "o"), ("õ", "o"), ("ọ", "o"),
                    ("ư", "u"), ("ú", "u"), ("ù", "u"), ("ủ", "u"), ("ũ", "u"), ("ụ", "u"),
                    ("đ", "d")]
            for a, b in repl:
                s = s.replace(a, b)
            return s.strip()

        norm_raw = norm(raw_id)
        for rid, r in rooms.items():
            if norm(rid) == norm_raw or norm(r.display_name) == norm_raw:
                return rid
            if norm_raw and (norm_raw in norm(r.display_name) or norm(r.display_name) in norm_raw):
                return rid

        # 4. Fallback qua to_canonical_room_id của OfflineContractMock
        try:
            from office_hmi_gui.nlp_client import OfflineContractMock
            canon = OfflineContractMock.to_canonical_room_id(raw_id)
            if canon and canon in rooms:
                return canon
            if canon:
                return canon
        except Exception:
            pass

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
            self.mission_force_anchor = False
            self.mission_force_staging = False
            self.btn_start_mission.setEnabled(True)
            self.btn_cancel_mission.setEnabled(False)
            return

        current_rid = self.pending_mission_rooms.pop(0)
        room = self.ros_node.rooms.get(current_rid)
        room_name = room.display_name if room else f"phòng {current_rid}"

        # KIỂM TRA TRẠNG THÁI BUSY
        state = self.ros_node.room_states.get(current_rid, 0)
        if state == 1:
            self.log_msg(
                f"⚠️ Phòng [{current_rid} - {room_name}] đang BẬN (BUSY - Cửa khóa/Keepout đang bật)! ➔ TỰ ĐỘNG BỎ QUA và chuyển sang phòng kế tiếp...",
                QColor(211, 84, 0)
            )
            self.lbl_mission_status.setText(f"Phòng {current_rid} bận ➔ Chuyển tiếp...")
            QTimer.singleShot(2000, self.process_next_mission_room)
            return

        # XÁC ĐỊNH TỌA ĐỘ ĐÍCH (ƯU TIÊN ĐIỂM STAGING POSE KHI ĐIỀU PHỐI TỪ CHAT NLP HOẶC CHỌN STAGING POSE)
        force_staging = getattr(self, "mission_force_staging", False)
        force_anchor = getattr(self, "mission_force_anchor", False)
        target_is_staging = (self.cmb_target_type.currentIndex() == 0 or force_staging) and not force_anchor
        tx, ty, tyaw = None, None, 0.0
        target_desc = ""

        if target_is_staging:
            # GIAO ĐẾN ĐIỂM CHỜ NGOÀI CỬA (STAGING POSE)
            if room and room.doors and (room.doors[0].staging_pose.x != 0.0 or room.doors[0].staging_pose.y != 0.0):
                d = room.doors[0]
                tx, ty, tyaw = d.staging_pose.x, d.staging_pose.y, d.staging_pose.theta
                target_desc = f"Điểm chờ ngoài cửa Staging ({tx:.2f}, {ty:.2f})"
            else:
                # Tra cứu từ config/rooms.yaml qua OfflineContractMock
                try:
                    from office_hmi_gui.nlp_client import OfflineContractMock
                    mc = OfflineContractMock.get_room_staging(current_rid)
                    if mc:
                        tx, ty, tyaw = mc["x"], mc["y"], mc.get("yaw", 0.0)
                        target_desc = f"Điểm chờ ngoài cửa Staging ({tx:.2f}, {ty:.2f}) [Từ config/rooms.yaml]"
                except Exception:
                    pass

            if tx is None and room and hasattr(room, "room_anchor") and (room.room_anchor.x != 0.0 or room.room_anchor.y != 0.0):
                tx, ty, tyaw = room.room_anchor.x, room.room_anchor.y, 0.0
                target_desc = f"Tâm phòng Anchor ({tx:.2f}, {ty:.2f}) [Fallback]"
        else:
            # GIAO TRỰC TIẾP ĐẾN ĐIỂM ANCHOR TRONG PHÒNG
            if room and hasattr(room, "room_anchor") and (room.room_anchor.x != 0.0 or room.room_anchor.y != 0.0):
                tx, ty, tyaw = room.room_anchor.x, room.room_anchor.y, 0.0
                target_desc = f"Trong phòng Anchor ({tx:.2f}, {ty:.2f})"
            else:
                # Tra cứu từ config/rooms.yaml qua OfflineContractMock
                try:
                    from office_hmi_gui.nlp_client import OfflineContractMock
                    mc = OfflineContractMock.get_room_anchor(current_rid)
                    if mc:
                        tx, ty, tyaw = mc["x"], mc["y"], 0.0
                        target_desc = f"Trong phòng Anchor ({tx:.2f}, {ty:.2f}) [Từ config/rooms.yaml]"
                except Exception:
                    pass

            if tx is None and room and room.doors and (room.doors[0].staging_pose.x != 0.0 or room.doors[0].staging_pose.y != 0.0):
                d = room.doors[0]
                tx, ty, tyaw = d.staging_pose.x, d.staging_pose.y, d.staging_pose.theta
                target_desc = f"Điểm chờ ngoài cửa ({tx:.2f}, {ty:.2f}) [Fallback]"

        if tx is None:
            self.log_msg(f"❌ Phòng [{current_rid}] chưa được cài đặt tọa độ Anchor hoặc Staging! Bỏ qua.", Qt.red)
            QTimer.singleShot(1000, self.process_next_mission_room)
            return

        self.current_mission_room_id = current_rid
        self.log_msg(f"🚗 [Nhiệm vụ] Phòng [{current_rid} - {room_name}] RẢNH. Đang di chuyển đến {target_desc}...", Qt.blue)
        self.lbl_mission_status.setText(f"Đang đi đến phòng {current_rid}...")

        # Gửi lệnh Nav2
        def on_result(success, message):
            self.sig_goal_done.emit(success, message)

        ok, err_msg = self.ros_node.send_nav_goal(tx, ty, tyaw, result_callback=on_result)
        if not ok:
            self.log_msg(f"❌ Lỗi gửi Nav2: {err_msg}", Qt.red)
            self.lbl_mission_status.setText("Lỗi kết nối Nav2!")
            self.is_mission_running = False
            self.mission_force_anchor = False
            self.mission_force_staging = False
            self.btn_start_mission.setEnabled(True)
            self.btn_cancel_mission.setEnabled(False)

    def on_nav_goal_completed(self, success, message):
        if not self.is_mission_running:
            return

        rid = getattr(self, 'current_mission_room_id', 'Unknown')
        if success:
            self.log_msg(f"✅ Đã đến điểm giao phòng [{rid}] an toàn! Dừng phục vụ 3 giây trước khi đi phòng tiếp theo...", Qt.darkGreen)
            self.lbl_mission_status.setText(f"Đã đến phòng {rid}. Đang chờ 3s...")
            QTimer.singleShot(3000, self.process_next_mission_room)
        else:
            self.log_msg(f"⚠️ Di chuyển đến phòng [{rid}] không thành công ({message})! Chuyển sang phòng tiếp theo...", Qt.darkYellow)
            self.lbl_mission_status.setText(f"Thất bại phòng {rid} ➔ Chuyển tiếp...")
            QTimer.singleShot(2000, self.process_next_mission_room)

    def cancel_mission(self):
        self.is_mission_running = False
        self.mission_force_anchor = False
        self.mission_force_staging = False
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

    # ========================================================
    # Chat & NLP Methods (Chuẩn hóa Contract v1 & Dễ thao tác)
    # ========================================================
    def create_chat_tab(self):
        """Tạo tab Chat & NLP hội thoại với Robot và trích xuất task, batch trực quan."""
        chat_widget = QWidget()
        main_layout = QHBoxLayout(chat_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(14)

        splitter = QSplitter(Qt.Horizontal)

        # ----------------------------------------------------
        # BÊN TRÁI: Khung hội thoại Chatbot & Nhập tin nhắn
        # ----------------------------------------------------
        left_box = QWidget()
        left_lo = QVBoxLayout(left_box)
        left_lo.setContentsMargins(0, 0, 0, 0)
        left_lo.setSpacing(8)

        # Header bên trái
        h_left_header = QHBoxLayout()
        header_lbl = QLabel("💬 Hội thoại & Ra lệnh NLP (Robot Assistant)")
        header_lbl.setStyleSheet("font-size: 15px; font-weight: bold; color: #2980b9;")
        h_left_header.addWidget(header_lbl)
        h_left_header.addStretch()

        self.lbl_session_info = QLabel("Session: mock_default")
        self.lbl_session_info.setStyleSheet("color: #7f8c8d; font-size: 11px; background: #ecf0f1; padding: 3px 8px; border-radius: 10px;")
        h_left_header.addWidget(self.lbl_session_info)
        left_lo.addLayout(h_left_header)

        # Khung lịch sử tin nhắn
        self.txt_chat_history = QTextEdit()
        self.txt_chat_history.setReadOnly(True)
        self.txt_chat_history.setStyleSheet("""
            QTextEdit {
                background-color: #ffffff;
                color: #2f3640;
                border: 1px solid #dcdde1;
                border-radius: 8px;
                padding: 12px;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 13px;
            }
        """)
        left_lo.addWidget(self.txt_chat_history, stretch=1)

        # Gợi ý câu mẫu nhanh theo 2 nhóm rõ ràng
        grp_chips = QGroupBox("⚡ Lệnh mẫu Contract (Bấm là gửi ngay)")
        grp_chips.setStyleSheet("""
            QGroupBox {
                font-size: 12px;
                font-weight: bold;
                color: #2c3e50;
                border: 1px solid #e1e8ed;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #fafbfc;
            }
        """)
        lo_chips_box = QVBoxLayout(grp_chips)
        lo_chips_box.setSpacing(6)

        # Hàng 1: Giao 1 phòng đơn lẻ theo tất cả các phòng đã cấu hình (config/rooms.yaml)
        h_row1 = QHBoxLayout()
        sample_prompts_row1 = [
            ("📄 P1 (phong1)", "Giao tài liệu đến phòng 1."),
            ("📄 P2 (phong2)", "Giao tài liệu đến phòng 2."),
            ("📑 P3 (phong3)", "Giao văn bản đến phòng 3."),
            ("📦 P4 (phong4)", "Giao bưu phẩm đến phòng 4."),
            ("📊 P5 (phong5)", "Giao báo cáo đến phòng 5."),
            ("☕ P6 (Giải lao)", "Giao cà phê đến phòng giải lao."),
            ("🍱 P7 (Phòng ăn)", "Giao cơm trưa đến phòng ăn."),
        ]
        for label, text in sample_prompts_row1:
            btn = QPushButton(label)
            btn.setToolTip(f"Gửi: '{text}'")
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #ffffff; color: #1e293b;
                    border: 1px solid #cbd5e1; border-radius: 12px;
                    padding: 4px 8px; font-size: 11px; font-weight: normal;
                }
                QPushButton:hover {
                    background-color: #ebf5fb; color: #2980b9; border-color: #3498db;
                }
            """)
            btn.clicked.connect(lambda _, t=text: self.on_quick_prompt_clicked(t))
            h_row1.addWidget(btn)
        h_row1.addStretch()
        lo_chips_box.addLayout(h_row1)

        # Hàng 2: Lộ trình đa phòng & Thứ tự ưu tiên
        h_row2 = QHBoxLayout()
        sample_prompts_row2 = [
            ("📦 2 Phòng (P2 & P5)", "Giao hợp đồng đến phòng 2 và báo cáo đến phòng 5."),
            ("⭐ 3 Phòng (Ưu tiên P7)", "Giao hợp đồng đến phòng 2, báo cáo đến phòng 5 và hồ sơ đến phòng 7. Ưu tiên hồ sơ trước."),
            ("🚀 4 Phòng (P1, P3, P4, P6)", "Giao vật phẩm đến phòng 1, phòng 3, phòng 4 và phòng giải lao."),
            ("🔒 Giữ đúng toàn bộ thứ tự", "Giao hợp đồng đến phòng 2 và báo cáo đến phòng 5. Bắt buộc giữ toàn bộ thứ tự."),
        ]
        for label, text in sample_prompts_row2:
            btn = QPushButton(label)
            btn.setToolTip(f"Gửi: '{text}'")
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #f8fafc; color: #0f172a;
                    border: 1px solid #cbd5e1; border-radius: 12px;
                    padding: 4px 8px; font-size: 11px; font-weight: 500;
                }
                QPushButton:hover {
                    background-color: #e0f2fe; color: #0369a1; border-color: #38bdf8;
                }
            """)
            btn.clicked.connect(lambda _, t=text: self.on_quick_prompt_clicked(t))
            h_row2.addWidget(btn)
        h_row2.addStretch()
        lo_chips_box.addLayout(h_row2)

        # Hàng 3: Kịch bản làm rõ (Clarification), xung đột (Conflict) & phản hồi nhanh
        h_row3 = QHBoxLayout()
        sample_prompts_row3 = [
            ("❓ Phòng 8 lạ (Chưa config)", "Giao hợp đồng đến phòng 2 và báo cáo đến phòng 8."),
            ("⚡ Xung đột thứ tự", "Giao đúng thứ tự báo cáo đến phòng 5 rồi hợp đồng đến phòng 2, nhưng ưu tiên hợp đồng cao hơn."),
            ("👉 'Phòng 5.'", "Phòng 5."),
            ("🎖️ 'theo mức ưu tiên'", "theo mức ưu tiên"),
            ("🔄 'giữ đúng thứ tự'", "giữ đúng thứ tự"),
            ("🛑 'Hủy'", "Hủy"),
        ]
        for label, text in sample_prompts_row3:
            btn = QPushButton(label)
            btn.setToolTip(f"Gửi: '{text}'")
            bg = "#fff7ed" if label.startswith("👉") else ("#f5f3ff" if label.startswith("🎖️") or label.startswith("🔄") else ("#fef2f2" if label.startswith("🛑") else "#ffffff"))
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {bg}; color: #334155;
                    border: 1px solid #cbd5e1; border-radius: 12px;
                    padding: 4px 8px; font-size: 11px; font-weight: normal;
                }}
                QPushButton:hover {{
                    background-color: #e2e8f0; color: #0f172a; border-color: #94a3b8;
                }}
            """)
            btn.clicked.connect(lambda _, t=text: self.on_quick_prompt_clicked(t))
            h_row3.addWidget(btn)
        h_row3.addStretch()
        lo_chips_box.addLayout(h_row3)

        left_lo.addWidget(grp_chips)

        # Khung nhập câu lệnh và nút gửi
        input_lo = QHBoxLayout()
        self.txt_chat_input = QLineEdit()
        self.txt_chat_input.setPlaceholderText("Nhập câu lệnh tự nhiên (Ví dụ: Giao hợp đồng đến phòng 2 và báo cáo đến phòng 5)...")
        self.txt_chat_input.setStyleSheet("""
            QLineEdit {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 9px 12px;
                font-size: 13px;
            }
            QLineEdit:focus {
                border: 1px solid #3498db;
                background-color: #fafbfc;
            }
        """)
        self.txt_chat_input.returnPressed.connect(self.on_send_chat)

        self.btn_send_chat = QPushButton("Gửi 🚀")
        self.btn_send_chat.setStyleSheet("""
            QPushButton {
                background-color: #2980b9; color: white;
                font-weight: bold; border-radius: 6px;
                padding: 9px 18px; font-size: 13px;
            }
            QPushButton:hover { background-color: #1f618d; }
            QPushButton:disabled { background-color: #bdc3c7; color: #7f8c8d; }
        """)
        self.btn_send_chat.clicked.connect(self.on_send_chat)

        self.btn_clear_chat = QPushButton("🗑")
        self.btn_clear_chat.setToolTip("Xóa lịch sử trò chuyện")
        self.btn_clear_chat.setStyleSheet("""
            QPushButton {
                background-color: #ecf0f1; color: #7f8c8d;
                border: 1px solid #bdc3c7; border-radius: 6px;
                padding: 9px 12px; font-size: 13px;
            }
            QPushButton:hover { background-color: #e74c3c; color: white; border-color: #c0392b; }
        """)
        self.btn_clear_chat.clicked.connect(self.on_clear_chat)

        input_lo.addWidget(self.txt_chat_input, stretch=1)
        input_lo.addWidget(self.btn_send_chat)
        input_lo.addWidget(self.btn_clear_chat)
        left_lo.addLayout(input_lo)

        splitter.addWidget(left_box)

        # ----------------------------------------------------
        # BÊN PHẢI: Trực quan hóa Contract, Trạng thái & Bảng Batch
        # ----------------------------------------------------
        right_box = QWidget()
        right_lo = QVBoxLayout(right_box)
        right_lo.setContentsMargins(0, 0, 0, 0)
        right_lo.setSpacing(10)

        # 1. Cấu hình & Chế độ Chạy (Mock vs Live HTTP)
        grp_endpoint = QGroupBox("⚙️ Cấu hình NLP & Chế độ Chạy")
        grp_endpoint_lo = QVBoxLayout(grp_endpoint)
        grp_endpoint_lo.setSpacing(6)

        # Toggle Mock Mode
        self.chk_mock_mode = QCheckBox("🔘 Bật Chế độ Giả lập Offline (Mock Contract - Test ngay không cần Server)")
        self.chk_mock_mode.setChecked(True)
        self.chk_mock_mode.setStyleSheet("font-weight: bold; color: #27ae60; font-size: 12px;")
        self.chk_mock_mode.toggled.connect(self.on_toggle_mock_mode)
        grp_endpoint_lo.addWidget(self.chk_mock_mode)

        h_url = QHBoxLayout()
        h_url.addWidget(QLabel("URL API:"))
        self.txt_nlp_url = QLineEdit("http://localhost:8000")
        self.txt_nlp_url.setStyleSheet("padding: 5px; background-color: #fff; border: 1px solid #cbd5e1; border-radius: 4px; font-size: 12px;")
        h_url.addWidget(self.txt_nlp_url, stretch=1)

        self.btn_test_nlp = QPushButton("🔌 Kiểm tra /health")
        self.btn_test_nlp.setStyleSheet("padding: 5px 10px; background-color: #7f8c8d; color: white; border-radius: 4px; font-weight: bold; font-size: 11px;")
        self.btn_test_nlp.clicked.connect(self.on_test_nlp_connection)
        h_url.addWidget(self.btn_test_nlp)

        self.btn_new_session = QPushButton("🔄 Phiên mới")
        self.btn_new_session.setStyleSheet("padding: 5px 10px; background-color: #34495e; color: white; border-radius: 4px; font-weight: bold; font-size: 11px;")
        self.btn_new_session.clicked.connect(self.on_create_new_session)
        h_url.addWidget(self.btn_new_session)
        grp_endpoint_lo.addLayout(h_url)

        self.lbl_nlp_status = QLabel("Trạng thái: ● Đang chạy Giả lập Offline (Mock Contract v1)")
        self.lbl_nlp_status.setStyleSheet("color: #27ae60; font-size: 12px; font-weight: 500;")
        grp_endpoint_lo.addWidget(self.lbl_nlp_status)

        right_lo.addWidget(grp_endpoint)

        # 2. Thẻ Trạng thái Hội thoại (Dialogue Status Badge)
        grp_status = QGroupBox("🎯 Trạng thái Hợp đồng Hội thoại (Dialogue Status)")
        grp_status_lo = QVBoxLayout(grp_status)
        grp_status_lo.setSpacing(6)

        self.lbl_task_badge = QLabel("⚪ CHƯA CÓ NHIỆM VỤ")
        self.lbl_task_badge.setAlignment(Qt.AlignCenter)
        self.lbl_task_badge.setStyleSheet("""
            QLabel {
                background-color: #f1f2f6;
                color: #57606f;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border-radius: 6px;
                border: 1px solid #dcdde1;
            }
        """)
        grp_status_lo.addWidget(self.lbl_task_badge)

        self.lbl_status_hint = QLabel("Hãy chọn hoặc nhập câu lệnh giao tài liệu để quan sát.")
        self.lbl_status_hint.setStyleSheet("color: #7f8c8d; font-size: 11px; font-style: italic; padding: 0 4px;")
        grp_status_lo.addWidget(self.lbl_status_hint)

        # Hộp nút lựa chọn nhanh khi phát hiện DECISION_READY (Xung đột)
        self.box_decision_buttons = QWidget()
        box_dec_lo = QHBoxLayout(self.box_decision_buttons)
        box_dec_lo.setContentsMargins(0, 0, 0, 0)
        box_dec_lo.setSpacing(8)

        self.btn_choice_priority = QPushButton("🎖️ Chọn: Theo mức ưu tiên")
        self.btn_choice_priority.setStyleSheet("background-color: #8e44ad; color: white; padding: 6px 12px; border-radius: 4px; font-weight: bold;")
        self.btn_choice_priority.clicked.connect(lambda: self.on_resolve_decision("theo mức ưu tiên"))
        box_dec_lo.addWidget(self.btn_choice_priority)

        self.btn_choice_strict = QPushButton("🔄 Chọn: Giữ đúng thứ tự")
        self.btn_choice_strict.setStyleSheet("background-color: #2980b9; color: white; padding: 6px 12px; border-radius: 4px; font-weight: bold;")
        self.btn_choice_strict.clicked.connect(lambda: self.on_resolve_decision("giữ đúng thứ tự"))
        box_dec_lo.addWidget(self.btn_choice_strict)

        self.box_decision_buttons.setVisible(False)
        grp_status_lo.addWidget(self.box_decision_buttons)

        right_lo.addWidget(grp_status)

        # 3. Phản hồi văn bản từ Trợ lý
        grp_resp = QGroupBox("📝 Phản hồi từ Trợ lý (Assistant Reply)")
        grp_resp_lo = QVBoxLayout(grp_resp)
        self.txt_nlp_response_view = QTextEdit()
        self.txt_nlp_response_view.setReadOnly(True)
        self.txt_nlp_response_view.setMaximumHeight(70)
        self.txt_nlp_response_view.setStyleSheet("""
            QTextEdit {
                background-color: #f8fafc;
                color: #1e293b;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 8px;
                font-size: 12px;
                font-weight: 500;
            }
        """)
        self.txt_nlp_response_view.setPlaceholderText("Phản hồi văn bản từ NLP API sẽ hiển thị ở đây...")
        grp_resp_lo.addWidget(self.txt_nlp_response_view)
        right_lo.addWidget(grp_resp)

        # 4. Danh sách Batch & Thứ tự thực hiện (Order Preview)
        grp_batch = QGroupBox("📍 Lộ trình Giao nhận & Thứ tự Thực thi (Batch & Order Preview)")
        grp_batch_lo = QVBoxLayout(grp_batch)
        grp_batch_lo.setSpacing(6)

        h_batch_header = QHBoxLayout()
        self.lbl_batch_count = QLabel("📊 Điểm dừng: 0 | Chính sách: N/A")
        self.lbl_batch_count.setStyleSheet("font-weight: bold; color: #2c3e50; font-size: 12px;")
        h_batch_header.addWidget(self.lbl_batch_count)

        self.lbl_preview_notice = QLabel("💡 Thứ tự bên dưới đã được tối ưu theo Preview")
        self.lbl_preview_notice.setStyleSheet("color: #0284c7; font-size: 11px; font-style: italic;")
        h_batch_header.addStretch()
        h_batch_header.addWidget(self.lbl_preview_notice)
        grp_batch_lo.addLayout(h_batch_header)

        # Table với 6 cột chi tiết
        self.table_batch = QTableWidget(0, 6)
        self.table_batch.setHorizontalHeaderLabels([
            "Thứ tự (#)", "Điểm đến", "Tài liệu / Vật phẩm", "Ưu tiên", "Ràng buộc", "Tọa độ Staging Pose (Điểm giao)"
        ])
        header = self.table_batch.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_batch.verticalHeader().setVisible(False)
        self.table_batch.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                color: #1e293b;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                gridline-color: #f1f5f9;
                font-size: 12px;
            }
            QHeaderView::section {
                background-color: #f8fafc;
                color: #334155;
                font-weight: bold;
                padding: 6px;
                border: 1px solid #e2e8f0;
            }
        """)
        grp_batch_lo.addWidget(self.table_batch, stretch=1)

        # Nút Hành động Điều phối trực tiếp sang Robot Nav2
        h_dispatch = QHBoxLayout()
        self.btn_dispatch_mission = QPushButton("🚀 Thực thi Lộ trình trên Robot (Gửi tới Nav2)")
        self.btn_dispatch_mission.setEnabled(False)
        self.btn_dispatch_mission.setStyleSheet("""
            QPushButton {
                background-color: #27ae60; color: white;
                font-size: 13px; font-weight: bold;
                padding: 9px 16px; border-radius: 6px;
            }
            QPushButton:hover { background-color: #219653; }
            QPushButton:disabled { background-color: #cbd5e1; color: #94a3b8; }
        """)
        self.btn_dispatch_mission.clicked.connect(self.on_dispatch_nlp_mission)
        h_dispatch.addWidget(self.btn_dispatch_mission, stretch=3)

        self.btn_clear_batch = QPushButton("✕ Xóa lộ trình")
        self.btn_clear_batch.setStyleSheet("""
            QPushButton {
                background-color: #ecf0f1; color: #7f8c8d;
                font-size: 12px; padding: 9px 12px; border-radius: 6px;
            }
            QPushButton:hover { background-color: #e2e8f0; color: #334155; }
        """)
        self.btn_clear_batch.clicked.connect(lambda: self.update_batch_display(None, []))
        h_dispatch.addWidget(self.btn_clear_batch, stretch=1)
        grp_batch_lo.addLayout(h_dispatch)

        # Dữ liệu kỹ thuật JSON (Raw JSON)
        self.txt_raw_json = QTextEdit()
        self.txt_raw_json.setReadOnly(True)
        self.txt_raw_json.setMaximumHeight(65)
        self.txt_raw_json.setStyleSheet("background-color: #f8fafc; color: #64748b; font-family: monospace; font-size: 11px; border: 1px solid #e2e8f0; border-radius: 4px;")
        self.txt_raw_json.setPlaceholderText("Raw DTO JSON (MessageResponse)...")
        grp_batch_lo.addWidget(self.txt_raw_json)

        right_lo.addWidget(grp_batch, stretch=1)

        splitter.addWidget(right_box)
        splitter.setSizes([560, 480])

        main_layout.addWidget(splitter)

        self.init_chat_welcome()
        return chat_widget

    def init_chat_welcome(self):
        welcome_html = """
        <div style="margin-bottom: 12px;">
            <div style="color: #2980b9; font-weight: bold; margin-bottom: 4px;">🤖 Trợ lý Robot (NLP Assistant):</div>
            <div style="background-color: #ebf5fb; color: #2c3e50; border-radius: 6px; padding: 10px; border-left: 4px solid #3498db; line-height: 1.4;">
                Xin chào! Tôi là trợ lý giao nhận và dẫn đường văn phòng.<br>
                Bạn có thể ra lệnh bằng tiếng Việt tự nhiên, ví dụ:<br>
                • <i>"Giao tài liệu đến phòng 2."</i><br>
                • <i>"Giao hợp đồng đến phòng 2 và báo cáo đến phòng 5."</i><br>
                • <i>"Giao hợp đồng đến phòng 2, báo cáo đến phòng 5 và hồ sơ đến phòng 7. Ưu tiên hồ sơ trước."</i><br>
                <span style="color: #7f8c8d; font-size: 11px;">Mẹo: Bạn có thể click các nút lệnh mẫu ở bên dưới để thử nghiệm ngay!</span>
            </div>
        </div>
        """
        self.txt_chat_history.setHtml(welcome_html)

    def on_toggle_mock_mode(self, is_checked):
        self.nlp_client.set_mock_mode(is_checked)
        if is_checked:
            self.lbl_nlp_status.setText("Trạng thái: ● Đang chạy Giả lập Offline (Mock Contract v1)")
            self.lbl_nlp_status.setStyleSheet("color: #27ae60; font-size: 12px; font-weight: 500;")
            self.log_msg("[NLP] Đã chuyển sang Chế độ Giả lập Offline (Mock Contract)", Qt.darkGreen)
        else:
            self.lbl_nlp_status.setText("Trạng thái: ● Kết nối HTTP Trực tiếp")
            self.lbl_nlp_status.setStyleSheet("color: #2980b9; font-size: 12px; font-weight: 500;")
            self.log_msg("[NLP] Đã chuyển sang Chế độ Kết nối HTTP Trực tiếp", Qt.blue)

    def on_session_created(self, session_id):
        self.lbl_session_info.setText(f"Session: {session_id[:16]}")
        self.log_msg(f"[NLP Session] Đã tạo phiên: {session_id}", Qt.darkCyan)

    def on_health_checked(self, is_ok, msg):
        if is_ok:
            self.lbl_nlp_status.setText(f"Trạng thái: ● {msg}")
            self.lbl_nlp_status.setStyleSheet("color: #27ae60; font-size: 12px; font-weight: 500;")
            self.log_msg(f"[NLP Health] {msg}", Qt.darkGreen)
        else:
            self.lbl_nlp_status.setText(f"Trạng thái: ❌ {msg}")
            self.lbl_nlp_status.setStyleSheet("color: #e74c3c; font-size: 12px; font-weight: 500;")
            self.log_msg(f"[NLP Health] {msg}", Qt.red)

    def on_quick_prompt_clicked(self, prompt_text):
        self.txt_chat_input.setText(prompt_text)
        self.on_send_chat()

    def on_resolve_decision(self, choice_text):
        self.txt_chat_input.setText(choice_text)
        self.on_send_chat()

    def on_send_chat(self):
        text = self.txt_chat_input.text().strip()
        if not text:
            return

        endpoint = self.txt_nlp_url.text().strip()
        if not endpoint:
            self.log_msg("[NLP] Lỗi: Chưa cấu hình địa chỉ URL NLP API.", Qt.red)
            return

        timestamp = time.strftime("%H:%M:%S")
        user_html = f"""
        <div style="margin-bottom: 12px; text-align: right;">
            <div style="color: #27ae60; font-weight: bold; margin-bottom: 4px;">🧑 Bạn ({timestamp}):</div>
            <div style="display: inline-block; text-align: left; background-color: #eafaf1; color: #1e8449; border-radius: 6px; padding: 8px 12px; border-left: 4px solid #2ecc71;">
                {text}
            </div>
        </div>
        """
        self.txt_chat_history.append(user_html)
        self.txt_chat_history.verticalScrollBar().setValue(self.txt_chat_history.verticalScrollBar().maximum())
        self.txt_chat_input.clear()

        self.btn_send_chat.setEnabled(False)
        self.btn_send_chat.setText("Đang gửi...")
        self.lbl_nlp_status.setText("Trạng thái: ⏳ Đang xử lý yêu cầu...")
        self.lbl_nlp_status.setStyleSheet("color: #e67e22; font-size: 12px;")

        self.nlp_client.send_prompt(endpoint, text)
        self.log_msg(f"[NLP Request] '{text}'", Qt.darkBlue)

    def on_nlp_response(self, parsed_data):
        self.btn_send_chat.setEnabled(True)
        self.btn_send_chat.setText("Gửi 🚀")
        self.lbl_nlp_status.setText("Trạng thái: ● Nhận phản hồi thành công")
        self.lbl_nlp_status.setStyleSheet("color: #27ae60; font-size: 12px;")

        reply = parsed_data.get("reply") or parsed_data.get("response", "")
        status = parsed_data.get("dialogueStatus") or parsed_data.get("task", "UNKNOWN")
        batch_dict = parsed_data.get("batch")
        batch_obj = parsed_data.get("batch_obj")
        ordered_tasks = parsed_data.get("ordered_tasks", [])
        raw_data = parsed_data.get("raw", {})
        session_id = parsed_data.get("sessionId")
        if session_id and session_id != "N/A":
            self.lbl_session_info.setText(f"Session: {session_id[:16]}")

        self.current_nlp_batch = batch_obj or batch_dict
        if not ordered_tasks and self.current_nlp_batch:
            if hasattr(self.current_nlp_batch, "get_ordered_tasks"):
                ordered_tasks = self.current_nlp_batch.get_ordered_tasks()
            elif isinstance(self.current_nlp_batch, dict):
                tasks = self.current_nlp_batch.get("tasks", [])
                previews = self.current_nlp_batch.get("orderPreview", [])
                if previews:
                    task_map = {t.get("taskId"): t for t in tasks if isinstance(t, dict)}
                    sorted_prev = sorted(previews, key=lambda p: p.get("position", 0) if isinstance(p, dict) else getattr(p, "position", 0))
                    ordered_tasks = [task_map[p.get("taskId") if isinstance(p, dict) else getattr(p, "taskId")] for p in sorted_prev if (p.get("taskId") if isinstance(p, dict) else getattr(p, "taskId")) in task_map]
                else:
                    ordered_tasks = tasks
        self.current_ordered_tasks = ordered_tasks

        # Hiển thị tin nhắn trợ lý trong khung chat
        timestamp = time.strftime("%H:%M:%S")
        bot_html = f"""
        <div style="margin-bottom: 14px;">
            <div style="color: #2980b9; font-weight: bold; margin-bottom: 4px;">🤖 Trợ lý Robot ({timestamp}):</div>
            <div style="background-color: #ebf5fb; color: #2c3e50; border-radius: 6px; padding: 10px; border-left: 4px solid #2ecc71; line-height: 1.4;">
                {reply}
            </div>
        </div>
        """
        self.txt_chat_history.append(bot_html)
        self.txt_chat_history.verticalScrollBar().setValue(self.txt_chat_history.verticalScrollBar().maximum())

        # Cập nhật Badge Trạng thái và Gợi ý xử lý
        self.box_decision_buttons.setVisible(status == "DECISION_READY")
        if status == "TASK_READY":
            self.lbl_task_badge.setText("✅ NHIỆM VỤ ĐƠN SẴN SÀNG (TASK_READY)")
            self.lbl_task_badge.setStyleSheet("background-color: #e8f8f5; color: #16a085; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #a3e4d7;")
            self.lbl_status_hint.setText("Nhiệm vụ đơn lẻ đã sẵn sàng. Bạn có thể bấm 'Thực thi Lộ trình trên Robot' bên dưới.")
            self.lbl_status_hint.setStyleSheet("color: #16a085; font-size: 11px;")
        elif status == "BATCH_READY":
            self.lbl_task_badge.setText("✅ LỘ TRÌNH ĐA ĐIỂM SẴN SÀNG (BATCH_READY)")
            self.lbl_task_badge.setStyleSheet("background-color: #e8f8f5; color: #16a085; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #a3e4d7;")
            self.lbl_status_hint.setText("Lộ trình đã được tối ưu thứ tự giao hàng. Bạn có thể bấm 'Thực thi Lộ trình trên Robot'.")
            self.lbl_status_hint.setStyleSheet("color: #16a085; font-size: 11px;")
        elif status == "CLARIFICATION_REQUIRED":
            self.lbl_task_badge.setText("🟠 CẦN LÀM RÕ ĐIỂM ĐẾN (CLARIFICATION_REQUIRED)")
            self.lbl_task_badge.setStyleSheet("background-color: #fef9e7; color: #d35400; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #f9e79f;")
            self.lbl_status_hint.setText("Chưa xác định được phòng đích. Vui lòng nói rõ tên phòng (Ví dụ bấm: '👉 Phòng 5.').")
            self.lbl_status_hint.setStyleSheet("color: #d35400; font-size: 11px; font-weight: bold;")
        elif status == "DECISION_READY":
            self.lbl_task_badge.setText("🟣 XUNG ĐỘT THỨ TỰ (DECISION_READY)")
            self.lbl_task_badge.setStyleSheet("background-color: #f4ecf7; color: #6c3483; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #d2b4de;")
            self.lbl_status_hint.setText("Có xung đột giữa thứ tự nói trước và mức ưu tiên cao hơn. Vui lòng bấm chọn phương án bên dưới:")
            self.lbl_status_hint.setStyleSheet("color: #6c3483; font-size: 11px; font-weight: bold;")
        elif status == "SOCIAL":
            self.lbl_task_badge.setText("💬 GIAO TIẾP XÃ HỘI (SOCIAL)")
            self.lbl_task_badge.setStyleSheet("background-color: #ebf5fb; color: #2980b9; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #aed6f1;")
            self.lbl_status_hint.setText("Chào hỏi xã giao. Hãy gửi một lệnh giao tài liệu tới các phòng.")
            self.lbl_status_hint.setStyleSheet("color: #2980b9; font-size: 11px;")
        elif status == "PENDING_CANCELLED":
            self.lbl_task_badge.setText("⚪ ĐÃ HỦY LỆNH CHỜ (PENDING_CANCELLED)")
            self.lbl_task_badge.setStyleSheet("background-color: #f2f4f4; color: #7f8c8d; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #bdc3c7;")
            self.lbl_status_hint.setText("Các lệnh đang chờ xử lý đã được hủy.")
            self.lbl_status_hint.setStyleSheet("color: #7f8c8d; font-size: 11px;")
        else:
            self.lbl_task_badge.setText(f"⚠️ TRẠNG THÁI: {status}")
            self.lbl_task_badge.setStyleSheet("background-color: #fdedec; color: #c0392b; font-weight: bold; font-size: 13px; padding: 9px; border-radius: 6px; border: 1px solid #f5b7b1;")
            self.lbl_status_hint.setText("Chưa hiểu lệnh. Bạn hãy thử chọn các câu lệnh mẫu ở góc trái.")
            self.lbl_status_hint.setStyleSheet("color: #c0392b; font-size: 11px;")

        # Cập nhật phản hồi văn bản
        self.txt_nlp_response_view.setText(reply)

        # Cập nhật bảng Lộ trình
        self.update_batch_display(self.current_nlp_batch, ordered_tasks)

        # Cập nhật Raw JSON
        try:
            self.txt_raw_json.setText(json.dumps(raw_data, ensure_ascii=False, indent=2))
        except Exception:
            self.txt_raw_json.setText(str(raw_data))

        self.log_msg(f"[NLP Success] Status: {status} | Điểm dừng: {len(ordered_tasks)}", Qt.darkGreen)

    def on_nlp_error(self, err_msg):
        self.btn_send_chat.setEnabled(True)
        self.btn_send_chat.setText("Gửi 🚀")
        self.lbl_nlp_status.setText("Trạng thái: ❌ Lỗi kết nối")
        self.lbl_nlp_status.setStyleSheet("color: #e74c3c; font-size: 12px;")

        timestamp = time.strftime("%H:%M:%S")
        err_html = f"""
        <div style="margin-bottom: 12px;">
            <div style="color: #c0392b; font-weight: bold; margin-bottom: 4px;">⚠️ Lỗi NLP ({timestamp}):</div>
            <div style="background-color: #fdedec; color: #c0392b; border-radius: 6px; padding: 8px 12px; border-left: 4px solid #e74c3c;">
                {err_msg}
            </div>
        </div>
        """
        self.txt_chat_history.append(err_html)
        self.txt_chat_history.verticalScrollBar().setValue(self.txt_chat_history.verticalScrollBar().maximum())
        self.log_msg(f"[NLP Error] {err_msg}", Qt.red)

    def update_nlp_status(self, status_text):
        if not hasattr(self, "lbl_nlp_status"):
            return
        self.lbl_nlp_status.setText(f"Trạng thái: {status_text}")
        if "Lỗi" in status_text:
            self.lbl_nlp_status.setStyleSheet("color: #e74c3c; font-size: 12px;")
        elif "thành công" in status_text or "tốt" in status_text or "Sẵn sàng" in status_text or "Giả lập" in status_text:
            self.lbl_nlp_status.setStyleSheet("color: #27ae60; font-size: 12px;")
        else:
            self.lbl_nlp_status.setStyleSheet("color: #e67e22; font-size: 12px;")

    def on_test_nlp_connection(self):
        endpoint = self.txt_nlp_url.text().strip()
        if not endpoint:
            self.lbl_nlp_status.setText("Trạng thái: Chưa nhập URL")
            return
        self.nlp_client.test_health(endpoint)

    def on_create_new_session(self):
        endpoint = self.txt_nlp_url.text().strip()
        self.nlp_client.create_new_session(endpoint)

    def on_clear_chat(self):
        self.txt_chat_history.clear()
        self.init_chat_welcome()

    def update_batch_display(self, batch_data, ordered_tasks):
        self.table_batch.setRowCount(0)
        self.current_ordered_tasks = ordered_tasks or []

        if not self.current_ordered_tasks:
            self.lbl_batch_count.setText("📊 Điểm dừng: 0 | Chính sách: N/A")
            self.btn_dispatch_mission.setEnabled(False)
            self.btn_dispatch_mission.setText("🚀 Thực thi Lộ trình trên Robot (Gửi tới Nav2)")
            return

        policy = "N/A"
        if isinstance(batch_data, dict):
            policy = batch_data.get("orderPolicy") or "PRIORITY"
        elif hasattr(batch_data, "orderPolicy"):
            policy = str(getattr(batch_data, "orderPolicy")) or "PRIORITY"

        self.lbl_batch_count.setText(f"📊 Điểm dừng: {len(self.current_ordered_tasks)} | Chính sách: {policy}")
        self.table_batch.setRowCount(len(self.current_ordered_tasks))
        self.btn_dispatch_mission.setEnabled(True)
        self.btn_dispatch_mission.setText(f"🚀 Thực thi Lộ trình trên Robot ({len(self.current_ordered_tasks)} phòng)")

        for row, task in enumerate(self.current_ordered_tasks):
            # 1. Thứ tự preview (#)
            item_pos = QTableWidgetItem(f"Điểm {row + 1}")
            item_pos.setTextAlignment(Qt.AlignCenter)
            item_pos.setFont(QFont("Arial", 9, QFont.Bold))
            self.table_batch.setItem(row, 0, item_pos)

            # Trích xuất dữ liệu task
            if isinstance(task, dict):
                dest = task.get("destinationText") or task.get("destinationId") or f"Phòng_{row+1}"
                doc = task.get("documentText") or "(Không chỉ định)"
                prio = task.get("priority", "NORMAL")
                constraints = task.get("constraints") or {}
                if isinstance(constraints, dict):
                    c_parts = [f"{k}={v}" for k, v in constraints.items() if v]
                    c_str = ", ".join(c_parts) if c_parts else "Mặc định"
                else:
                    c_str = str(constraints)
                dest_id = task.get("destinationId", "")
            else:
                dest = getattr(task, "destinationText", "") or getattr(task, "destinationId", f"Phòng_{row+1}")
                doc = getattr(task, "documentText", "") or "(Không chỉ định)"
                prio = getattr(task, "priority", "NORMAL")
                c_str = "Mặc định"
                dest_id = getattr(task, "destinationId", "")

            # 2. Điểm đến
            item_dest = QTableWidgetItem(str(dest).capitalize())
            item_dest.setFont(QFont("Arial", 9, QFont.Bold))
            self.table_batch.setItem(row, 1, item_dest)

            # 3. Tài liệu / Vật phẩm
            item_doc = QTableWidgetItem(str(doc).capitalize())
            self.table_batch.setItem(row, 2, item_doc)

            # 4. Ưu tiên (Priority Badge)
            item_prio = QTableWidgetItem(f" {prio} ")
            item_prio.setTextAlignment(Qt.AlignCenter)
            if prio == "HIGH":
                item_prio.setForeground(QColor("#c0392b"))
                item_prio.setBackground(QColor("#fadbd8"))
                item_prio.setFont(QFont("Arial", 9, QFont.Bold))
            else:
                item_prio.setForeground(QColor("#2980b9"))
                item_prio.setBackground(QColor("#ebf5fb"))
            self.table_batch.setItem(row, 3, item_prio)

            # 5. Ràng buộc
            item_const = QTableWidgetItem(c_str)
            item_const.setForeground(QColor("#7f8c8d"))
            self.table_batch.setItem(row, 4, item_const)

            # 6. Tọa độ Staging Pose (Từ phòng bản đồ hoặc mock)
            coord_str = self._lookup_room_coords(str(dest_id) or str(dest))
            item_coords = QTableWidgetItem(coord_str)
            item_coords.setFont(QFont("Consolas", 8))
            self.table_batch.setItem(row, 5, item_coords)

    def _lookup_room_coords(self, dest_text: str) -> str:
        clean = str(dest_text).lower().replace("phòng", "").strip()
        # 1. Tìm trong danh sách phòng bản đồ ROS (self.ros_node.rooms) -> Ưu tiên Staging Pose ngoài cửa
        if hasattr(self, "ros_node") and hasattr(self.ros_node, "rooms"):
            m_id = self.resolve_room_id(clean)
            if m_id and m_id in self.ros_node.rooms:
                r = self.ros_node.rooms[m_id]
                if r.doors and len(r.doors) > 0:
                    sp = r.doors[0].staging_pose
                    if sp.x != 0.0 or sp.y != 0.0:
                        return f"Staging: ({sp.x:.2f}, {sp.y:.2f})"
                if hasattr(r, "room_anchor") and (r.room_anchor.x != 0.0 or r.room_anchor.y != 0.0):
                    return f"Anchor: ({r.room_anchor.x:.2f}, {r.room_anchor.y:.2f})"

        # 2. Tìm trong cấu hình OfflineContractMock (đã nạp từ config/rooms.yaml) -> Ưu tiên Staging Pose ngoài cửa
        try:
            from office_hmi_gui.nlp_client import OfflineContractMock
            mc = OfflineContractMock.get_room_staging(clean)
            if mc:
                return f"Staging: ({mc['x']:.2f}, {mc['y']:.2f})"
            mc_anchor = OfflineContractMock.get_room_anchor(clean)
            if mc_anchor:
                return f"Anchor: ({mc_anchor['x']:.2f}, {mc_anchor['y']:.2f})"
        except Exception:
            pass
        return "Chưa có Staging"

    def on_dispatch_nlp_mission(self):
        """Chuyển lộ trình NLP đã tối ưu sang bộ điều phối Mission Dispatcher của Robot (giao trực tiếp đến ĐIỂM CHỜ NGOÀI CỬA - STAGING POSE)."""
        if not self.current_ordered_tasks:
            QMessageBox.warning(self, "Chưa có lộ trình", "Không có nhiệm vụ giao hàng nào trong danh sách.")
            return

        room_names_or_ids = []
        staging_details = []
        for task in self.current_ordered_tasks:
            if isinstance(task, dict):
                r = task.get("destinationId") or task.get("destinationText")
            else:
                r = getattr(task, "destinationId", "") or getattr(task, "destinationText", "")
            clean = str(r).replace("phòng", "").strip()
            resolved = self.resolve_room_id(clean) or clean
            if resolved:
                room_names_or_ids.append(resolved)
                coord_str = self._lookup_room_coords(resolved)
                staging_details.append(f"• Phòng {resolved}: {coord_str}")

        if not room_names_or_ids:
            QMessageBox.warning(self, "Lỗi điểm đến", "Không thể trích xuất ID phòng hợp lệ từ lộ trình.")
            return

        route_str = ", ".join(room_names_or_ids)
        self.edit_mission_rooms.setText(route_str)

        # CỰC KỲ QUAN TRỌNG: Bắt buộc chọn loại đích đến là "Điểm chờ ngoài cửa (Staging Pose)" theo yêu cầu
        self.cmb_target_type.setCurrentIndex(0)
        self.mission_force_staging = True
        self.mission_force_anchor = False

        self.log_msg(f"🎯 [NLP Chat ➔ Mission] ĐÃ THIẾT LẬP ĐÍCH ĐẾN: 'ĐIỂM CHỜ NGOÀI CỬA (STAGING POSE)'!", Qt.darkCyan)
        for sd in staging_details:
            self.log_msg(f"   {sd}", Qt.darkCyan)
        self.log_msg(f"🚀 [NLP Chat ➔ Mission] Đã nạp lộ trình {route_str} vào bộ điều phối Robot Nav2.", Qt.darkGreen)

        # Chuyển ngay sang tab Bản đồ & Bắt đầu nhiệm vụ
        self.tabs.setCurrentIndex(0)
        self.start_mission()
        QMessageBox.information(
            self,
            "Đã điều phối Robot Giao Điểm Staging 🚀",
            f"Đã kích hoạt lộ trình giao hàng trực tiếp đến ĐIỂM CHỜ NGOÀI CỬA (STAGING POSE) qua {len(room_names_or_ids)} phòng ({route_str}) theo đúng thứ tự tối ưu!\n\n"
            + "\n".join(staging_details) + "\n\n"
            f"Robot đang di chuyển đến tọa độ Staging Pose ngoài cửa của từng phòng."
        )


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
