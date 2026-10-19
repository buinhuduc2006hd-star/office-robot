import math
import rclpy
from rclpy.node import Node
from office_interfaces.msg import RoomList, RoomState, KeepoutZone, ActiveKeepoutZones
from office_interfaces.srv import SetRoomState
from geometry_msgs.msg import Polygon, Point32, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

class RoomAccessManager(Node):
    def __init__(self):
        super().__init__('room_access_manager')
        
        self.rooms_sub = self.create_subscription(RoomList, 'office/rooms', self.rooms_callback, 10)
        
        self.state_pub = self.create_publisher(RoomState, 'office/room_states', 10)
        self.active_zones_pub = self.create_publisher(ActiveKeepoutZones, '/office/keepout_updates', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/office/keepout_markers', 10)
        
        self.set_state_srv = self.create_service(SetRoomState, 'office/set_room_state', self.set_room_state_callback)
        
        self.rooms_config = {}
        self.room_states = {}
        self.active_polygons = [] # List of KeepoutZone
        self.prev_marker_ids = set()

        # Heartbeat timer (1Hz) để liên tục đồng bộ trạng thái cửa cản với Nav2 Costmap và RViz2
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def rooms_callback(self, msg):
        for room in msg.rooms:
            self.rooms_config[room.room_id] = room
            if room.room_id not in self.room_states:
                self.room_states[room.room_id] = 0 # FREE
                
        # Re-evaluate active zones in case config changed
        self.update_all_active_zones()

    def set_room_state_callback(self, request, response):
        room_id = request.room_id
        new_state = request.state
        
        if room_id not in self.rooms_config:
            response.success = False
            response.message = f"Room {room_id} not found in configuration."
            return response
            
        self.room_states[room_id] = new_state
        self.update_all_active_zones()
        
        # Publish notification
        state_msg = RoomState()
        state_msg.room_id = room_id
        state_msg.state = new_state
        self.state_pub.publish(state_msg)
        
        response.success = True
        response.message = f"Room {room_id} state set to {'BUSY' if new_state == 1 else 'FREE'}."
        return response

    def calculate_door_polygon(self, door, room_anchor):
        # A = p1, B = p2
        ax, ay = door.p1.x, door.p1.y
        bx, by = door.p2.x, door.p2.y
        rx, ry = room_anchor.x, room_anchor.y
        
        # Vector v = B - A
        vx = bx - ax
        vy = by - ay
        
        dist = math.hypot(vx, vy)
        if dist < 0.01:
            self.get_logger().warn(f"Door {door.door_id} is too narrow (width < 0.01m). Skipping.")
            return None
            
        if door.zone_depth <= 0.001:
            self.get_logger().warn(f"Door {door.door_id} has invalid depth ({door.zone_depth}). Skipping.")
            return None
            
        # M = (A + B) / 2
        mx = (ax + bx) / 2.0
        my = (ay + by) / 2.0
        
        # Vector M->R
        mrx = rx - mx
        mry = ry - my
        
        # Vector đơn vị dọc theo mép cửa
        ux = vx / dist
        uy = vy / dist

        # Vector pháp tuyến vuông góc với đường cửa
        n1x = -uy
        n1y = ux
        
        # Hướng vector n vào phía trong phòng
        dot1 = (n1x * mrx) + (n1y * mry)
        if dot1 >= 0:
            nx, ny = n1x, n1y
        else:
            nx, ny = -n1x, -n1y
            
        # Mở rộng nhẹ 2 đầu A, B một đoạn margin để phủ kín mép tường/khuôn cửa
        margin = 0.05
        a_ext_x = ax - ux * margin
        a_ext_y = ay - uy * margin
        b_ext_x = bx + ux * margin
        b_ext_y = by + uy * margin

        # Tạo layer tường cản ảo (Keepout Barrier Wall) bao trọn đường nối P1-P2:
        # Phủ ra phía ngoài hành lang d_out = 0.15m và vào phía trong phòng d_in = max(depth, 0.25m)
        d_out = 0.15
        d_in = max(float(door.zone_depth), 0.25)

        # 4 đỉnh của khối rào chắn đường cửa P1-P2:
        p1_out_x = a_ext_x - nx * d_out
        p1_out_y = a_ext_y - ny * d_out
        p2_out_x = b_ext_x - nx * d_out
        p2_out_y = b_ext_y - ny * d_out
        p2_in_x = b_ext_x + nx * d_in
        p2_in_y = b_ext_y + ny * d_in
        p1_in_x = a_ext_x + nx * d_in
        p1_in_y = a_ext_y + ny * d_in
        
        # Polygon: P1_out -> P2_out -> P2_in -> P1_in
        poly = Polygon()
        poly.points = [
            Point32(x=float(p1_out_x), y=float(p1_out_y), z=0.0),
            Point32(x=float(p2_out_x), y=float(p2_out_y), z=0.0),
            Point32(x=float(p2_in_x), y=float(p2_in_y), z=0.0),
            Point32(x=float(p1_in_x), y=float(p1_in_y), z=0.0)
        ]
        return poly

    def update_all_active_zones(self):
        self.active_polygons = []
        for room_id, state in self.room_states.items():
            if state == 1: # BUSY
                room = self.rooms_config.get(room_id)
                if room:
                    for door in room.doors:
                        poly = self.calculate_door_polygon(door, room.room_anchor)
                        if poly:
                            kz = KeepoutZone()
                            kz.room_id = room_id
                            kz.door_id = door.door_id
                            kz.polygon = poly
                            self.active_polygons.append(kz)
                            
        self.publish_active_zones()
        self.publish_markers()
        
    def timer_callback(self):
        self.publish_active_zones()
        self.publish_markers()

    def publish_active_zones(self):
        msg = ActiveKeepoutZones()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.zones = self.active_polygons
        self.active_zones_pub.publish(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        current_marker_ids = set()
        now = self.get_clock().now().to_msg()

        for room_id, state in self.room_states.items():
            if state == 1:  # BUSY
                room = self.rooms_config.get(room_id)
                if not room:
                    continue
                for door in room.doors:
                    p1 = door.p1
                    p2 = door.p2
                    dx = p2.x - p1.x
                    dy = p2.y - p1.y
                    dist = math.hypot(dx, dy)
                    if dist < 0.01:
                        continue
                    
                    mid_x = (p1.x + p2.x) / 2.0
                    mid_y = (p1.y + p2.y) / 2.0
                    yaw = math.atan2(dy, dx)
                    
                    qz = math.sin(yaw / 2.0)
                    qw = math.cos(yaw / 2.0)

                    base_id = (hash(f"{room_id}_{door.door_id}") & 0x3FFFFFFF)
                    box_id = base_id * 2
                    text_id = base_id * 2 + 1

                    current_marker_ids.add(box_id)

                    # 1. 3D Barrier Box Marker (Khối cản màu đỏ chiều cao thấp)
                    height = 0.35  # Chiều cao thấp 35cm
                    m_box = Marker()
                    m_box.header.frame_id = 'map'
                    m_box.header.stamp = now
                    m_box.ns = 'office_doors'
                    m_box.id = box_id
                    m_box.type = Marker.CUBE
                    m_box.action = Marker.ADD
                    m_box.pose.position.x = float(mid_x)
                    m_box.pose.position.y = float(mid_y)
                    m_box.pose.position.z = float(height / 2.0)  # Đáy nằm sát mặt đất
                    m_box.pose.orientation.z = qz
                    m_box.pose.orientation.w = qw
                    m_box.scale.x = float(dist + 0.1)  # Bề ngang phủ kín cửa
                    m_box.scale.y = 0.35              # Độ dày tường cản 35cm
                    m_box.scale.z = float(height)     # Chiều cao thấp 35cm
                    m_box.color.r = 0.95
                    m_box.color.g = 0.15
                    m_box.color.b = 0.15
                    m_box.color.a = 0.65
                    marker_array.markers.append(m_box)

                    # Luôn gửi lệnh DELETE cho text_id để đảm bảo chữ vàng cũ bị xóa sạch trên RViz2
                    del_text = Marker()
                    del_text.header.frame_id = 'map'
                    del_text.header.stamp = now
                    del_text.ns = 'office_doors'
                    del_text.id = text_id
                    del_text.action = Marker.DELETE
                    marker_array.markers.append(del_text)

        # Xóa các marker cũ của các phòng đã chuyển sang FREE
        removed_ids = self.prev_marker_ids - current_marker_ids
        for rid in removed_ids:
            del_m = Marker()
            del_m.header.frame_id = 'map'
            del_m.header.stamp = now
            del_m.ns = 'office_doors'
            del_m.id = rid
            del_m.action = Marker.DELETE
            marker_array.markers.append(del_m)

        if not current_marker_ids and self.prev_marker_ids:
            del_all = Marker()
            del_all.action = Marker.DELETEALL
            marker_array.markers.append(del_all)

        self.prev_marker_ids = current_marker_ids
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RoomAccessManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
