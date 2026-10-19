import os
import yaml
import rclpy
from rclpy.node import Node
from office_interfaces.msg import RoomList, RoomConfig, DoorConfig
from office_interfaces.srv import SaveRoom, DeleteRoom

class ConfigManager(Node):
    def __init__(self):
        super().__init__('config_manager')
        
        default_config = os.path.join(os.path.expanduser('~'), 'robot_ws', 'config', 'rooms.yaml')
        self.declare_parameter('config_file', default_config)
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        # Resolve relative path to absolute (relative to ~/robot_ws)
        if not os.path.isabs(self.config_file):
            self.config_file = os.path.join(os.path.expanduser('~'), 'robot_ws', self.config_file)
        
        self.room_list_pub = self.create_publisher(RoomList, 'office/rooms', 10)
        
        self.save_srv = self.create_service(SaveRoom, 'office/save_room', self.save_room_callback)
        self.delete_srv = self.create_service(DeleteRoom, 'office/delete_room', self.delete_room_callback)
        
        self.rooms = []
        self.load_config()
        
        self.timer = self.create_timer(2.0, self.publish_rooms)

    def load_config(self):
        self.rooms = []
        if not os.path.exists(self.config_file):
            self.get_logger().warn(f"Config file {self.config_file} not found. Publishing empty list.")
            return
            
        try:
            with open(self.config_file, 'r') as f:
                data = yaml.safe_load(f)
                
            if not data or 'rooms' not in data:
                return
                
            for room_id, r_data in data['rooms'].items():
                msg = RoomConfig()
                msg.room_id = room_id
                msg.display_name = r_data.get('display_name', room_id)
                msg.description = r_data.get('description', '')
                
                anchor_data = r_data.get('room_anchor') or r_data.get('label_pose', {})
                msg.room_anchor.x = float(anchor_data.get('x', 0.0))
                msg.room_anchor.y = float(anchor_data.get('y', 0.0))
                    
                if 'doors' in r_data:
                    for d_data in r_data['doors']:
                        door = DoorConfig()
                        door.door_id = d_data.get('id', 'default_door')
                        if 'p1' in d_data:
                            door.p1.x = float(d_data['p1'].get('x', 0.0))
                            door.p1.y = float(d_data['p1'].get('y', 0.0))
                        if 'p2' in d_data:
                            door.p2.x = float(d_data['p2'].get('x', 0.0))
                            door.p2.y = float(d_data['p2'].get('y', 0.0))
                        door.zone_depth = float(d_data.get('zone_depth', 0.25))
                        if 'staging_pose' in d_data:
                            door.staging_pose.x = float(d_data['staging_pose'].get('x', 0.0))
                            door.staging_pose.y = float(d_data['staging_pose'].get('y', 0.0))
                            door.staging_pose.theta = float(d_data['staging_pose'].get('yaw', 0.0))
                        msg.doors.append(door)
                
                self.rooms.append(msg)
                
            self.get_logger().info(f"Loaded {len(self.rooms)} rooms from {self.config_file}")
            self.publish_rooms()
            
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")

    def save_to_yaml(self):
        data = {'rooms': {}}
        for r in self.rooms:
            r_data = {
                'display_name': r.display_name,
                'description': r.description,
                'room_anchor': {'x': r.room_anchor.x, 'y': r.room_anchor.y},
                'doors': []
            }
            for d in r.doors:
                d_data = {
                    'id': d.door_id,
                    'p1': {'x': d.p1.x, 'y': d.p1.y},
                    'p2': {'x': d.p2.x, 'y': d.p2.y},
                    'zone_depth': d.zone_depth,
                    'staging_pose': {
                        'x': d.staging_pose.x,
                        'y': d.staging_pose.y,
                        'yaw': d.staging_pose.theta
                    }
                }
                r_data['doors'].append(d_data)
            data['rooms'][r.room_id] = r_data
            
        os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
        with open(self.config_file, 'w') as f:
            yaml.dump(data, f, allow_unicode=True, sort_keys=False)

    def save_room_callback(self, request, response):
        new_room = request.room
        # Find and replace, or append
        replaced = False
        for i, r in enumerate(self.rooms):
            if r.room_id == new_room.room_id:
                self.rooms[i] = new_room
                replaced = True
                break
        if not replaced:
            self.rooms.append(new_room)
            
        try:
            self.save_to_yaml()
            response.success = True
            response.message = f"Room {new_room.room_id} saved successfully."
            self.publish_rooms()
        except Exception as e:
            response.success = False
            response.message = f"Failed to save room: {e}"
        return response

    def delete_room_callback(self, request, response):
        room_id = request.room_id
        initial_len = len(self.rooms)
        self.rooms = [r for r in self.rooms if r.room_id != room_id]
        
        if len(self.rooms) < initial_len:
            try:
                self.save_to_yaml()
                response.success = True
                response.message = f"Room {room_id} deleted."
                self.publish_rooms()
            except Exception as e:
                response.success = False
                response.message = f"Failed to save after deletion: {e}"
        else:
            response.success = False
            response.message = f"Room {room_id} not found."
        return response

    def publish_rooms(self):
        msg = RoomList()
        msg.rooms = self.rooms
        self.room_list_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConfigManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
