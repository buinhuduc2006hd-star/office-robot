import rclpy
from rclpy.node import Node
from office_interfaces.msg import RoomState

class ScheduleManager(Node):
    def __init__(self):
        super().__init__('schedule_manager')
        
        self.state_pub = self.create_publisher(RoomState, 'office/room_states', 10)
        
        # In a real system, this would read from a database or Calendar API
        # For demonstration, we'll just publish a dummy state periodically
        # or expose a ROS service to change state.
        
        self.timer = self.create_timer(5.0, self.publish_dummy_state)
        self.toggle = False
        
    def publish_dummy_state(self):
        msg = RoomState()
        msg.room_id = 'room_001'
        msg.state = RoomState.BUSY if self.toggle else RoomState.FREE
        
        self.state_pub.publish(msg)
        self.get_logger().info(f"Published RoomState: {msg.room_id} -> {'BUSY' if msg.state == RoomState.BUSY else 'FREE'}")
        
        self.toggle = not self.toggle

def main(args=None):
    rclpy.init(args=args)
    node = ScheduleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
