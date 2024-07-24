import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.publisher_ = self.create_publisher(String, 'goal_pose', 10)
        self.get_logger().info("UI Node started. Enter a pose name to navigate to:")

    def get_input_and_publish(self):
        while True:
            pose_name = input("Enter pose name: ")
            msg = String()
            msg.data = pose_name
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published pose name: {pose_name}")

def main(args=None):
    rclpy.init(args=args)
    ui_node = UINode()
    try:
        ui_node.get_input_and_publish()
    except KeyboardInterrupt:
        pass
    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
