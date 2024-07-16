import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # 수정된 가져오기 구문

class DriveForwardNode(Node):
    def __init__(self):
        super().__init__('drive_forward')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.duration = 5.0  # assuming the robot moves at 0.2 m/s, 5 seconds to cover 1 meter
        self.get_logger().info("Drive forward node has been started")

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time < self.duration:
            msg = Twist()
            msg.linear.x = 0.2  # Move forward with 0.2 m/s
            self.publisher_.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = 0.0  # Stop the robot
            self.publisher_.publish(msg)
            self.get_logger().info("Reached the destination, stopping the robot")
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DriveForwardNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
