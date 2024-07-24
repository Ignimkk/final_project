import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/new_goal_pose', 10)
        self.timer = self.create_timer(2, self.publish_goal)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 0.8  # 예시 좌표 x
        self.goal_pose.pose.position.y = 0.8  # 예시 좌표 y
        self.goal_pose.pose.orientation.w = 1.0

    def publish_goal(self):
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.goal_pose)
        self.get_logger().info('Published goal position: x={}, y={}'.format(
            self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
