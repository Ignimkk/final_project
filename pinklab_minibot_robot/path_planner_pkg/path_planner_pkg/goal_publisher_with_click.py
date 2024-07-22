import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

class GoalPublisherWithClick(Node):
    def __init__(self):
        super().__init__('goal_publisher_with_click')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.clicked_point_callback,
            10
        )
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'

    def clicked_point_callback(self, msg):
        self.goal_pose.pose.position.x = msg.point.x
        self.goal_pose.pose.position.y = msg.point.y
        self.goal_pose.pose.orientation.w = 1.0
        self.publish_goal()

    def publish_goal(self):
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.goal_pose)
        self.get_logger().info('Published goal position: x={}, y={}'.format(
            self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisherWithClick()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
