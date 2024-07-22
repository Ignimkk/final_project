import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy

class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 목표 위치 구독자 설정
        self.subscription = self.create_subscription(
            PoseStamped,
            '/new_goal_pose',
            self.goal_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # 초기 위치 설정 구독자
        self.initialpose_subscription = self.create_subscription(
            PoseStamped,
            '/initialpose',
            self.initialpose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # 오도메트리 구독자
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # 속도 명령 발행
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.current_pose = None
        self.goal_pose = None

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal: {msg.pose}')
        self.send_goal(msg)

    def initialpose_callback(self, msg):
        self.get_logger().info(f'Setting initial pose: {msg.pose}')
        # 초기 위치를 설정하는 로직 추가 (AMCL 초기화 등)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(f'Current pose: {self.current_pose}')

def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseClient()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
