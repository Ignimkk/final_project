import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID gains
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.1
        
        self.prev_error = 0.0
        self.integral = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y
    
    def control_loop(self):
        error = self.calculate_error(self.current_x, self.current_y, self.target_x, self.target_y)
        self.integral += error
        derivative = error - self.prev_error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        cmd = Twist()
        cmd.linear.x = output  # 선형 속도를 조정하여 로봇을 이동시킴
        self.vel_publisher.publish(cmd)
    
    def calculate_error(self, current_x, current_y, target_x, target_y):
        return math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()

    # Set a target position
    pid_controller.set_target(5.0, 5.0)

    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
