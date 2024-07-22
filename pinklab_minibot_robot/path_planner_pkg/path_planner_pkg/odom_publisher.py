import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
from datetime import datetime

# import tf_transformations
import tf2_ros
import tf2_geometry_msgs

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to Quaternion.
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class VehicleModel:
    @staticmethod
    def create_concrete_vehicle_model(model_name):
        if model_name == "DifferentialDrive":
            return DifferentialDriveModel()
        else:
            raise ValueError(f"Unknown model name: {model_name}")

class DifferentialDriveModel:
    def calculate_next_state(self, rpm_left_avg, rpm_right_avg, current_state, dt):
        wheel_radius = 0.05  # 바퀴 반지름 (미터)
        wheel_base = 0.3  # 바퀴 간 거리 (미터)

        v_left = wheel_radius * rpm_left_avg * 2 * math.pi / 60  # 왼쪽 바퀴 속도 (m/s)
        v_right = wheel_radius * rpm_right_avg * 2 * math.pi / 60  # 오른쪽 바퀴 속도 (m/s)

        v = (v_left + v_right) / 2  # 선속도 (m/s)
        omega = (v_right - v_left) / wheel_base  # 각속도 (rad/s)

        dx = v * math.cos(current_state.yaw) * dt
        dy = v * math.sin(current_state.yaw) * dt
        dyaw = omega * dt

        new_state = VehicleState(
            current_state.x + dx,
            current_state.y + dy,
            current_state.yaw + dyaw
        )

        return new_state

class VehicleState:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class OdometryEstimator(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.vehicle_model = VehicleModel.create_concrete_vehicle_model('DifferentialDrive')
        self.state = VehicleState(0.0, 0.0, 0.0)
        self.previous_time = datetime.now()

        self.right_wheel_subscriber = self.create_subscription(
            Int64,
            'right_wheel_rpm',
            self.handle_right_wheel_input,
            10
        )
        self.left_wheel_subscriber = self.create_subscription(
            Int64,
            'left_wheel_rpm',
            self.handle_left_wheel_input,
            10
        )

        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.publish)

        self.rpms_right = []
        self.rpms_left = []

    def handle_right_wheel_input(self, msg):
        self.rpms_right.append(msg.data)

    def handle_left_wheel_input(self, msg):
        self.rpms_left.append(msg.data)

    def publish(self):
        current_time = datetime.now()
        dt = (current_time - self.previous_time).total_seconds()

        rpm_left_avg = sum(self.rpms_left) / len(self.rpms_left) if self.rpms_left else 0.0
        rpm_right_avg = sum(self.rpms_right) / len(self.rpms_right) if self.rpms_right else 0.0

        self.rpms_left.clear()
        self.rpms_right.clear()

        new_state = self.vehicle_model.calculate_next_state(rpm_left_avg, rpm_right_avg, self.state, dt)

        quat = quaternion_from_euler(0.0, 0.0, new_state.yaw)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.pose.pose.position.x = new_state.x
        message.pose.pose.position.y = new_state.y
        message.pose.pose.orientation = quat

        self.publisher.publish(message)

        self.state = new_state
        self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
