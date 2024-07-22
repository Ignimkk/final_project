# aruco_cmd_vel_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from .coordinate_transformer import CoordinateTransformer
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from std_msgs.msg import Header

class ArucoCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('aruco_cmd_vel_publisher')
        self.get_logger().info('ArucoCmdVelPublisher node has been started.')

        self.bridge = CvBridge()
        self.marker_size = 7.5  # 센티미터 단위

        try:
            package_share_directory = get_package_share_directory('aruco_detector_pkg')
            calib_data_path = os.path.join(package_share_directory, 'calib_data', 'MultiMatrix.npz')

            # 캘리브레이션 데이터 로드
            calib_data = np.load(calib_data_path)
            self.cam_mat = calib_data["camMatrix"]
            self.dist_coef = calib_data["distCoef"]

            # 아루코 마커 사전 및 감지 파라미터 설정
            self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
            self.parameters = cv.aruco.DetectorParameters()
            self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

            # 이미지를 subscribe
            self.image_subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )

            # goal_pose 퍼블리셔
            self.goal_pose_publisher = self.create_publisher(Float32MultiArray, 'goal_pose', 10)

            # 오도메트리 퍼블리셔
            self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

            # 좌표 변환 객체 생성
            base_to_front_camera_translation = [0.045, 0.0, 0.085]
            base_to_front_camera_rotation = [0, 0, 0]
            front_camera_to_camera_link_translation = [0.0111, 0.0, 0.0193]
            front_camera_to_camera_link_rotation = [0, np.pi / 2, 0]
            map_origin = [-2.15, -0.503, 0]
            map_resolution = 0.05
            self.transformer = CoordinateTransformer(
                base_to_front_camera_translation, base_to_front_camera_rotation,
                front_camera_to_camera_link_translation, front_camera_to_camera_link_rotation,
                map_origin, map_resolution
            )

            self.get_logger().info('Initialization complete.')

        except Exception as e:
            self.get_logger().error(f'Error during initialization: {e}')

    def image_callback(self, msg):
        try:
            # 이미지 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # 아루코 마커 검출 및 코너 좌표 추출
            marker_corners, marker_IDs, _ = self.detector.detectMarkers(gray_frame)

            if marker_corners:
                for marker_corner, marker_id in zip(marker_corners, marker_IDs):
                    # 각 마커의 2D 코너를 1D 배열로 변환
                    marker_corner_2d = marker_corner.reshape(-1, 2).astype(np.float32)

                    # PnP 알고리즘을 사용한 포즈 추정
                    ret, rVec, tVec = cv.solvePnP(
                        self.get_marker_corners_3d(), marker_corner_2d, self.cam_mat, self.dist_coef
                    )

                    if ret:
                        # 지도 기준 좌표로 변환
                        map_translation = [1.0, 2.0, 0.0]
                        map_rotation = [0, 0, 0]
                        world_point = self.transformer.transform_camera_to_map(tVec.flatten(), map_translation, map_rotation)
                        self.get_logger().info(f'Aruco marker world coordinates: {world_point}')

                        # goal_pose 발행
                        goal_pose = Float32MultiArray(data=[world_point[0], world_point[1], 0.0])  # z축은 0으로 설정
                        self.goal_pose_publisher.publish(goal_pose)

                        # 오도메트리 메시지 발행
                        odometry_msg = self.create_odometry_message(world_point)
                        self.odom_publisher.publish(odometry_msg)

            else:
                self.get_logger().info('No markers detected.')
            
            # OpenCV를 사용하여 화면에 이미지 표시
            cv.imshow('Aruco Detection', frame)
            cv.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Error in process_image: {e}')

    def get_marker_corners_3d(self):
        return np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)

    def create_odometry_message(self, position):
        odometry_msg = Odometry()

        # 메시지 헤더 설정
        odometry_msg.header = Header()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_link'

        # 위치와 자세 설정
        odometry_msg.pose = PoseWithCovariance()
        odometry_msg.pose.pose = Pose()
        odometry_msg.pose.pose.position.x = position[0]
        odometry_msg.pose.pose.position.y = position[1]
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.x = 0.0
        odometry_msg.pose.pose.orientation.y = 0.0
        odometry_msg.pose.pose.orientation.z = 0.0
        odometry_msg.pose.pose.orientation.w = 1.0

        # 속도 설정 (이 예제에서는 속도는 0으로 설정)
        odometry_msg.twist = TwistWithCovariance()
        odometry_msg.twist.twist = Twist()
        odometry_msg.twist.twist.linear.x = 0.0
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = 0.0

        return odometry_msg

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

    # OpenCV 창 닫기
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
