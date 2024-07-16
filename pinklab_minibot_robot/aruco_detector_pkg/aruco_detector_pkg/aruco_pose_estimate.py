import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.get_logger().info('ArucoMarkerDetector node has been started.')

        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 패키지 디렉터리 경로 설정
        try:
            package_share_directory = get_package_share_directory('aruco_detector_pkg')
            calib_data_path = os.path.join(package_share_directory, 'calib_data', 'MultiMatrix.npz')

            # 캘리브레이션 데이터 로드
            calib_data = np.load(calib_data_path)
            self.cam_mat = calib_data["camMatrix"]
            self.dist_coef = calib_data["distCoef"]
            self.r_vectors = calib_data["rVector"]
            self.t_vectors = calib_data["tVector"]

            self.marker_size = 7.5  # 센티미터 단위
            
            # 아루코 마커 사전 및 감지 파라미터 설정
            self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
            self.parameters = cv.aruco.DetectorParameters()
            self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

            self.bridge = CvBridge()

            # 이미지를 subscribe
            self.image_subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )

            # 로봇 포즈 출판 설정
            self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
            
            self.get_logger().info('Initialization complete.')
        
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {e}')

    def image_callback(self, msg):
        self.get_logger().info('Image received.')
        
        try:
            # 이미지 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # 아루코 마커 검출 및 코너 좌표 추출
            marker_corners, marker_IDs, reject = self.detector.detectMarkers(gray_frame)
            self.get_logger().info(f'Detected {len(marker_corners)} markers.')

            if marker_corners:
                for marker_corner, marker_id in zip(marker_corners, marker_IDs):
                    # 각 마커의 2D 코너를 1D 배열로 변환
                    marker_corner_2d = marker_corner.reshape(-1, 2).astype(np.float32)

                    # PnP 알고리즘을 사용한 포즈 추정
                    ret, rVec, tVec = cv.solvePnP(
                        self.get_marker_corners_3d(), marker_corner_2d, self.cam_mat, self.dist_coef
                    )

                    # 마커의 포즈 그리기 및 정보 출력
                    cv.polylines(
                        frame, [marker_corner.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                    )

                    corners = marker_corner.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    bottom_right = corners[2].ravel()

                    distance = np.sqrt(
                        tVec[0][0] ** 2 + tVec[1][0] ** 2 + tVec[2][0] ** 2
                    )

                    cv.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec, tVec, 4, 4)
                    cv.putText(
                        frame,
                        f"id: {marker_id[0]} Dist: {round(distance, 2)}",
                        top_right,
                        cv.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )
                    cv.putText(
                        frame,
                        f"x:{round(tVec[0][0],1)} y: {round(tVec[1][0],1)} z: {round(tVec[2][0],1)}",
                        bottom_right,
                        cv.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )

                    self.get_logger().info(f'Estimated pose: x={tVec[0][0]}, y={tVec[1][0]}, z={tVec[2][0]}')
                    
                    # 로봇 포즈 출판
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'  # Ensure frame ID is set to 'map' if using map-based navigation
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = tVec[0][0]
                    pose.pose.position.y = tVec[1][0]
                    pose.pose.position.z = tVec[2][0]

                    # rotation vector를 quaternion으로 변환
                    rot_matrix, _ = cv.Rodrigues(rVec)
                    pose.pose.orientation.x = rot_matrix[0][0]
                    pose.pose.orientation.y = rot_matrix[1][0]
                    pose.pose.orientation.z = rot_matrix[2][0]
                    pose.pose.orientation.w = rot_matrix[2][1]

                    self.pose_publisher.publish(pose)
                    self.get_logger().info(f'Published pose for marker ID {marker_id[0]}')

                    # TransformStamped 메시지 생성
                    transform = TransformStamped()
                    transform.header.stamp = self.get_clock().now().to_msg()
                    transform.header.frame_id = 'map'  # Ensure frame ID is set to 'map'
                    transform.child_frame_id = 'base_footprint'

                    # 위치 설정
                    transform.transform.translation.x = tVec[0][0]
                    transform.transform.translation.y = tVec[1][0]
                    transform.transform.translation.z = tVec[2][0]

                    # 회전 설정
                    transform.transform.rotation.x = pose.pose.orientation.x
                    transform.transform.rotation.y = pose.pose.orientation.y
                    transform.transform.rotation.z = pose.pose.orientation.z
                    transform.transform.rotation.w = pose.pose.orientation.w

                    # TF 브로드캐스트
                    self.tf_broadcaster.sendTransform(transform)
                    self.get_logger().info(f'TF published for marker ID {marker_id[0]}')
                    
            else:
                self.get_logger().info('No markers detected.')
                
            cv.imshow("frame", frame)
            cv.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

    def get_marker_corners_3d(self):
        return np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
