import rclpy as rp
import os
import time
import math
import json
import heapq
from rclpy.node import Node
from copy import deepcopy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan


class robot_control(Node):
    def __init__(self):
        super().__init__("robot_control")
        self.get_logger().info("start robot_control")
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 로봇의 현재 위치를 구독하는 서브스크립션
        self.subcription_amclpose = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.current_pose,
            10
        )
        
        # 레이저 스캔 데이터를 구독하는 서브스크립션
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.qos_profile,
        )
        
        # 맵 데이터를 구독하는 서브스크립션
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )
        
        self.subscription = self.create_subscription(
                String,
                'detection_result',
                self.listener_callback,
                10)
        self.subscription # prevent unused variable warning
        
        self.goal_subscription = self.create_subscription(
            String,
            'goal_pose',
            self.goal_pose_callback,
            10
        )
        
        self.load_pose_and_yaw()
    
    def goal_pose_callback(self, msg):
        pose_name = msg.data
        self.navigate_to_pose(pose_name)
    
    def load_pose_and_yaw(self):
        try:
            package_share_directory = os.path.join(
                os.path.dirname(__file__), 'lrobot', 'poses.json'
            )
            with open(package_share_directory, 'r') as file:
                data = json.load(file)
                self.POSE_DICT = data['poses']
                self.YAW_DICT = data.get('yaws', {})
                self.get_logger().info("Successfully loaded poses and yaws from JSON file.")
        except Exception as e:
            self.get_logger().error(f"Failed to load poses and yaws from JSON file: {e}")
            self.POSE_DICT = {}
            self.YAW_DICT = {}
    
    def listener_callback(self, msg):
        detection_data = json.loads(msg.data)
        labels = detection_data['labels']
        distances = detection_data['distances']
        self.get_logger().info(f'Received detection data: Labels - {labels}, Distances - {distances}')
        self.obstacles = self.process_detection_data(labels, distances)
    
    def process_detection_data(self, labels, distances):
        obstacles = []
        for label, distance in zip(labels, distances):
            if label == "obstacle":  # 장애물 레이블 확인
                angle = distance['angle']
                distance = distance['distance']
                obs_x = self.my_pose[0] + distance * math.cos(angle + self.my_yaw)
                obs_y = self.my_pose[1] + distance * math.sin(angle + self.my_yaw)
                obstacles.append((obs_x, obs_y))
        return obstacles
    
    # 현재 위치 받기
    def current_pose(self, data):
        self.get_logger().info("Received data on /amcl_pose topic.")
        
        self.my_pose = []
        self.my_pose[0] = data.pose.pose.position.x
        self.my_pose[1] = data.pose.pose.position.y
        self.my_pose[2] = data.pose.pose.position.z
        
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)
        self.my_yaw = yaw
        
        # current_pos 변수에 x,y,z,yaw 값을 리스트로 반환
        self.current_pos = [self.my_pose[0], self.my_pose[1], self.my_yaw]
        
    # 쿼터니언을 오일러 각도로 변환하는 함수
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z
        
    # 목표 지점 지정
    def set_goal_pose(self, pose_name):
        if pose_name not in self.POSE_DICT:
            self.get_logger().error(f"Pose {pose_name} is not in POSE_DICT")
            return None, None
        
        target_pose = self.POSE_DICT[pose_name]
        target_yaw = self.YAW_DICT[pose_name]
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = target_pose[0]
        pose.pose.position.y = target_pose[1]
        pose.pose.position.z = target_pose[2]
        
        quaternion = self.quaternion_from_euler(0, 0, target_yaw)
        
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        
        return pose, pose_name
    
    def navigate_to_pose(self, pose_name):
        goal_pose, pose_name = self.set_goal_pose(pose_name)
        if goal_pose is None:
            self.get_logger().error(f"Failed to set goal pose for {pose_name}")
            return
        
        if self.plan_path(goal_pose):
            self.avoid_obstacles()
    
    # 오일러 각도를 쿼터니언으로 변환하는 함수
    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]
            
    # 경로 지정
    def plan_path(self, goal_pose):
        self.get_logger().info("Planning path")
        self.nav.goToPose(goal_pose)
        result = self.nav.waitUntilNav2Active()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Path planning succeeded")
            return True
        else:
            self.get_logger().error("Path planning failed")
            return False
    
    # 장애물 회피
    def avoid_obstacles(self):
        self.get_logger().info("Avoiding obstacles")
        # 장애물 회피 로직을 구현
        # 여기서는 단순히 로직의 위치만 표시합니다.
        pass

    def scan_callback(self, msg):
        self.get_logger().info("Received data on /scan topic.")
        
    
    def map_callback(self, data):
        self.get_logger().info("Received data on /map topic.")
        
    #     self.map_data = data.data
    #     self.map_info = data.info
    
    #     # 예시: 맵의 가로, 세로 크기와 해상도 출력
    #     self.get_logger().info(f"Map width: {self.map_info.width}, height: {self.map_info.height}")
        
    #     # 예시: 맵의 원점 위치와 방향 출력
    #     origin_x = self.map_info.origin.orientation.x
    #     origin_y = self.map_info.origin.orientation.y
    #     origin_z = self.map_info.origin.orientation.z
    #     origin_w = self.map_info.origin.orientation.w
        
    #     roll, pitch, yaw = self.euler_from_quaternion(origin_x, origin_y, origin_z, origin_w)
    #     self.map_yaw = yaw
        
    #     self.get_logger().info(f"Map origin position: ({data.info.position.x}, {data.info.position.y}, {data.info.position.z})")
    #     self.get_logger().info(f"Map origin orientation: ({self.map_yaw})")
        
        
        
def main(args=None): 
    rp.init(args=args)
    
    node = robot_control()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()
    
    # con = robot_control()

    # executor = MultiThreadedExecutor()
    # executor.add_node(con)
    
    # try: 
    #     executor.spin()
    # finally: 
    #     executor.shutdown()
    #     con.destroy_node()
    #     rp.shutdown()

if __name__ == "__main__": 
    main()