import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import heapq
import yaml
from PIL import Image
import os
from nav2_simple_commander.robot_navigator import BasicNavigator
import matplotlib.pyplot as plt

# NavigationResult 열거형을 모킹합니다.
class NavigationResult:
    SUCCEEDED = 0
    CANCELED = 1
    FAILED = 2

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        self.navigator = BasicNavigator()

        self.subscription = self.create_subscription(
            PoseStamped,
            'new_goal_pose',
            self.goal_pose_callback,
            10)
        self.subscription

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10)
        self.amcl_subscription

        self.grid_size = 10
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        self.map_loaded = False
        self.current_position = (0.0, 0.0)
        self.current_position_grid = (0, 0)
        self.goal_position = None
        self.path = []
        self.load_map(os.path.expanduser('/home/mk/final_project/ros-repo-4/maps/mfc.yaml'))
        self.get_logger().info('PathPlanningNode가 초기화되었습니다')

    def load_map(self, yaml_file):
        try:
            with open(yaml_file, 'r') as file:
                map_data = yaml.safe_load(file)

            map_image_path = os.path.expanduser(map_data['image'])
            if not os.path.isabs(map_image_path):
                map_image_path = os.path.join(os.path.dirname(yaml_file), map_image_path)

            map_image = Image.open(map_image_path).convert('L')
            map_array = np.array(map_image)
            map_array = np.where(map_array > 127, 0, 1)  # 이진 그리드로 변환 (0: 자유, 1: 장애물)

            # 맵을 10x10 그리드에 맞게 크기 조정
            image = Image.fromarray(map_array.astype(np.uint8) * 255)  # 이미지로 변환
            image = image.resize((self.grid_size, self.grid_size), Image.NEAREST)
            resized_data = np.array(image) // 255  # 다시 이진 그리드로 변환

            self.grid = resized_data
            self.map_loaded = True
            self.get_logger().info('맵이 로드되고 그리드가 생성되었습니다')
            self.get_logger().info(f'그리드 데이터:\n{self.grid}')

            self.visualize_grid()

        except Exception as e:
            self.get_logger().error(f'맵 로드 실패: {e}')

            

    def visualize_grid(self):
        fig, ax = plt.subplots()
        ax.imshow(self.grid, cmap='gray')

        # 그리드 선 추가
        for x in range(self.grid_size):
            ax.axhline(x - 0.5, color='black', linewidth=0.5)
            ax.axvline(x - 0.5, color='black', linewidth=0.5)

        ax.set_xticks(np.arange(-.5, self.grid_size, 1))
        ax.set_yticks(np.arange(-.5, self.grid_size, 1))
        ax.set_xticklabels([])
        ax.set_yticklabels([])

        plt.grid(True)
        plt.savefig('/tmp/grid_visualization.png')  # 이미지 파일로 저장
        plt.show()  # 화면에 표시

    def goal_pose_callback(self, msg):
        if not self.map_loaded:
            self.get_logger().warn('맵이 아직 로드되지 않았습니다')
            return

        # 원시 목표 좌표
        raw_goal_x = msg.pose.position.x
        raw_goal_y = msg.pose.position.y
        self.get_logger().info(f'수신된 원시 목표 지점: ({raw_goal_x}, {raw_goal_y})')

        # 좌표 변환: 1.5m x 3.6m 맵을 고려하여 그리드 크기로 변환
        goal_x = int((raw_goal_x / 1.5) * self.grid_size)
        goal_y = int((raw_goal_y / 3.6) * self.grid_size)
        self.goal_position = (goal_x, goal_y)

        self.get_logger().info(f'변환된 목표 지점: {self.goal_position}')

        if not (0 <= goal_x < self.grid_size and 0 <= goal_y < self.grid_size):
            self.get_logger().warn(f'목표 위치가 범위를 벗어났습니다: {self.goal_position}')
            return

        self.plan_and_execute_path()

    def amcl_callback(self, msg):
        self.get_logger().debug('amcl_callback 호출됨')
        
        # amcl_pose 데이터를 기반으로 로봇의 현재 위치 업데이트
        raw_position_x = msg.pose.pose.position.x
        raw_position_y = msg.pose.pose.position.y

        # 현재 위치는 float형으로 유지
        self.current_position = (raw_position_x, raw_position_y)
        
        # 현재 위치를 그리드 좌표로 변환
        grid_position_x = int((raw_position_x / 1.5) * self.grid_size)
        grid_position_y = int((raw_position_y / 3.6) * self.grid_size)
        self.current_position_grid = (grid_position_x, grid_position_y)

        self.get_logger().info(f'amcl_pose로부터 수신된 위치: ({raw_position_x}, {raw_position_y}), 그리드 위치: {self.current_position_grid}')

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star_algorithm(self, start, goal):
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + 1

                if 0 <= neighbor[0] < self.grid.shape[0]:
                    if 0 <= neighbor[1] < self.grid.shape[1]:
                        if self.grid[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        continue
                else:
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue

                if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
                    
        self.get_logger().info('A* 알고리즘 실패: 경로를 찾지 못했습니다.')
        return False

    def plan_and_execute_path(self):
        if not self.goal_position:
            return

        # 현재 위치와 목표 위치를 그리드 좌표로 변환
        start = self.current_position_grid
        goal = self.goal_position

        self.get_logger().info(f'시작 지점: {start}, 목표 지점: {goal}')

        path = self.a_star_algorithm(start, goal)
        if path:
            self.get_logger().info(f'경로가 성공적으로 생성되었습니다: {path}')
            self.path = path
            self.execute_path()
        else:
            self.get_logger().warn('유효한 경로를 찾을 수 없습니다')

    def execute_path(self):
        if not self.path:
            return

        # self.path는 그리드 좌표로 되어 있으므로 이를 실제 월드 좌표로 변환
        for point in self.path:
            world_x = (point[0] / self.grid_size) * 1.5
            world_y = (point[1] / self.grid_size) * 3.6

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = world_x
            goal_pose.pose.position.y = world_y
            goal_pose.pose.orientation.w = 1.0

            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == NavigationResult.SUCCEEDED:
                    self.get_logger().info(f'목표 지점에 성공적으로 도달: ({world_x}, {world_y})')
                    break
                elif result in [NavigationResult.CANCELED, NavigationResult.FAILED]:
                    self.get_logger().warn(f'목표 지점에 도달 실패: ({world_x}, {world_y})')
                    return

        self.get_logger().info('모든 경로 지점에 도달했습니다.')

def main(args=None):
    rclpy.init(args=args)
    path_planning_node = PathPlanningNode()
    rclpy.spin(path_planning_node)
    path_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
