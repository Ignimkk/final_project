import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'new_goal_pose',
            self.goal_callback,
            10)
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin = None
        self.goal_pose = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin
        self.get_logger().info('Map received: {}x{}, resolution: {}'.format(
            self.map_width, self.map_height, self.map_resolution))

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info('Goal received: x={}, y={}'.format(
            self.goal_pose.position.x, self.goal_pose.position.y))
        if self.map_data is not None:
            self.plan_path()

    def plan_path(self):
        sx, sy = 0.0, 0.0  # 로봇의 시작 위치 (예: 실제 위치를 얻어올 수 있도록 수정 가능)
        gx = self.goal_pose.position.x
        gy = self.goal_pose.position.y
        ox, oy = [], []
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.map_data[y, x] > 50:  # 50은 장애물의 임계값
                    ox.append(x * self.map_resolution + self.map_origin.position.x)
                    oy.append(y * self.map_resolution + self.map_origin.position.y)
        a_star = AStar(ox, oy, self.map_resolution, 0.1)
        rx, ry = a_star.planning(sx, sy, gx, gy)
        path = Path()
        path.header.frame_id = 'map'
        for x, y in zip(rx, ry):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        self.path_publisher.publish(path)
        self.get_logger().info('Published planned path')

class AStar:
    def __init__(self, ox, oy, resolution, rr):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y, self.max_x, self.max_y = 0, 0, 0, 0
        self.x_width, self.y_width = 0, 0
        self.obstacle_map = None
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if not open_set:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x, current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_grid_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    def calc_heuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * np.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return node.y * self.x_width + node.x

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = np.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, np.sqrt(2)],
                  [-1, 1, np.sqrt(2)],
                  [1, -1, np.sqrt(2)],
                  [1, 1, np.sqrt(2)]]
        return motion

def main(args=None):
    rclpy.init(args=args)
    a_star_planner = AStarPlanner()
    rclpy.spin(a_star_planner)
    a_star_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
