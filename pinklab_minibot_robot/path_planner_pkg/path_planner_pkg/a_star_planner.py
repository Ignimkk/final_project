import numpy as np
from PIL import Image
import yaml
import heapq

from pathlib import Path

def load_map(map_yaml_path):
    with open(map_yaml_path, 'r') as file:
        map_data = yaml.safe_load(file)
    
    map_image_path = Path(map_yaml_path).parent / map_data['image']
    resolution = map_data['resolution']
    origin = map_data['origin']
    occupied_thresh = map_data['occupied_thresh']
    free_thresh = map_data['free_thresh']
    
    map_image = Image.open(map_image_path)
    map_array = np.array(map_image)
    
    grid = np.zeros_like(map_array, dtype=np.int8)
    grid[map_array > occupied_thresh * 255] = 1
    grid[map_array < free_thresh * 255] = 0
    
    return grid, resolution, origin


class AStarPlanner:
    def __init__(self, grid, resolution, origin):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_within_bounds(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols

    def is_walkable(self, x, y):
        return self.grid[x][y] == 0

    def plan(self, start, goal):
        start = self.world_to_grid(start)
        goal = self.world_to_grid(goal)
        
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(current))
                    current = came_from[current]
                path.append(self.grid_to_world(start))
                return path[::-1]

            for direction in self.directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if self.is_within_bounds(neighbor[0], neighbor[1]) and self.is_walkable(neighbor[0], neighbor[1]):
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None

    def world_to_grid(self, point):
        x = int((point[0] - self.origin[0]) / self.resolution)
        y = int((point[1] - self.origin[1]) / self.resolution)
        return (x, y)
    
    def grid_to_world(self, point):
        x = point[0] * self.resolution + self.origin[0]
        y = point[1] * self.resolution + self.origin[1]
        return (x, y)

class ObstacleAvoider:
    def __init__(self, grid, resolution, origin):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin

    def avoid_obstacles(self, path):
        adjusted_path = []
        for point in path:
            if self.is_obstacle(point):
                adjusted_path.append(self.find_alternate_route(point))
            else:
                adjusted_path.append(point)
        return adjusted_path

    def is_obstacle(self, point):
        x, y = self.world_to_grid(point)
        return self.grid[x][y] == 1

    def find_alternate_route(self, point):
        x, y = self.world_to_grid(point)
        for direction in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_point = (x + direction[0], y + direction[1])
            if self.is_within_bounds(new_point[0], new_point[1]) and self.grid[new_point[0]][new_point[1]] == 0:
                return self.grid_to_world(new_point)
        return point

    def world_to_grid(self, point):
        x = int((point[0] - self.origin[0]) / self.resolution)
        y = int((point[1] - self.origin[1]) / self.resolution)
        return (x, y)
    
    def grid_to_world(self, point):
        x = point[0] * self.resolution + self.origin[0]
        y = point[1] * self.resolution + self.origin[1]
        return (x, y)
    
    def is_within_bounds(self, x, y):
        return 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0])