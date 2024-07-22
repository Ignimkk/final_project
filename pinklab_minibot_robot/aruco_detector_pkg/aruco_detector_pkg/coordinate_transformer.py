# coordinate_transformer.py
import numpy as np
from scipy.spatial.transform import Rotation as R

class CoordinateTransformer:
    def __init__(self, base_to_front_camera_translation, base_to_front_camera_rotation, front_camera_to_camera_link_translation, front_camera_to_camera_link_rotation, map_origin, map_resolution):
        self.base_to_front_camera_matrix = self.create_transformation_matrix(base_to_front_camera_translation, self.euler_to_rotation_matrix(*base_to_front_camera_rotation))
        self.front_camera_to_camera_link_matrix = self.create_transformation_matrix(front_camera_to_camera_link_translation, self.euler_to_rotation_matrix(*front_camera_to_camera_link_rotation))
        self.camera_to_base_matrix = np.dot(self.base_to_front_camera_matrix, self.front_camera_to_camera_link_matrix)
        self.map_origin = map_origin
        self.map_resolution = map_resolution

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        r = R.from_euler('xyz', [roll, pitch, yaw])
        return r.as_matrix()

    def create_transformation_matrix(self, translation, rotation_matrix):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation
        return transform_matrix

    def transform_coordinates(self, point, transform_matrix):
        homogenous_point = np.append(point, 1)
        transformed_point = np.dot(transform_matrix, homogenous_point)
        return transformed_point[:3]

    def map_to_world(self, map_point):
        world_point = np.zeros(2)
        world_point[0] = self.map_origin[0] + map_point[0] * self.map_resolution
        world_point[1] = self.map_origin[1] + map_point[1] * self.map_resolution
        return world_point

    def transform_camera_to_map(self, camera_point, base_to_map_translation, base_to_map_rotation):
        base_to_map_matrix = self.create_transformation_matrix(base_to_map_translation, self.euler_to_rotation_matrix(*base_to_map_rotation))
        camera_to_map_matrix = np.dot(base_to_map_matrix, self.camera_to_base_matrix)
        map_point = self.transform_coordinates(camera_point, camera_to_map_matrix)
        return self.map_to_world(map_point[:2])
