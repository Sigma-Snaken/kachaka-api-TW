from math import atan2, cos, pi, sin
from typing import Tuple

import numpy as np

from ..generated.kachaka_api_pb2 import Map, Pose, Quaternion


def calculate_yaw_from_quaternion(q: Quaternion) -> float:
    # Kachaka 的姿態僅包含繞 Z 軸的旋轉，因此忽略其他軸進行計算
    yaw = atan2(q.z, q.w) * 2
    if abs(yaw) > pi:
        yaw -= np.sign(yaw) * 2 * pi
    return yaw


def calculate_2d_transform_matrix(
    tx: float, ty: float, theta: float = 0
) -> np.ndarray:
    c, s = cos(theta), sin(theta)
    return np.array(((c, -s, tx), (s, c, ty), (0, 0, 1)))


def calculate_2d_scale_matrix(sx: float, sy: float) -> np.ndarray:
    return np.array(((sx, 0, 0), (0, sy, 0), (0, 0, 1)))


class MapImage2DGeometry:
    def __init__(self, png_map: Map) -> None:
        resolution = png_map.resolution
        origin = png_map.origin
        self._map_to_image_origin = (
            calculate_2d_transform_matrix(origin.x, origin.y, origin.theta)
            @ calculate_2d_scale_matrix(resolution, -resolution)
            @ calculate_2d_transform_matrix(0.5, -png_map.height + 0.5)
        )
        self._image_origin_to_map = np.linalg.inv(self._map_to_image_origin)

    def calculate_robot_pose_matrix_in_pixel(
        self, robot_pose: Pose
    ) -> np.ndarray:
        """將機器人姿態轉換為影像上姿態的矩陣

        :param robot_pose: 機器人姿態 [m, m, rad]
        :returns 3x3 矩陣
        """
        return self._image_origin_to_map @ calculate_2d_transform_matrix(
            robot_pose.x, robot_pose.y, robot_pose.theta
        )

    def calculate_robot_pose_matrix_from_pixel(
        self, pixel_xy: Tuple[float, float], angle: float = 0
    ) -> np.ndarray:
        """將影像上的姿態轉換為機器人姿態的矩陣

        :param pixel_xy: 從影像原點的位置 [px, px]
        :param angle: 以逆時針為正的角度 [radian]
        :returns 3x3 矩陣
        """
        return self._map_to_image_origin @ calculate_2d_transform_matrix(
            *pixel_xy, angle
        )
