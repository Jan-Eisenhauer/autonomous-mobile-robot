import math
from math import cos, sin

from robot_state import RobotState

GRID_SIZE = 0.25


def _polar2cartesian(laser_range, angle, position):
    x = laser_range * cos(angle) + position[0]
    y = laser_range * sin(angle) + position[1]
    return x, y


def position2grid(position):
    # type: ((float, float)) -> (float, float)
    x = _coord2grid(position[0])
    y = _coord2grid(position[1])
    return x, y


def _coord2grid(coord):
    translated_coord = math.floor(coord / GRID_SIZE)
    grid_coord = translated_coord * GRID_SIZE + GRID_SIZE * 0.5
    return grid_coord


class Grid:
    def __init__(self):
        self.obstacles = set()

    def update(self, robot_state):
        # type: (RobotState) -> None
        """ Update the grid obstacles.

        Args:
            robot_state: The robot state.
        """
        position = robot_state.proximal_position
        rotation = robot_state.proximal_rotation
        laser_scan = robot_state.laser_scan

        for i, laser_range in enumerate(laser_scan.ranges):
            if math.isinf(laser_range):
                continue

            angle = laser_scan.angle_min + (i * laser_scan.angle_increment) + rotation

            obstacle_hit_position = _polar2cartesian(laser_range, angle, position)
            obstacle_grid_position = position2grid(obstacle_hit_position)
            self.obstacles.add(obstacle_grid_position)
