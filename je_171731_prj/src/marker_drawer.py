from math import cos, sin

import rospy
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from goal import GOAL_RADIUS
from robot_state import RobotState

VISUALIZATION_MARKER_TOPIC = "/visualization_marker"

GOAL_NAMESPACE = "goals"
GOAL_SCALE = Vector3(GOAL_RADIUS, GOAL_RADIUS, GOAL_RADIUS)
COLLECTED_GOAL_COLOR = ColorRGBA(0, 1, 0, 1)
UNCOLLECTED_GOAL_COLOR = ColorRGBA(0, 0, 1, 1)


def _global2local_point(position, global_position, rotation):
    relative_x = position[0] - global_position[0]
    relative_y = position[1] - global_position[1]
    transformed_x = cos(-rotation) * relative_x - sin(-rotation) * relative_y
    transformed_y = cos(-rotation) * relative_y + sin(-rotation) * relative_x
    return Point(transformed_x, transformed_y, 0)


class MarkerDrawer:
    def __init__(self):
        self._visualization_publisher = rospy.Publisher(VISUALIZATION_MARKER_TOPIC, Marker, queue_size=4)

    def draw_goals(self, goals, robot_state):
        # type: (list, RobotState) -> None
        points = []
        colors = []

        for goal in list(goals):
            point = _global2local_point((goal.x, goal.y), robot_state.exact_position, robot_state.exact_rotation)
            points.append(point)

            if goal.collected:
                color = COLLECTED_GOAL_COLOR
            else:
                color = UNCOLLECTED_GOAL_COLOR

            colors.append(color)

        self._draw_sphere_list(0, GOAL_NAMESPACE, GOAL_SCALE, points, colors)

    def _draw_sphere_list(self, uid, namespace, scale, points, colors):
        marker = Marker()
        # Header
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()

        # Body
        marker.ns = namespace
        marker.id = uid
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale = scale
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.points = points
        marker.colors = colors

        self._visualization_publisher.publish(marker)
