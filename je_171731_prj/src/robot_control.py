from enum import IntEnum

import rospy
from geometry_msgs.msg import Twist, Vector3

VELOCITY_TOPIC = "/cmd_vel"
MOVEMENT_SPEED = 0.3
STEERING_SPEED = 0.5


class RobotControl:
    def __init__(self):
        self._velocity_publisher = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=4)

    def move(self, movement, steering):
        # type: (Movement, Steering) -> None
        """ Moves the robot in the direction of movement and turns depending on steering.

        Args:
            movement: The movement to apply to the robot.
            steering: The steering to apply to the robot.
        """
        rospy.logdebug("Move robot %s %s", movement, steering)

        move = movement * MOVEMENT_SPEED
        steer = steering * STEERING_SPEED
        self._publish(move, steer)

    def stop(self):  # type: () -> None
        """ Stops the robot. """
        self.move(Movement.STANDING, Steering.STRAIGHT)

    def _publish(self, movement, steering):
        # type: (float, float) -> None
        linear_velocity = Vector3(movement, 0.0, 0.0)
        angular_velocity = Vector3(0.0, 0.0, steering)
        twist = Twist(linear=linear_velocity, angular=angular_velocity)
        self._velocity_publisher.publish(twist)


class Movement(IntEnum):
    BACKWARD = -1
    STANDING = 0
    FORWARD = 1


class Steering(IntEnum):
    RIGHT = -1
    STRAIGHT = 0
    LEFT = 1
