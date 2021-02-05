from goal import GoalPool
from robot_control import RobotControl


class RobotNavigation:
    def __init__(self):
        self._robot_control = RobotControl()
        self._goal_pool = GoalPool()

    def update(self):  # type: () -> None
        pass
