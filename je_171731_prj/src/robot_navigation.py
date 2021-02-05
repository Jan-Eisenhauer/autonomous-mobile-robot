from goal import GoalPool
from robot_control import RobotControl
from robot_state import RobotState


class RobotNavigation:
    def __init__(self):
        self._robot_state = RobotState()
        self._robot_control = RobotControl()
        self._goal_pool = GoalPool()

    def update(self):  # type: () -> None
        if not self._robot_state.received_all_data():
            return
