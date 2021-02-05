from goal import GoalPool
from marker_drawer import MarkerDrawer
from robot_control import RobotControl
from robot_state import RobotState


class RobotNavigation:
    def __init__(self):
        self._robot_state = RobotState()
        self._robot_control = RobotControl()
        self._goal_pool = GoalPool()
        self._marker_drawer = MarkerDrawer()

    def update(self):  # type: () -> None
        if not self._robot_state.received_all_data():
            return

        self._goal_pool.check_goals(self._robot_state)

        self._marker_drawer.draw_goals(self._goal_pool.goals, self._robot_state)
