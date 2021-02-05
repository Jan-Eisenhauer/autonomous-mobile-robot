import rospy

from goal import GoalPool
from goal_selector import GoalSelector
from marker_drawer import MarkerDrawer
from robot_control import RobotControl
from robot_state import RobotState


class RobotNavigation:
    def __init__(self):
        self._robot_state = RobotState()
        self._robot_control = RobotControl()
        self._goal_pool = GoalPool()
        self._goal_selector = GoalSelector()
        self._marker_drawer = MarkerDrawer()
        self._current_goal = None

    def update(self):  # type: () -> None
        if not self._robot_state.received_all_data():
            return

        self._goal_pool.check_goals(self._robot_state)

        new_goal = self._goal_selector.select_goal(self._goal_pool.get_uncollected_goals(), self._robot_state)
        if new_goal is not self._current_goal and new_goal is not None:
            rospy.loginfo("Target: (%s %s), reward=%s" % (new_goal.x, new_goal.y, new_goal.reward))
        self._current_goal = new_goal

        self._marker_drawer.draw_goals(self._goal_pool.goals, self._robot_state)
