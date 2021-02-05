import rospy

from goal import GoalPool
from goal_selector import GoalSelector
from grid import Grid
from marker_drawer import MarkerDrawer
from path_finder import PathFinder
from robot_control import RobotControl
from robot_state import RobotState


class RobotNavigation:
    def __init__(self):
        self._robot_state = RobotState()
        self._robot_control = RobotControl()
        self._goal_pool = GoalPool()
        self._grid = Grid()
        self._goal_selector = GoalSelector()
        self._path_finder = PathFinder()
        self._marker_drawer = MarkerDrawer()
        self._current_goal = None

    def update(self):  # type: () -> None
        if not self._robot_state.received_all_data():
            return

        self._grid.update(self._robot_state)
        self._marker_drawer.draw_obstacles(self._grid.obstacles, self._robot_state)

        self._goal_pool.check_goals(self._robot_state)

        new_goal = self._goal_selector.select_goal(self._goal_pool.get_uncollected_goals(), self._robot_state)
        if new_goal is not self._current_goal and new_goal is not None:
            rospy.loginfo("Target: (%s %s), reward=%s" % (new_goal.x, new_goal.y, new_goal.reward))
        self._current_goal = new_goal

        self._marker_drawer.draw_goals(self._current_goal, self._goal_pool.goals, self._robot_state)

        # stop robot if no goal is selected
        if self._current_goal is None:
            self._robot_control.stop()
            return

        # find the path to the goal
        path = self._path_finder.find_path(self._grid.obstacles, self._robot_state.proximal_position,
                                           (self._current_goal.x, self._current_goal.y))

        # check if path was found
        if len(path) <= 1:
            self._robot_control.stop()
            return

        target_position = path[1]
        # TODO - navigate robot towards target_position
