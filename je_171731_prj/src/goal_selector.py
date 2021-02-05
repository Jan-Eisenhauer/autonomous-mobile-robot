from goal import Goal

from robot_state import RobotState


class GoalSelector:
    def select_goal(self, goals, robot_state):
        # type: (list, RobotState) -> Goal
        min_distance_sqrt = None
        nearest_goal = None
        for goal in list(goals):
            distance_sqrt = goal.distance_sqrt(robot_state.exact_position)
            if min_distance_sqrt is None or min_distance_sqrt > distance_sqrt:
                min_distance_sqrt = distance_sqrt
                nearest_goal = goal

        return nearest_goal
