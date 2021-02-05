from goal import Goal

from robot_state import RobotState


class GoalSelector:
    def select_goal(self, goals, robot_state):
        # type: (list, RobotState) -> Goal
        min_distance_reward = None
        nearest_goal = None
        for goal in list(goals):
            distance_sqrt = goal.distance_sqrt(robot_state.exact_position)
            distance_reward = distance_sqrt * (1.0 / goal.reward)
            if min_distance_reward is None or min_distance_reward > distance_reward:
                min_distance_reward = distance_reward
                nearest_goal = goal

        return nearest_goal
