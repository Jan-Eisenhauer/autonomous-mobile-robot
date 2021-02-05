import rospy
from goal_publisher.msg import PointArray

from robot_state import RobotState

GOALS_TOPIC = "/goals"
GOAL_RADIUS_SQRT = 0.5 * 0.5


class GoalPool:
    def __init__(self):
        self.goals = set()
        rospy.Subscriber(GOALS_TOPIC, PointArray, self._goals_callback)

    def _goals_callback(self, point_array):
        for published_goal in point_array.goals:
            goal = Goal(published_goal.x, published_goal.y, published_goal.reward)
            if not self.goals.__contains__(goal):
                self.goals.add(goal)

    def get_total_collected_reward(self):
        # type: () -> float
        """ Calculates the sum of the collected goal rewards.

        Returns: The total collected reward.
        """
        return sum(goal.reward for goal in list(self.goals) if goal.collected)

    def check_goals(self, robot_state):
        # type: (RobotState) -> None
        for goal in list(self.goals):
            if goal.collected:
                continue

            distance_sqrt = goal.distance_sqrt(robot_state.exact_position)
            if distance_sqrt <= GOAL_RADIUS_SQRT:
                goal.collected = True
                print("Goal (%s %s) collected with reward=%s" % (goal.x, goal.y, goal.reward))
                print("Total reward: %s" % self.get_total_collected_reward())
                continue


class Goal:
    def __init__(self, x, y, reward):
        self.x = x
        self.y = y
        self.reward = reward
        self.collected = False

    def __eq__(self, other):
        if not isinstance(other, Goal):
            return False

        return self.x == other.x and self.y == other.y and self.reward == other.reward

    def __hash__(self):
        return hash((self.x, self.y, self.reward))

    def distance_sqrt(self, position):
        # type: ((float, float)) -> float
        """ Calculates the squared distance from this goal to the given position.

        Args:
            position: The other position to calculate the distance to.
        Returns: The squared distance to the position.
        """
        return (position[0] - self.x) ** 2 + (position[1] - self.y) ** 2
