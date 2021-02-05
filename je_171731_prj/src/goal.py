import rospy
from goal_publisher.msg import PointArray

GOALS_TOPIC = "/goals"


class GoalPool:
    def __init__(self):
        self.goals = set()
        rospy.Subscriber(GOALS_TOPIC, PointArray, self._goals_callback)

    def _goals_callback(self, point_array):
        for published_goal in point_array.goals:
            goal = Goal(published_goal.x, published_goal.y, published_goal.reward)
            if not self.goals.__contains__(goal):
                self.goals.add(goal)


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
