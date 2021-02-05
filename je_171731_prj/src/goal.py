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
