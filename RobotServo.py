class RobotServo:
    def __init__(self, index, k, b):
        self.index = index
        self.k = k
        self.b = b

    def ToTarget(self, degree):
        print([self.index, degree, int(degree * self.k + self.b)])
        return [self.index, int(degree * self.k + self.b)]

