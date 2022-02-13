from ServoController import ServoController
import time
from math import *

class RobotRobot:
    def __init__(self, body):
        # self.leftLeg = leftLeg
        # self.rightLeg = rightLeg
        self.body = body
        self.controller = ServoController(9600)

    def Action(self, moveTime, indexAndTargetList):
        step = 4
        for i in range(ceil(len(indexAndTargetList) / step)):
            self.controller.Cmd_SERVO_MOVE(moveTime, indexAndTargetList[i*step:i*step+step])
            time.sleep(0.001)


if __name__ == '__main__':
    test_robot = RobotRobot(1)
    test_robot.Action(3000, [[8, 500]])


