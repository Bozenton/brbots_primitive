"""
BZT
新的运动学解测试
可用于两种机器人
"""

import numpy as np
import time
import copy
from math import *
from RobotLegs import RobotLegs
from RobotServo import RobotServo
from RobotRobot import RobotRobot
# from RobotConfig import LinkLength

# 机器人腿部各个杆件的长度
# 此处为黑色机器人的参数
LinkLength = [0, 90, 97, 0, 45]

param_angle2info = 1 / 0.24     # 角度与舵机转角的换算
leg_dis = 78.5                  # 两腿根部的距离
gch = 0                         # 重心距离两腿根部的高度

Servos = [
            # RobotServo(6, -param_angle2info, 500),
            RobotServo(10, -param_angle2info, 500),
            RobotServo(9, param_angle2info, 500),
            RobotServo(8, -param_angle2info, 500),
            RobotServo(6, -param_angle2info, 500),
            RobotServo(7, param_angle2info, 500),

            # RobotServo(12, param_angle2info, 500),
            RobotServo(5, -param_angle2info, 500),
            RobotServo(4, -param_angle2info, 500),
            RobotServo(3, -param_angle2info, 500),
            RobotServo(1, param_angle2info, 500),
            RobotServo(2, param_angle2info, 500),

            RobotServo(11, -param_angle2info, 1000),#右大臂，正向前
            RobotServo(12, -param_angle2info, 950),#左大臂，负向前
            RobotServo(13, param_angle2info, 400),#左右小臂，范围为屈臂到与大臂平行
            RobotServo(14, param_angle2info, 400)
          ]
body = RobotLegs(Servos, LinkLength, leg_dis, gch)

# 定义机器人
test_robot = RobotRobot(body)


def op(robot_obj, dt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_upper , arm_lower , Upperfoot=0):
    """
    op: short for operate
    输入机器人重心的绝对坐标、重心的转角、两个脚的绝对坐标
    以及执行时间
    即可执行动作
    :param robot_obj: RobotRobot类
    :param dt: 执行时间
    :param gcx:
    :param gcy:
    :param gcz:
    :param gcroll:
    :param gcpitch:
    :param gcyaw:
    :param lx:
    :param ly:
    :param lz:
    :param rx:
    :param ry:
    :param rz:
    arm_upper:大臂转角，向前为正，向后为负，-90————90
    arm_upper:小臂转角，屈臂为正，直臂为负，-50————90
    :return: 各个舵机的转角
    """

    GravityCenterGeneral = np.array([[gcx], [gcy], [gcz]])
    LeftLegTipGeneral = np.array([[lx], [ly], [lz]])
    RightLegTipGeneral = np.array([[rx], [ry], [rz]])
    GCRollPitchYaw = np.array([gcroll, gcpitch, gcyaw])
    ArmAngle=np.array([arm_upper, arm_lower])

    list = test_robot.body.ToTarget(GravityCenterGeneral, LeftLegTipGeneral, RightLegTipGeneral, GCRollPitchYaw,ArmAngle)
    if(Upperfoot==1):
        list[3]=[6,200]#6
        list[8]=[1,800]#1
        #list[2]=[8,500]
        #list[7]=[3,500]
    if (Upperfoot == 2):
        list[3] = [6, 200]  # 6
        list[8] = [1, 800]  # 1
        list[1]=[9,500]
        list[6]=[4,500]

    if (Upperfoot == 3):
        list[3] = [6, 200]  # 6
        list[8] = [1, 800]  # 1
        list[1]=[9,500]
        list[6]=[4,500]

    if (Upperfoot == 4):
        list[3] = [6, 600]  # 6
        list[8] = [1, 400]  # 1
        list[2]=[8,300]
        list[7]=[3,300]
        list[1]=[9,300]
        list[6]=[4,700]

    if (Upperfoot == 5):
        list[3] = [6, 500]  # 6
        list[8] = [1, 500]  # 1
        list[1]=[9,300]
        list[6]=[4,700]
        list[2]=[8,200]
        list[7]=[3,200]

    if (Upperfoot == 6):
        list[3] = [6, 250]  # 6
        list[8] = [1, 750]  # 1
        list[1]=[9,300]
        list[6]=[4,700]
        list[2]=[8,200]
        list[7]=[3,200]

    if (Upperfoot == 7):
        list[3] = [6, 300]  # 6
        list[8] = [1, 700]  # 1
        list[1]=[9,100]
        list[6]=[4,900]
        list[2]=[8,400]
        list[7]=[3,400]

    if (Upperfoot == 8):
        list[3] = [6, 300]  # 6
        list[8] = [1, 700]  # 1
        list[1]=[9,100]
        list[6]=[4,900]
        list[2]=[8,400]
        list[7]=[3,400]

    if (Upperfoot == 9):
        list[3] = [6, 700]  # 6
        list[8] = [1, 300]  # 1

    # print('-------------------------------------------------')
    # print('动作组--------------------------------------------')
    # print(list)
    # print('时间----------------------------------------------')
    # print(dt)
    # print('-------------------------------------------------')
    # print([dt, list])
    robot_obj.Action(dt, list)
    time.sleep(dt / 1000)

    return list


def init_standup(robot_obj):
    """
    初始化，立正
    :param robot_obj: RobotRobot类
    :return:
    """
    op(robot_obj, 2500,
       -5, 0, 200,
       0, pi/60, 0,
       0, 78.5/2+15, 0,
       0, -78.5/2-15, 0,
       0,-50)


if __name__ == '__main__':
    list = []
    init_standup(test_robot)

    temp = op(test_robot, 800,
       -5, 0, 200,
       0, pi/60, 0,
       -50, 78.5/2+15, 60,
       -50, -78.5/2-15, 60,
       -90,0,Upperfoot=9)
    list.append([800, temp])

    temp = op(test_robot, 800,
       -5, 0, 200,
       0, pi/60, 0,
       -50, 78.5/2+15, 10,
       -50, -78.5/2-15, 10,
       -90,0,Upperfoot=9)
    list.append([800, temp])

    temp = op(test_robot, 1000,
       -5, 0, 200,
       0, pi/60, 0,
       -50, 78.5/2+15, 10,
       -50, -78.5/2-15, 10,
       -90,90,Upperfoot=9)
    list.append([1000, temp])


    temp = op(test_robot, 1000,
       -5, 0, 200,
       0, pi/60, 0,
       -60, 78.5/2+15, 60,
       -60, -78.5/2-15, 60,
       -50,90,Upperfoot=9)
    list.append([1000, temp])


    time.sleep(1)
    list.append([1000, temp])

    temp = op(test_robot, 1200,
       -5, 0, 200,
       0, pi / 60, 0,
       0, 78.5 / 2 + 15, 0,
       0, -78.5 / 2 - 15, 0,
       0, 90)
    list.append([1200, temp])

    print('backup_servolist =', list)