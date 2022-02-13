"""
此文件用于输出各个动作的舵机转角
"""

import numpy as np
import pandas as pd
import time
from RobotLegs import RobotLegs
from RobotServo import RobotServo
from RobotRobot import RobotRobot

# =========================== 机器人参数 ===========================
# 机器人腿部各个杆件的长度
# 此处为黑色机器人的参数
LinkLength = [0, 90, 97, 0, 45]

param_angle2info = 1 / 0.24     # 角度与舵机转角的换算
leg_dis = 78.5                  # 两腿根部的距离
gch = 0                         # 重心距离两腿根部的高度

Servos = [
            RobotServo(10, -param_angle2info, 500),
            RobotServo(9, param_angle2info, 490),
            RobotServo(8, -param_angle2info, 530),
            RobotServo(6, -param_angle2info, 520),
            RobotServo(7, param_angle2info, 520),

            RobotServo(5, -param_angle2info, 500),
            RobotServo(4, -param_angle2info, 515),
            RobotServo(3, param_angle2info, 505),
            RobotServo(1, param_angle2info, 510),
            RobotServo(2, param_angle2info, 500),

            RobotServo(11, -param_angle2info, 510),#右大臂，正向前
            RobotServo(12, -param_angle2info, 515),#左大臂，负向前
            RobotServo(13, -param_angle2info, 510),#左右小臂，范围为屈臂到与大臂平行
            RobotServo(14, param_angle2info, 500)
          ]
body = RobotLegs(Servos, LinkLength, leg_dis, gch)

# 定义机器人
test_robot = RobotRobot(body)

# =========================== 基本的函数 ===========================
def op(robot_obj, dt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_upper, arm_lower):
    """
    op: short for operate
    按照绝对坐标以及执行时间，执行动作
    输入机器人重心的绝对坐标、重心的转角、两个脚的绝对坐标
    以及执行时间
    即可执行动作
    :param robot_obj: RobotRobot类
    :param dt: 执行时间
    :param gcx: 重心的x坐标
    :param gcy: 重心的y坐标
    :param gcz: 重心的z坐标
    :param gcroll: 重心的roll
    :param gcpitch: 重心的pitch
    :param gcyaw: 重心的yaw
    :param lx: 左脚的x坐标
    :param ly: 左脚的y坐标
    :param lz: 左脚的z坐标
    :param rx: 右脚的x坐标
    :param ry: 右脚的y坐标
    :param rz: 右脚的z坐标
    :return: 各个舵机的转角
    """

    GravityCenterGeneral = np.array([[gcx], [gcy], [gcz]])
    LeftLegTipGeneral = np.array([[lx], [ly], [lz]])
    RightLegTipGeneral = np.array([[rx], [ry], [rz]])
    GCRollPitchYaw = np.array([gcroll, gcpitch, gcyaw])
    ArmAngle = np.array([arm_upper, arm_lower])

    list = test_robot.body.ToTarget(GravityCenterGeneral, LeftLegTipGeneral,
                                    RightLegTipGeneral, GCRollPitchYaw, ArmAngle)

    robot_obj.Action(dt, list)
    time.sleep(dt / 1000)

    return list

def scan_csv(path):
    """
    读取动作列表
    动作列表中的值为绝对坐标
    注意csv文件中的第一行读不到
    :param path: 文件路径
    :return:
    """
    df = pd.read_csv(path)
    return df.values

# =========================== 动作 ===========================
# 读取动作列表
golist = scan_csv('PositionList/golist.csv')
walklist = scan_csv('PositionList/walklist.csv')
stoplist = scan_csv('PositionList/stoplist.csv')
kicklist = scan_csv('PositionList/kickball.csv')
turnrightlist = scan_csv('PositionList/turnrightlist.csv')
turnleftlist = scan_csv('PositionList/turnleftlist.csv')

siderightlist = scan_csv('PositionList/sideright.csv')
sideleftlist = scan_csv('PositionList/sideleft.csv')

testlist = scan_csv('testlist.csv')

dt = 50     # 每一帧50ms
# ================== 立正部分 ==================
def init_standup():
    # 初始化立正
    list = []
    tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = golist[0]
    temp = op(test_robot, 1000, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
    list.append([1000, temp])
    return list

# ================== 直行部分 ==================
"""
直行部分分别为三种动作：
    1. 起步：从立正到迈出第一步
    2. 中步：两腿交替走
    3. 止步：从走路过渡到立正
"""

# --------- 起步 ---------
def go():
    list = []
    for i in range(len(golist)):
        tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = golist[i]
        tt = round(tt)
        temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
        list.append([tt, temp])
    return list

# --------- 中步 ---------
def walk(cnt):
    list = []
    j = 0
    while j < cnt:
        for i in range(len(walklist)):
            tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = walklist[i]
            tt = round(tt)
            temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
            list.append([tt, temp])
        j = j + 1
    return list

# --------- 止步 ---------
def stop():
    list = []
    for i in range(len(stoplist)):
        tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = stoplist[i]
        tt = round(tt)
        temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
        list.append([tt, temp])
    return list


# ================== 踢球部分 ==================
def kick():
    # 初始化立正
    list = []

    tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = kicklist[0]
    temp = op(test_robot, 1000, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
    list.append([1000, temp])

    time.sleep(1)
    list.append([1000, temp])

    for i in range(len(kicklist)):
        tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = kicklist[i]
        tt = round(tt)
        temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
        list.append([tt, temp])

    tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = kicklist[0]
    temp = op(test_robot, 1000, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
    list.append([1000, temp])

    return list

# ================== 转弯部分 ==================
def turn_right(cnt):
    list = []
    j = 0
    while j < cnt:
        for i in range(len(turnrightlist)):
            tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = turnrightlist[i]
            tt = round(tt)
            temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
            list.append([tt, temp])
        j = j + 1
    return list


def turn_left(cnt):
    list = []
    j = 0
    while j < cnt:
        for i in range(len(turnleftlist)):
            tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = turnleftlist[i]
            tt = round(tt)
            temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
            list.append([tt, temp])
        j = j + 1
    return list


# ================== 侧向横着走 ==================
def side_right(cnt):
    list = []
    j = 0
    while j < cnt:
        for i in range(len(siderightlist)):
            tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = siderightlist[i]
            tt = round(tt)
            temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
            list.append([tt, temp])
        j = j + 1
    return list

def side_left(cnt):
    list = []
    j = 0
    while j < cnt:
        for i in range(len(sideleftlist)):
            tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = sideleftlist[i]
            tt = round(tt)
            temp = op(test_robot, tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)
            list.append([tt, temp])
        j = j + 1
    return list


# ================== 爬起来部分 ==================
# 暂时么得


def testaction():
    for i in range(len(testlist)):
        tt, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low = testlist[i]
        tt = round(tt)
        op(test_robot, 500, gcx, gcy, gcz, gcroll, gcpitch, gcyaw, lx, ly, lz, rx, ry, rz, arm_up, arm_low)

# *************************************************************
# 将位置输出成舵机转角
# *************************************************************
def pos2servo():
    init_servolist = init_standup()
    go_servolist = go()
    walk_servolist = walk(1)
    stop_servolist = stop()

    turnright_servolist = turn_right(1)
    turnleft_servolist = turn_left(1)

    sideright_servolist = side_right(1)
    sideleft_servolist = side_left(1)

    print('init_servolist =', init_servolist)

    print('go_servolist =', go_servolist)
    print('walk_servolist =', walk_servolist)
    print('stop_servolist =', stop_servolist)

    print('turnright_servolist =', turnright_servolist)
    print('turnleft_servolist =', turnleft_servolist)

    print('sideright_servolist =', sideright_servolist)
    print('sideleft_servolist =', sideleft_servolist)


# demo:
if __name__ == '__main__':
    init_servolist = init_standup()
    time.sleep(2)
    #
    go_servolist = go()
    walk_servolist = walk(1)
    stop_servolist = stop()

    turnright_servolist = turn_right(1)
    turnleft_servolist = turn_left(1)

    sideright_servolist = side_right(1)
    sideleft_servolist = side_left(1)

    kick_servolist = kick()


    print('init_servolist =', init_servolist)

    print('go_servolist =', go_servolist)
    print('walk_servolist =', walk_servolist)
    print('stop_servolist =', stop_servolist)
    #
    print('turnright_servolist = ', turnright_servolist)
    print('turnleft_servolist = ', turnleft_servolist)

    print('sideright_servolist = ', sideright_servolist)
    print('sideleft_servolist = ', sideleft_servolist)

    print('kick_servolist =', kick_servolist)




    # go()
    # walk(2)
    # stop()
    # time.sleep(0.5)
    #
    # side_left(5)
    #
    # time.sleep(0.5)
    # kick()

    # time.sleep(0.5)
    # go()
    # walk(2)
    # stop()


    # side_right(10)

    # turn_right(3)

    # turn_left(3)

    # testaction()

    # go()
    # walk(1)
    # stop()
    # time.sleep(0.5)
    #
    # kick()
    #
    # time.sleep(0.5)
    # go()
    # walk(1)
    # stop()
    # time.sleep(1)
