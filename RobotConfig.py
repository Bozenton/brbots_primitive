from math import *
import numpy as np

#机器人各组件长度
# 单位：mm
LinkLength = [90, 97, 45]

LeftLeg_init = [472, 588, 692, 389, 475, 498]
RighLeg_init = [494, 600, 295, 610, 535, 501]
# def Ctrl2Angle(ctrl_info):
#     angle = ctrl_info * 240 / 1000
#     return angle

LeftLeg_init_info = np.array(LeftLeg_init) * 240 / 1000
RighLeg_init_info = np.array(RighLeg_init) * 240 / 1000


#机器人舵机转化比标定

