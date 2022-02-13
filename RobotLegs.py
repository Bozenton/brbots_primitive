from math import *
import numpy as np

def cosine(a, b, c):
    """
    余弦定理解三角形，a和b为角的邻边，c为角的对边
    :param a:
    :param b:
    :param c:
    :return:
    """
    angle = acos((a**2 + b**2 - c**2) / (2 * a * b))
    return angle

def inter_angle(vec_a, vec_b):
    """
    计算两个向量的夹角
    :param vec_a:
    :param vec_b:
    :return:
    """
    if type(vec_a) is not np.ndarray or type (vec_b) is not np.ndarray:
        raise TypeError("Input vectors should be numpy.array")
    if vec_a.shape != (3, 1) or vec_b.shape != (3, 1):
        raise ValueError("Input vectors should be col vectors and in shape (3,1)")
    vec_a = vec_a.reshape(3)
    vec_b = vec_b.reshape(3)
    # 计算两向量的夹角，正数无方向
    angle_abs = np.dot(vec_a, vec_b) / (np.linalg.norm(vec_a) * np.linalg.norm(vec_b))
    # print('1********************************************')
    # print(angle_abs)
    # print('2********************************************')
    angle_abs = round(angle_abs*1000) / 1000
    # print(angle_abs)
    # print('3********************************************')
    angle_abs = acos(angle_abs)
    # print(angle_abs)
    # print('4********************************************')
    # 计算两向量的方向（先算叉积，然后叉积的结果与正方向点积）
    cross_mul = np.cross(vec_a, vec_b)
    direction = np.sign(np.dot(cross_mul, [0, -1, 0]))
    # print('va', vec_a)
    # print('vb', vec_b)
    # print('cross_mul', cross_mul)
    # print('dot', np.dot(cross_mul, [0, -1, 0]))
    # 得出带方向的角度
    angle_abs = angle_abs * direction
    # print(angle_abs)
    # print('5********************************************')
    # print(" ")
    # print(" ")
    # print(" ")
    # print(" ")
    # print(" ")
    # print(" ")
    # print(" ")
    return angle_abs

class RobotLegs:
    def __init__(self, Servos, Length, leg_dis, gch):
        self.lservo_A = Servos[0]
        self.lservo_B = Servos[1]
        self.lservo_C = Servos[2]
        self.lservo_D = Servos[3]
        self.lservo_E = Servos[4]
        self.rservo_A = Servos[5]
        self.rservo_B = Servos[6]
        self.rservo_C = Servos[7]
        self.rservo_D = Servos[8]
        self.rservo_E = Servos[9]

        self.lservo_F = Servos[10]
        self.rservo_F = Servos[11]
        self.lservo_G = Servos[12]
        self.rservo_G = Servos[13] #手部电机

        self.L1 = Length[0]
        self.L2 = Length[1]
        self.L3 = Length[2]
        self.L4 = Length[3]
        self.L5 = Length[4]

        self.leg_dis = leg_dis # 两腿根部的距离
        self.gch = gch # 重心到两腿根部的距离

    def ToTarget(self, GravityCenterGeneral, LeftLegTipGeneral, RightLegTipGeneral,
                 GCRollPitchYaw, ArmAngle):
        """
        逆运动学解
        :param GravityCenterGeneral:
        :param LeftLegTipGeneral:
        :param RightLegTipGeneral:
        :param GCRollPitchYaw:
        :return:
        """
        GCroll = GCRollPitchYaw[0]
        # GCroll = 0  # 使roll为0
        GCpitch = GCRollPitchYaw[1]
        GCyaw = GCRollPitchYaw[2]

        if fabs(GCroll) > pi/4 or fabs(GCpitch) > pi/4 or fabs(GCyaw) > pi/4:
            raise ValueError("输入角度过大或者为角度值，应该输入弧度值")

        # 重心的旋转
        GCrpyM = np.array([[cos(GCpitch)*cos(GCroll), -cos(GCpitch)*sin(GCroll), sin(GCpitch) ],
                           [sin(GCpitch)*cos(GCroll)*sin(GCyaw)+sin(GCroll)*cos(GCyaw),
                            cos(GCroll)*cos(GCyaw)-sin(GCpitch)*sin(GCroll)*sin(GCyaw),
                            -cos(GCpitch)*sin(GCyaw)],
                           [sin(GCroll)*sin(GCyaw)-sin(GCpitch)*cos(GCroll)*cos(GCyaw),
                            sin(GCpitch)*sin(GCroll)*cos(GCyaw) + cos(GCroll)*sin(GCyaw),
                            cos(GCpitch)*cos(GCyaw)
                            ]])

        # 左右腿的根部的全局坐标
        LeftLegBaseGC = np.array([[0], [self.leg_dis/2], [-self.gch]])
        LeftLegBaseGeneral = np.dot(GCrpyM, LeftLegBaseGC) + GravityCenterGeneral
        RightLegBaseGC = np.array([[0], [-self.leg_dis/2], [-self.gch]])
        RightLegBaseGeneral = np.dot(GCrpyM, RightLegBaseGC) + GravityCenterGeneral

        # 约束条件：每个腿的L5杆都要始终和地面垂
        LeftLegEGeneral = LeftLegTipGeneral + [[0], [0], [self.L5]]
        RightLegEGeneral = RightLegTipGeneral + [[0], [0], [self.L5]]

        # 计算两腿E点的相对腿根部的坐标, 坐标轴方向与绝对坐标方向相同，未旋转
        LeftLegELLB = LeftLegEGeneral - LeftLegBaseGeneral
        RightLegERLB = RightLegEGeneral - RightLegBaseGeneral

        # 计算未旋转的坐标系中向量EA
        VecEaLeft = -LeftLegELLB
        VecEaRight = -RightLegERLB

        # 计算未旋转的坐标系中向量EA在yz平面上的投影
        yz_proj = np.array([[0], [1], [1]])
        VecEaYzLeft = yz_proj * VecEaLeft # "*"为对应元素相乘
        VecEaYzRight = yz_proj * VecEaRight

        # 计算未旋转的坐标系中D点的坐标
        LeftLegDLLB = LeftLegELLB + VecEaYzLeft * self.L4 / np.linalg.norm(VecEaYzLeft)
        RightLegDRLB = RightLegERLB + VecEaYzRight * self.L4 / np.linalg.norm(VecEaYzRight)

        # 计算未旋转的坐标系中D'点的坐标
        LeftLegDpLLB = LeftLegELLB + VecEaYzLeft / np.linalg.norm(VecEaYzLeft)
        RightLegDpRLB = RightLegERLB + VecEaYzRight / np.linalg.norm(VecEaYzRight)

        # === === === 以下为旋转后的坐标系 === === ===
        # 旋转矩阵
        GCrpyM_inv = np.linalg.inv(GCrpyM)

        # 计算旋转后的坐标系中E点的坐标
        LeftLegELLBRotated = np.dot(GCrpyM_inv, LeftLegELLB)
        RightLegERLBRotated = np.dot(GCrpyM_inv, RightLegERLB)

        # 计算旋转后的坐标系中D点的坐标
        LeftLegDLLBRotated = np.dot(GCrpyM_inv, LeftLegDLLB)
        RightLegDRLBRotated = np.dot(GCrpyM_inv, RightLegDRLB)

        # 计算旋转后的坐标系中D'点的坐标
        LeftLegDpLLBRotated = np.dot(GCrpyM_inv, LeftLegDpLLB)
        RightLegDpRLBRotated = np.dot(GCrpyM_inv, RightLegDpRLB)


        # 向量AD
        VecAdLeft = LeftLegDLLBRotated
        VecAdRight = RightLegDRLBRotated

        # 向量AD在yz平面上的投影
        VecAdYzLeft = yz_proj * VecAdLeft
        VecAdYzRight = yz_proj * VecAdRight

        # 向量AB
        VecAbLeft = VecAdYzLeft * self.L1 / np.linalg.norm(VecAdYzLeft)
        VecAbRight = VecAdYzRight * self.L1 / np.linalg.norm(VecAdYzRight)

        # 向量BD
        VecBdLeft = VecAdLeft - VecAbLeft
        VecBdRight = VecAdRight - VecAbRight

        # BD的长度求出来，△BCD可解
        DbdLL = np.linalg.norm(VecBdLeft)
        DbdRL = np.linalg.norm(VecBdRight)

        angleBLL = cosine(self.L2, DbdLL, self.L3)
        angleCLL = cosine(self.L2, self.L3, DbdLL)
        angleDLL = cosine(self.L3, DbdLL, self.L2)
        angleBRL = cosine(self.L2, DbdRL, self.L3)
        angleCRL = cosine(self.L2, self.L3, DbdRL)
        angleDRL = cosine(self.L3, DbdRL, self.L2)

        # 计算向量AB和BD的夹角
        angleAbBdLeft = inter_angle(VecAdYzLeft, VecBdLeft)
        angleAbBdRight = inter_angle(VecAdYzRight, VecBdRight)

        # 向量D'E
        VecDeLeft = LeftLegELLBRotated - LeftLegDpLLBRotated
        VecDeRight = RightLegERLBRotated - RightLegDpRLBRotated

        # 计算向量DE和BD的夹角
        angleDeBdLeft = inter_angle(VecDeLeft, VecBdLeft)
        angleDeBdRight = inter_angle(VecDeRight, VecBdRight)

        # === === === 计算各个舵机的角度 === === ===
        LeftLegServoA = -atan(LeftLegELLBRotated[1]/LeftLegELLBRotated[2])
        RightLegServoA = -atan(RightLegERLBRotated[1]/RightLegERLBRotated[2])

        LeftLegServoB = -(angleAbBdLeft + angleBLL)
        RightLegServoB = -(angleAbBdRight + angleBRL)

        LeftLegServoC = pi - angleCLL
        RightLegServoC = pi - angleCRL

        LeftLegServoD = -(angleDLL - angleDeBdLeft)
        RightLegServoD = -(angleDRL - angleDeBdRight)

        LeftLegServoE = atan(LeftLegELLB[1] / LeftLegELLB[2])
        RightLegServoE = atan(RightLegERLB[1] / RightLegERLB[2])

        # 返回舵机信息
        indexAndTargetList = []
        indexAndTargetList.append(self.lservo_A.ToTarget(LeftLegServoA * 180 / pi))
        indexAndTargetList.append(self.lservo_B.ToTarget(LeftLegServoB * 180 / pi))
        indexAndTargetList.append(self.lservo_C.ToTarget(LeftLegServoC * 180 / pi))
        indexAndTargetList.append(self.lservo_D.ToTarget(LeftLegServoD * 180 / pi))
        indexAndTargetList.append(self.lservo_E.ToTarget(LeftLegServoE * 180 / pi))

        indexAndTargetList.append(self.rservo_A.ToTarget(RightLegServoA * 180 / pi))
        indexAndTargetList.append(self.rservo_B.ToTarget(RightLegServoB * 180 / pi))
        indexAndTargetList.append(self.rservo_C.ToTarget(RightLegServoC * 180 / pi))
        indexAndTargetList.append(self.rservo_D.ToTarget(RightLegServoD * 180 / pi))
        indexAndTargetList.append(self.rservo_E.ToTarget(RightLegServoE * 180 / pi))

        indexAndTargetList.append(self.lservo_F.ToTarget(ArmAngle[0]))
        indexAndTargetList.append(self.rservo_F.ToTarget(ArmAngle[0]))
        indexAndTargetList.append(self.lservo_G.ToTarget(ArmAngle[1]))
        indexAndTargetList.append(self.rservo_G.ToTarget(ArmAngle[1]))

        return indexAndTargetList
