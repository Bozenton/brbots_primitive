# from servolist import init_servolist, go_servolist, walk_servolist, stop_servolist, turnright_servolist, turnleft_servolist, sideright_servolist, sideleft_servolist,backup_servolist
from servolist import act_dict
from ServoController import ServoController
import time
from math import *

controller = ServoController(9600)

def servo_act(chose):
    actlist = act_dict[chose]
    num = len(actlist)

    step = 4
    for j in range(num):
        moveTime = round(actlist[j][0])
        indexAndTargetList = actlist[j][1]
        print('=======================================================')
        print('t =', moveTime, 'actlist =', indexAndTargetList)
        print('=======================================================')
        for i in range(ceil(len(indexAndTargetList) / step)):
            controller.Cmd_SERVO_MOVE(moveTime, indexAndTargetList[i * step:i * step + step])
            time.sleep(0.001)

        time.sleep(moveTime/1000)   # 暂停等待舵机执行

        # print('t =', actlist[i][0], 'actlist =', actlist[i][1])




if __name__ == '__main__':
    # servo_act('test')
    servo_act('init_stand_up')
    time.sleep(1)
    servo_act('go')
    servo_act('walk')
    servo_act('stop')

    # time.sleep(2)
    #
    # servo_act('kick')
    #
    # time.sleep(1)
    #
    # servo_act('turnright')
    # servo_act('turnright')
    # servo_act('turnright')
    # servo_act('turnright')
    #
    # servo_act('turnleft')
    # servo_act('turnleft')
    # servo_act('turnleft')
    # servo_act('turnleft')
    #
    # servo_act('sideright')
    # servo_act('sideright')
    # servo_act('sideright')
    # servo_act('sideright')
    #
    # servo_act('sideleft')
    # servo_act('sideleft')
    # servo_act('sideleft')
    # servo_act('sideleft')
    #
    # servo_act('backup')
    # servo_act('frontup')



