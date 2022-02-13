# 机器人运动

本部分为机器人运动相关的控制。

有立正（初始化）、起步、中步、止步、左转、右转、向左横着走、向右横着走、踢球、从前面爬起、从后面爬起等动作。

每一个动作都由若干个动作帧组成，每一帧可能会有不同的执行时间，将这一系列动作帧按照相应的时间执行，即可得到一个完整的、近似连续的动作。

## 调试部分

在文件夹PositionList下的一系列csv文件中直接调参，然后在`RobotActions.py`中直接执行，机器人即可执行相应的动作，并将该动作的参数打印出来。

## 上位机直接调用

将每一个动作的调试结果（各个舵机的转角和动作每一帧的时间）复制粘贴到`servolist.py`中。

每一个动作的参数格式如下：

| 动作帧执行时间 | 舵机参数                                                     |
| -------------- | ------------------------------------------------------------ |
| 第一帧的时间   | [舵机1编号，舵机转角]，[舵机2编号，舵机转角]，[舵机3编号，舵机转角]…… |
| 第二帧的时间   | [舵机1编号，舵机转角]，[舵机2编号，舵机转角]，[舵机3编号，舵机转角]…… |
| ……             | ……                                                           |

所有动作将会储存在该文件里的字典中：

```python
act_dict = {
    'init_stand_up': init_servolist,
    'go': go_servolist,
    'walk': walk_servolist,
    'stop': stop_servolist,
    'turnright': turnright_servolist,
    'turnleft': turnleft_servolist,
    'sideright': sideright_servolist,
    'sideleft': sideleft_servolist,
    'kick': kick_servolist,
    'backup': backup_servolist,
    'forwardup': forwardup_servolist
}
```

在RobotServoAct.py中直接调用这些参数并执行即可

```python
servo_act('init_stand_up')	# 站立
time.sleep(1)

# 直走部分分为起步、中步和止步三个部分
# 起步：从立正状态开始，迈出右脚
# 中步：左脚走一步、右脚走一步
# 止步：收左脚，回到立正状态
servo_act('go')				# 起步
servo_act('walk')			# 中步
servo_act('stop')			# 止步

# 踢球
time.sleep(1)				# 建议在踢球前先立正，然后停一下，使机器人能够站稳
servo_act('kick')			# 踢球
time.sleep(1)				# 踢完球也要停一下，站稳了再执行其他动作

# 转变方向
servo_act('turnright')		# 向右转
servo_act('turnleft')		# 向左转

# 侧着走
servo_act('sideright')		# 向右横着走
servo_act('sideleft')		# 向左横着走

# 爬起来
servo_act('init_stand_up')	#建议摔倒之后，先让机器人回到初始状态，再开始爬起来
servo_act('backup')			# 从后面爬起来
servo_act('frontup')		# 从前面爬起来
```

