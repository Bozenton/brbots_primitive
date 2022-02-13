import serial
import serial.tools.list_ports
import struct
import threading
import time


# 遥控器类
class ServoController:
    def __init__(self, bound):
        self.bound = bound
        self.startTime = time.time()
        while True:
            port_list = list(serial.tools.list_ports.comports())
            if len(port_list) == 1:
                self.portx = port_list[0].device
                break
            elif len(port_list) > 1:
                temp_i = 1
                print(port_list)
                for item in port_list:
                    print(temp_i, '  -  ', item.device)
                self.portx = port_list[int(input('输入序号选择串口：')) - 1].device
                break
            else:
                input('未发现串口，重新检测！')
        self.timeout = 2
        self.serial = serial.Serial(self.portx, self.bound, timeout=self.timeout)
        print(self.portx, '打开！')

        self.RecvThread_thread = threading.Thread(target=self.RecvThread, args=())
        self.RecvThread_thread.daemon = True
        self.RecvThread_thread.start()  # 打开收数据的线程

    def RecvThread(self):  # 接收数据线程
        while True:
            time.sleep(0.01)
            if self.serial.in_waiting:
                RecvData = self.serial.read(self.serial.in_waiting)
                # if len(RecvData) > 0:
                #     print(RecvData)

    def Cmd_SERVO_MOVE(self, moveTime, indexAndTargetList):
        numOfServo = len(indexAndTargetList)
        dataLength = 5 + numOfServo * 3
        sendData = b'\x55\x55'
        sendData += struct.pack('=BBBH', dataLength, 3, numOfServo, moveTime)
        for i in range(numOfServo):
            index, target = indexAndTargetList[i]
            sendData += struct.pack('=BH', index, target)
        result = self.serial.write(sendData)
        # print(len(sendData))
        # for c in sendData:
        #     print('%#x ' % c, end='')
        # print(list(sendData))


if __name__ == "__main__":
    c = Controller(115200)
    t = 1000
    while True:
        numOfServo = 1
        moveTime = 500
        indexAndTargetList = [[6, t]]
        if t == 1000:
            t = 2000
        else:
            t = 1000
        c.Cmd_SERVO_MOVE(numOfServo, moveTime, indexAndTargetList)
        time.sleep(1)




