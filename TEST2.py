from time import sleep
from usb_device import *
from usb2lin_ex import *
import time


RxLINMsg = LIN_EX_MSG()
CheckRxLINMsg = LIN_EX_MSG()
TxLINMsg = LIN_EX_MSG()
CheckTxLINMsg = LIN_EX_MSG()
LINMasterIndex = 0
DevHandles = (c_uint * 20)()
LIN_EX_PASS = 1


class DeviceOperate():
    def __init__(self) -> None:
        pass

    def ScanDevice(self):
        ret = USB_ScanDevice(byref(DevHandles))
        if (ret == 0):
            print("No device connected!")
            exit()
        else:
            print("Have %d device connected!" % ret)

    def OpenDevice(self):
        self.ScanDevice()
        ret = USB_OpenDevice(DevHandles[0])
        if (bool(ret)):
            print("Open device success!")
        else:
            print("Open device faild!")
            exit()
        USB2XXXInfo = DEVICE_INFO()
        USB2XXXFunctionString = (c_char * 256)()
        ret = DEV_GetDeviceInfo(DevHandles[0], byref(USB2XXXInfo), byref(USB2XXXFunctionString))
        if (bool(ret)):
            print("USB2XXX device infomation:")
            print("--Firmware Name: %s" % bytes(USB2XXXInfo.FirmwareName).decode('ascii'))
            print("--Firmware Version: v%d.%d.%d" % (
                (USB2XXXInfo.FirmwareVersion >> 24) & 0xFF, (USB2XXXInfo.FirmwareVersion >> 16) & 0xFF,
                USB2XXXInfo.FirmwareVersion & 0xFFFF))
            print("--Hardware Version: v%d.%d.%d" % (
                (USB2XXXInfo.HardwareVersion >> 24) & 0xFF, (USB2XXXInfo.HardwareVersion >> 16) & 0xFF,
                USB2XXXInfo.HardwareVersion & 0xFFFF))
            print("--Build Date: %s" % bytes(USB2XXXInfo.BuildDate).decode('ascii'))
            print("--Serial Number: ", end='')
            for i in range(0, len(USB2XXXInfo.SerialNumber)):
                print("%08X" % USB2XXXInfo.SerialNumber[i], end='')
            print("")
            print("--Function String: %s" % bytes(USB2XXXFunctionString.value).decode('ascii'))
        else:
            print("Get device infomation faild!")
            exit()
            # 初始化配置主LIN
        ret = LIN_EX_Init(DevHandles[0], LINMasterIndex, 19200, LIN_EX_MASTER)
        if ret != LIN_EX_SUCCESS:
            print("Config Master LIN failed!")
            exit()
        else:
            print("Config Master LIN Success!")

    def CloseDevice(self):
        ret = USB_CloseDevice(DevHandles[0])
        if (bool(ret)):
            print("close device success!")
        else:
            print("close device faild!")

    def ClosePanel(self):
        exit()


def ReadFrame():
    global CheckRxLINMsg
    RxLINMsg.MsgType = LIN_EX_MSG_TYPE_MR  # 消息类型为主机读数据
    RxLINMsg.Timestamp = 50  # 发送完毕数据之后延时10ms
    RxLINMsg.PID = 0x3D  # 高2位的校验数据可以设置为0，底层会自动计算
    RxLINMsg.CheckType = LIN_EX_CHECK_STD
    ret = LIN_EX_MasterSync(DevHandles[0], LINMasterIndex, byref(RxLINMsg), byref(CheckRxLINMsg), 1)
    if ret != 1:
        print("LIN read data failed!")
        exit()
    else:
        print("S2M", "[0x%02X] " % CheckRxLINMsg.PID, end='')
        for i in range(CheckRxLINMsg.DataLen):
            print("0x%02X " % CheckRxLINMsg.Data[i], end='')
        print("")
    return ret


def WriteFrame(data):
    global CheckTxLINMsg
    TxLINMsg.MsgType = LIN_EX_MSG_TYPE_MW  # 消息类型为主机写数据
    TxLINMsg.Timestamp = 10  # 发送完毕数据之后延时10ms
    TxLINMsg.PID = 0x15  # 高2位的校验数据可以设置为0，底层会自动计算
    TxLINMsg.CheckType = LIN_EX_CHECK_EXT  # 使用增stand校验
    TxLINMsg.DataLen = 8
    for i in range(0, TxLINMsg.DataLen):
        TxLINMsg.Data[i] = data[i]
    ret = LIN_EX_MasterSync(DevHandles[0], LINMasterIndex, byref(TxLINMsg), byref(CheckTxLINMsg), 1)
    if ret != 1:
        print("LIN ID[0x%02X] write data failed!" % CheckTxLINMsg.PID)
        exit()
    else:
        # 显示发送的同时接收到的数据，若数据发送成功，那么接收到的数据应该是跟发送出去的数据一样，否则数据出现了冲突
        print("M2S", "[0x%02X] " % CheckTxLINMsg.PID, end='')
        for i in range(CheckTxLINMsg.DataLen):
            print("0x%02X " % CheckTxLINMsg.Data[i], end='')
        print("")
    # sleep(0.05)
    return ret

device = DeviceOperate()
device.ScanDevice()
device.OpenDevice()
Buffer_data = [0x01, 0x00, 0xFF, 0x5E, 0x18, 0x64, 0x00, 0x02]
counter = 0
while Buffer_data[5] >= 0x0:
    # 发送数据
    WriteFrame(Buffer_data)
    # print(Buffer_data)
    # 等待20毫秒
    #time.sleep(0.01)

    # 每20秒，数据中的一个数增加1
    counter += 1
    if counter == 10 :  # 20秒内发送1000次数据（20ms * 1000 = 20000ms = 20s）
        counter = 0
        Buffer_data[5] -= 1  # 增加数据中的一个数
    if Buffer_data[5] == 0x40:
        Buffer_data1 = [0x01, 0x00, 0xFF, 0x5E, 0x18, 0x64, 0x00, 0x02]
        WriteFrame(Buffer_data1)
