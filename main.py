from ParameterRead import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QTextEdit
from PyQt5.QtWidgets import *
from PyQt5.Qt import QFile, QTextStream
from PyQt5.Qt import *
from PyQt5.QtCore import QDataStream, QFile, QIODevice
import sys
from ctypes import *
import platform
import sys
from time import sleep
from usb_device import *
from usb2lin_ex import *
import threading
import binascii

RxLINMsg = LIN_EX_MSG()
CheckRxLINMsg = LIN_EX_MSG()
TxLINMsg = LIN_EX_MSG()
CheckTxLINMsg = LIN_EX_MSG()
LINMasterIndex = 0
DevHandles = (c_uint * 20)()
LIN_EX_PASS = 1
RxData = []
Check_RxData = {}
DID_NVM = {}
Parameter_update = []

def ConsoleDisplay(str):
    ui.textEdit.insertPlainText(str)
    ui.textEdit.insertPlainText('\n')


class DeviceOperate():
    def __init__(self) -> None:
        pass

    def ScanDevice(self):
        ret = USB_ScanDevice(byref(DevHandles))
        if (ret == 0):
            print("No device connected!")
            ui.console.insertPlainText("No device connected!")
            sys.exit(app.exec_())
        else:
            print("Have %d device connected!" % ret)

    def OpenDevice(self):
        self.ScanDevice()
        ret = USB_OpenDevice(DevHandles[0])
        if (bool(ret)):
            ConsoleDisplay("Open device success!")
        else:
            ConsoleDisplay("Open device faild!")
            sys.exit(app.exec_())
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
            ConsoleDisplay("--Function String: %s" % bytes(USB2XXXFunctionString.value).decode('ascii'))
        else:
            ConsoleDisplay("Get device infomation faild!")
            sys.exit(app.exec_())
            # 初始化配置主LIN
        ret = LIN_EX_Init(DevHandles[0], LINMasterIndex, 19200, LIN_EX_MASTER)
        if ret != LIN_EX_SUCCESS:
            ConsoleDisplay("Config Master LIN failed!")
            sys.exit(app.exec_())
        else:
            ConsoleDisplay("Config Master LIN Success!")

    def CloseDevice(self):
        ret = USB_CloseDevice(DevHandles[0])
        if (bool(ret)):
            ConsoleDisplay("close device success!")
        else:
            ConsoleDisplay("close device faild!")

    def ClosePanel(self):
        sys.exit(app.exec_())


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
    TxLINMsg.Timestamp = 0  # 发送完毕数据之后延时10ms
    TxLINMsg.PID = 0x3C  # 高2位的校验数据可以设置为0，底层会自动计算
    TxLINMsg.CheckType = LIN_EX_CHECK_STD  # 使用增stand校验
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
    sleep(0.05)
    return ret


def WakeUp():
    LINOutMsg = LIN_EX_MSG()
    LINMsg = LIN_EX_MSG()
    LINMsg.MsgType = LIN_EX_MSG_TYPE_BK  # 消息类型为同步间隔信号
    LINMsg.Timestamp = 10  # 发送同步间隔信号之后延时10ms
    ret = LIN_EX_MasterSync(DevHandles[0], LINMasterIndex, byref(LINMsg), byref(LINOutMsg), 1)
    if ret != 1:
        print("Send LIN break failed!")
        exit()
    else:
        print("Send LIN break success!")
    sleep(0.01)


def ReadAllParameter():
    global RxData  # 读取到的数据
    global Check_RxData  # 校验的标准参数数据
    developMode = []  # 通过诊断方式读取的数据
    Buffer = [0x7F, 0x06, 0xB2, 0x33, 0x13, 0x00, 0xFE, 0xCA]
    WakeUp()
    # 报文发送成功
    if WriteFrame(Buffer) == LIN_EX_PASS:
        # 有报文回复
        if ReadFrame() == LIN_EX_PASS and CheckRxLINMsg.Data[2] == 0xF2:
            # 从地址0x0840开始校验，一共128位
            Buffer = [0x7F, 0x06, 0xB4, 0x08, 0x08, 0x40, 0x00, 0x7F]
            if WriteFrame(Buffer) == LIN_EX_PASS:
                for n in range(0, 22):
                    ReadFrame()
                    RxData.extend(CheckRxLINMsg.Data[2:8])
                # 读取到的参数打印到终端
                DisplayProductInfos(RxData)
                # 当前读取到的参数数据
                current_RxData = {'no': RxData[0], 'no1': RxData[1], 'no2': RxData[2],
                                  'RomValid Flag msb': RxData[3], 'RomValid Flag lsb': RxData[4], 'reserve': RxData[5],
                                  'reserve1': RxData[6], 'reserve2': RxData[7],
                                  'BrightnessRatio': RxData[8], 'Coordinate Shift X_bak': RxData[9],
                                  'Coordinate Shift Y_bak': RxData[10],
                                  'reserve3': 255,
                                  'DimmingWhen0': RxData[12],
                                  'DimmingFactor': RxData[13], 'gotoSleepDimmingTime': RxData[14],
                                  'waiteForGotoSleep': RxData[15],
                                  'p_log_dimmramape1': RxData[16], 'p_log_dimmramape2': RxData[17],
                                  'p_log_dimmramape3': RxData[18],
                                  'p_log_dimmramape4': RxData[19], 'p_log_dimmramape5': RxData[20],
                                  'p_log_dimmramape6': RxData[21],
                                  'p_log_dimmramape7': RxData[22], 'p_log_dimmramape8': RxData[23],
                                  'p_log_dimmramape9': RxData[24],
                                  'reserve4': 255,
                                  'SWpara_CHKSUM': 180,
                                  'NAD_EOL': RxData[27], 'NAD_write': RxData[28], 'NAD_AA': RxData[29],
                                  'NAD_Sel': RxData[30],
                                  'addressing_CHKSUM': 118, 'reserve5': 255, 'reserve6': 255, 'reserve7': 255,
                                  'short_R': RxData[35], 'short_G': RxData[36], 'short_B': RxData[37],
                                  'open_R': RxData[38],
                                  'open_G': RxData[39], 'open_B': RxData[40], 'error_RAM': RxData[41],
                                  'error_ROM': RxData[42],
                                  'error_NVM': RxData[43], 's_int_software_reset_cnt': RxData[44],
                                  's_int_hardware_reset_cnt': RxData[45], 'DTC_CHKSUM': 142,
                                  'reserve8': 255, 'reserve9': 255, 'reserve10': 255, 'reserve11': 255,
                                  'SerialNo1': 20, 'SerialNo2': 23, 'SerialNo3': 1,
                                  'SerialNo4': 1, 'SerialNo5': 0, 'SerialNo6': 11, 'SerialNo7': 22, 'SerialNo8': 30,
                                  'PartNo1': RxData[59], 'PartNo2': RxData[60],
                                  'PartNo3': RxData[61], 'PartNo4': RxData[62], 'PartNo5': RxData[63],
                                  'PartNo6': RxData[64],
                                  'PartNo7': RxData[65], 'PartNo8': RxData[66], 'PartNo9': RxData[67],
                                  'PartNo10': RxData[68], 'HWver1': RxData[69], 'HWver2': RxData[70],
                                  'HWver3': RxData[71],
                                  'reserve12': 255, 'reserve13': 255,
                                  'config_CHKSUM': 165,
                                  'StClibColor(Rx)': 105 + 27, 'StClibColor(Ry)': 161 + 11, 'StClibColor(RY)': 216 + 54,
                                  'StClibColor(Gx)': 32 + 7, 'StClibColor(Gy)': 86 + 28, 'StClibColor(GY)': 174 + 86,
                                  'StClibColor(Bx)': 7 + 6, 'StClibColor(By)': 254 + 0, 'StClibColor(BY)': 209 + 10,
                                  'calibration_flag': RxData[93], 'brightnessadjust': RxData[94],
                                  'calibration_CHKSUM': 236,
                                  'reserve14': 255, 'reserve15': 255, 'reserve16': 255, 'NadLock': 0, 'PidLock': 0,
                                  'SavePid1': 0,
                                  'SavePid2': 0, 'SavePid3': 0, 'SavePid4': 0, 'SavePid5': 0, 'SavePid6': 0,
                                  'SavePid7': 0,
                                  'SavePid8': 0, 'SavePid9': 0, 'SavePid10': 0, 'B0NAD': 0, 'reserve17': 0,
                                  'reserve18': 0,
                                  'reserve19': 0, 'reserve20': 0, 'reserve21': 0, 'reserve22': 0, 'reserve23': 0,
                                  'reserve24': 0,
                                  'reserve25': 0, 'reserve26': 0, 'reserve27': 0, 'reserve28': 0,
                                  'reserve29': 0, 'reserve30': 0, 'reserve31': 0, 'reserve32': 0, 'reserve33': 0,
                                  'reserve34': 0,
                                  'reserve35': 255, 'reserve36': 255}
                # 获取当前选择的版本
                current_text = ui.comboBox.currentText()
                print("当前版本号为:", current_text)
                # 版本号为 355
                if current_text == "355":
                    # 错误标志位
                    Error_sign = 0
                    Check_RxData = {'no': 129, 'no1': 244, 'no2': 5,
                                    'RomValid Flag msb': 85, 'RomValid Flag lsb': 170, 'reserve': 255, 'reserve1': 255,
                                    'reserve2': 255,
                                    'BrightnessRatio': 100, 'Coordinate Shift X_bak': 255,
                                    'Coordinate Shift Y_bak': 255,
                                    'reserve3': 255,
                                    'DimmingWhen0': 10, 'DimmingFactor': 1, 'gotoSleepDimmingTime': 50,
                                    'waiteForGotoSleep': 4,
                                    'p_log_dimmramape1': 1, 'p_log_dimmramape2': 4, 'p_log_dimmramape3': 13,
                                    'p_log_dimmramape4': 26,
                                    'p_log_dimmramape5': 46, 'p_log_dimmramape6': 71, 'p_log_dimmramape7': 105,
                                    'p_log_dimmramape8': 145, 'p_log_dimmramape9': 196,
                                    'reserve4': 255,
                                    'SWpara_CHKSUM': 180, 'NAD_EOL': 1, 'NAD_write': 255, 'NAD_AA': 16, 'NAD_Sel': 1,
                                    'addressing_CHKSUM': 118,
                                    'reserve5': 255, 'reserve6': 255, 'reserve7': 255,
                                    'short_R': 0, 'short_G': 0, 'short_B': 0, 'open_R': 0, 'open_G': 0, 'open_B': 0,
                                    'error_RAM': 0, 'error_ROM': 0, 'error_NVM': 0,
                                    's_int_software_reset_cnt': 0, 's_int_hardware_reset_cnt': 0,
                                    'DTC_CHKSUM': 142,
                                    'reserve8': 255, 'reserve9': 255, 'reserve10': 255, 'reserve11': 255,
                                    'SerialNo1': 20, 'SerialNo2': 23, 'SerialNo3': 1, 'SerialNo4': 1, 'SerialNo5': 0,
                                    'SerialNo6': 11,
                                    'SerialNo7': 22, 'SerialNo8': 30, 'PartNo1': 56, 'PartNo2': 53, 'PartNo3': 68,
                                    'PartNo4': 57,
                                    'PartNo5': 52, 'PartNo6': 55, 'PartNo7': 51, 'PartNo8': 53, 'PartNo9': 53,
                                    'PartNo10': 36,
                                    'HWver1': 72, 'HWver2': 48, 'HWver3': 50,
                                    'reserve12': 255, 'reserve13': 255, 'config_CHKSUM': 165,
                                    'StClibColor(Rx)': 105 + 27, 'StClibColor(Ry)': 161 + 11,
                                    'StClibColor(RY)': 216 + 54,
                                    'StClibColor(Gx)': 32 + 7, 'StClibColor(Gy)': 86 + 28, 'StClibColor(GY)': 174 + 86,
                                    'StClibColor(Bx)': 7 + 6, 'StClibColor(By)': 254 + 0, 'StClibColor(BY)': 209 + 10,
                                    'calibration_flag': 1, 'brightnessadjust': 100,
                                    'calibration_CHKSUM': 236,
                                    'reserve14': 255, 'reserve15': 255, 'reserve16': 255, 'NadLock': 0,
                                    'PidLock': 0, 'SavePid1': 0, 'SavePid2': 0, 'SavePid3': 0, 'SavePid4': 0,
                                    'SavePid5': 0,
                                    'SavePid6': 0, 'SavePid7': 0, 'SavePid8': 0, 'SavePid9': 0, 'SavePid10': 0,
                                    'B0NAD': 0,
                                    'reserve17': 0, 'reserve18': 0,
                                    'reserve19': 0, 'reserve20': 0, 'reserve21': 0, 'reserve22': 0, 'reserve23': 0,
                                    'reserve24': 0,
                                    'reserve25': 0, 'reserve26': 0, 'reserve27': 0, 'reserve28': 0, 'reserve29': 0,
                                    'reserve30': 0,
                                    'reserve31': 0, 'reserve32': 0, 'reserve33': 0, 'reserve34': 0, 'reserve35': 255,
                                    'reserve36': 255
                                    }
                    print('版本355参数有差异的名称：')
                    diff = Check_RxData.keys() & current_RxData
                    for k in diff:
                        if Check_RxData[k] != current_RxData[k]:
                            diff_vals = (k, Check_RxData[k], current_RxData[k])
                            Parameter_update.append(diff_vals[1])
                            Error_sign += 1
                            print(diff_vals)
                    SerialNo = RxData[51:59]
                    print("SerialNo", SerialNo)
                    calculate_Rx = (RxData[75] + RxData[76] * 256)
                    calculate_Ry = (RxData[77] + RxData[78] * 256)
                    calculate_RY = (RxData[79] + RxData[80] * 256)
                    calculate_Gx = (RxData[81] + RxData[82] * 256)
                    calculate_Gy = (RxData[83] + RxData[84] * 256)
                    calculate_GY = (RxData[85] + RxData[86] * 256)
                    calculate_Bx = (RxData[87] + RxData[88] * 256)
                    calculate_By = (RxData[89] + RxData[90] * 256)
                    calculate_BY = (RxData[91] + RxData[92] * 256)
                    if ((7500 <= calculate_Rx) or (calculate_Rx <= 6500) or
                            (3500 <= calculate_Ry) or (calculate_Ry <= 2500) or
                            (18000 <= calculate_RY) or (calculate_RY <= 8000)):
                        print("StClib Color(R):", calculate_Rx, calculate_Ry, calculate_RY)
                        Error_sign += 1
                    if ((2200 <= calculate_Gx) or (calculate_Gx <= 1200) or
                            (7800 <= calculate_Gy) or (calculate_Gy <= 6800) or
                            (30000 <= calculate_GY) or (calculate_GY <= 20000)):
                        print("StClibColor(G):", calculate_Gx, calculate_Gy, calculate_GY)
                        Error_sign += 1
                    if ((2000 <= calculate_Bx) or (calculate_Bx <= 1000) or
                            (350 <= calculate_By) or (calculate_By <= 150) or
                            (5000 <= calculate_BY) or (calculate_BY <= 2000)):
                        print("StClibColor(B):", calculate_Bx, calculate_By, calculate_BY)
                        Error_sign += 1
                    Buffer = [0x7F, 0x03, 0x22, 0x22, 0xA6, 0x13, 0xFF, 0xFF]
                    if WriteFrame(Buffer) == LIN_EX_PASS:
                        ReadFrame()
                        developMode.extend(CheckRxLINMsg.Data[2:8])
                        if developMode[0] != 34 or developMode[1] != 34 or developMode[2] != 166 or developMode[3] != 19:
                            print("developMode:有反馈")
                            Error_sign += 1
                    if Error_sign == 0:
                        print('参数校验正确')
                    else:
                        print('有 %d 个参数错误' % Error_sign)

                # 版本号为 355A
                if current_text == "355A":
                    # 错误标志位
                    Error_sign = 0
                    Check_RxData = {'no': 129, 'no1': 244, 'no2': 5,
                                    'RomValid Flag msb': 85, 'RomValid Flag lsb': 170, 'reserve': 255, 'reserve1': 255,
                                    'reserve2': 255,
                                    'BrightnessRatio': 100, 'Coordinate Shift X_bak': 73,
                                    'Coordinate Shift Y_bak': 183,
                                    'reserve3': 255,
                                    'DimmingWhen0': 10, 'DimmingFactor': 1, 'gotoSleepDimmingTime': 50,
                                    'waiteForGotoSleep': 4,
                                    'p_log_dimmramape1': 1, 'p_log_dimmramape2': 4, 'p_log_dimmramape3': 13,
                                    'p_log_dimmramape4': 26,
                                    'p_log_dimmramape5': 46, 'p_log_dimmramape6': 71, 'p_log_dimmramape7': 105,
                                    'p_log_dimmramape8': 145, 'p_log_dimmramape9': 196,
                                    'reserve4': 255,
                                    'SWpara_CHKSUM': 180, 'NAD_EOL': 1, 'NAD_write': 255, 'NAD_AA': 16, 'NAD_Sel': 1,
                                    'addressing_CHKSUM': 118,
                                    'reserve5': 255, 'reserve6': 255, 'reserve7': 255,
                                    'short_R': 0, 'short_G': 0, 'short_B': 0, 'open_R': 0, 'open_G': 0, 'open_B': 0,
                                    'error_RAM': 0, 'error_ROM': 0, 'error_NVM': 0,
                                    's_int_software_reset_cnt': 0, 's_int_hardware_reset_cnt': 0,
                                    'DTC_CHKSUM': 142,
                                    'reserve8': 255, 'reserve9': 255, 'reserve10': 255, 'reserve11': 255,
                                    'SerialNo1': 20, 'SerialNo2': 23, 'SerialNo3': 1, 'SerialNo4': 1, 'SerialNo5': 0,
                                    'SerialNo6': 11,
                                    'SerialNo7': 22, 'SerialNo8': 30, 'PartNo1': 56, 'PartNo2': 53, 'PartNo3': 68,
                                    'PartNo4': 57,
                                    'PartNo5': 52, 'PartNo6': 55, 'PartNo7': 51, 'PartNo8': 53, 'PartNo9': 53,
                                    'PartNo10': 36,
                                    'HWver1': 72, 'HWver2': 48, 'HWver3': 50,
                                    'reserve12': 255, 'reserve13': 255, 'config_CHKSUM': 165,
                                    'StClibColor(Rx)': 105 + 27, 'StClibColor(Ry)': 161 + 11,
                                    'StClibColor(RY)': 216 + 54,
                                    'StClibColor(Gx)': 32 + 7, 'StClibColor(Gy)': 86 + 28, 'StClibColor(GY)': 174 + 86,
                                    'StClibColor(Bx)': 7 + 6, 'StClibColor(By)': 254 + 0, 'StClibColor(BY)': 209 + 10,
                                    'calibration_flag': 1, 'brightnessadjust': 100,
                                    'calibration_CHKSUM': 236,
                                    'reserve14': 255, 'reserve15': 255, 'reserve16': 255, 'NadLock': 0,
                                    'PidLock': 0, 'SavePid1': 0, 'SavePid2': 0, 'SavePid3': 0, 'SavePid4': 0,
                                    'SavePid5': 0,
                                    'SavePid6': 0, 'SavePid7': 0, 'SavePid8': 0, 'SavePid9': 0, 'SavePid10': 0,
                                    'B0NAD': 0,
                                    'reserve17': 0, 'reserve18': 0,
                                    'reserve19': 0, 'reserve20': 0, 'reserve21': 0, 'reserve22': 0, 'reserve23': 0,
                                    'reserve24': 0,
                                    'reserve25': 0, 'reserve26': 0, 'reserve27': 0, 'reserve28': 0, 'reserve29': 0,
                                    'reserve30': 0,
                                    'reserve31': 0, 'reserve32': 0, 'reserve33': 0, 'reserve34': 0, 'reserve35': 255,
                                    'reserve36': 255
                                    }
                    print('版本355A参数有差异的名称：')
                    diff = Check_RxData.keys() & current_RxData
                    for k in diff:
                        if Check_RxData[k] != current_RxData[k]:
                            diff_vals = (k, Check_RxData[k], current_RxData[k])
                            Error_sign += 1
                            print(diff_vals)
                    SerialNo = RxData[51:59]
                    print("SerialNo", SerialNo)
                    calculate_Rx = (RxData[75] + RxData[76] * 256)
                    calculate_Ry = (RxData[77] + RxData[78] * 256)
                    calculate_RY = (RxData[79] + RxData[80] * 256)
                    calculate_Gx = (RxData[81] + RxData[82] * 256)
                    calculate_Gy = (RxData[83] + RxData[84] * 256)
                    calculate_GY = (RxData[85] + RxData[86] * 256)
                    calculate_Bx = (RxData[87] + RxData[88] * 256)
                    calculate_By = (RxData[89] + RxData[90] * 256)
                    calculate_BY = (RxData[91] + RxData[92] * 256)
                    if ((7500 <= calculate_Rx) or (calculate_Rx <= 6500) or
                            (3500 <= calculate_Ry) or (calculate_Ry <= 2500) or
                            (18000 <= calculate_RY) or (calculate_RY <= 8000)):
                        print("StClib Color(R):", calculate_Rx, calculate_Ry, calculate_RY)
                        Error_sign += 1
                    if ((2200 <= calculate_Gx) or (calculate_Gx <= 1200) or
                            (7800 <= calculate_Gy) or (calculate_Gy <= 6800) or
                            (30000 <= calculate_GY) or (calculate_GY <= 20000)):
                        print("StClibColor(G):", calculate_Gx, calculate_Gy, calculate_GY)
                        Error_sign += 1
                    if ((2000 <= calculate_Bx) or (calculate_Bx <= 1000) or
                            (350 <= calculate_By) or (calculate_By <= 150) or
                            (5000 <= calculate_BY) or (calculate_BY <= 2000)):
                        print("StClibColor(B):", calculate_Bx, calculate_By, calculate_BY)
                        Error_sign += 1
                    Buffer = [0x7F, 0x03, 0x22, 0x22, 0xA6, 0x13, 0xFF, 0xFF]
                    if WriteFrame(Buffer) == LIN_EX_PASS:
                        ReadFrame()
                        developMode.extend(CheckRxLINMsg.Data[2:8])
                        if developMode[0] != 34 or developMode[1] != 34 or developMode[2] != 166 or developMode[3] != 19:
                            print("developMode:有反馈")
                            Error_sign += 1
                    if Error_sign == 0:
                        print('参数校验正确')
                    else:
                        print('有 %d 个参数错误' % Error_sign)

                # 版本号为 355B
                if current_text == "355B":
                    # 错误标志位
                    Error_sign = 0
                    Check_RxData = {'no': 129, 'no1': 244, 'no2': 5,
                                    'RomValid Flag msb': 85, 'RomValid Flag lsb': 170, 'reserve': 255, 'reserve1': 255,
                                    'reserve2': 255,
                                    'BrightnessRatio': 100, 'Coordinate Shift X_bak': 255,
                                    'Coordinate Shift Y_bak': 255,
                                    'reserve3': 255,
                                    'DimmingWhen0': 10, 'DimmingFactor': 1, 'gotoSleepDimmingTime': 50,
                                    'waiteForGotoSleep': 4,
                                    'p_log_dimmramape1': 1, 'p_log_dimmramape2': 4, 'p_log_dimmramape3': 13,
                                    'p_log_dimmramape4': 26,
                                    'p_log_dimmramape5': 46, 'p_log_dimmramape6': 71, 'p_log_dimmramape7': 105,
                                    'p_log_dimmramape8': 145, 'p_log_dimmramape9': 196,
                                    'reserve4': 255,
                                    'SWpara_CHKSUM': 180, 'NAD_EOL': 1, 'NAD_write': 255, 'NAD_AA': 16, 'NAD_Sel': 1,
                                    'addressing_CHKSUM': 118,
                                    'reserve5': 255, 'reserve6': 255, 'reserve7': 255,
                                    'short_R': 0, 'short_G': 0, 'short_B': 0, 'open_R': 0, 'open_G': 0, 'open_B': 0,
                                    'error_RAM': 0, 'error_ROM': 0, 'error_NVM': 0,
                                    's_int_software_reset_cnt': 0, 's_int_hardware_reset_cnt': 0,
                                    'DTC_CHKSUM': 142,
                                    'reserve8': 255, 'reserve9': 255, 'reserve10': 255, 'reserve11': 255,
                                    'SerialNo1': 20, 'SerialNo2': 23, 'SerialNo3': 1, 'SerialNo4': 1, 'SerialNo5': 0,
                                    'SerialNo6': 11,
                                    'SerialNo7': 22, 'SerialNo8': 30, 'PartNo1': 56, 'PartNo2': 53, 'PartNo3': 68,
                                    'PartNo4': 57,
                                    'PartNo5': 52, 'PartNo6': 55, 'PartNo7': 51, 'PartNo8': 53, 'PartNo9': 53,
                                    'PartNo10': 36,
                                    'HWver1': 72, 'HWver2': 48, 'HWver3': 50,
                                    'reserve12': 255, 'reserve13': 255, 'config_CHKSUM': 165,
                                    'StClibColor(Rx)': 105 + 27, 'StClibColor(Ry)': 161 + 11,
                                    'StClibColor(RY)': 216 + 54,
                                    'StClibColor(Gx)': 32 + 7, 'StClibColor(Gy)': 86 + 28, 'StClibColor(GY)': 174 + 86,
                                    'StClibColor(Bx)': 7 + 6, 'StClibColor(By)': 254 + 0, 'StClibColor(BY)': 209 + 10,
                                    'calibration_flag': 1, 'brightnessadjust': 100,
                                    'calibration_CHKSUM': 236,
                                    'reserve14': 255, 'reserve15': 255, 'reserve16': 255, 'NadLock': 0,
                                    'PidLock': 0, 'SavePid1': 0, 'SavePid2': 0, 'SavePid3': 0, 'SavePid4': 0,
                                    'SavePid5': 0,
                                    'SavePid6': 0, 'SavePid7': 0, 'SavePid8': 0, 'SavePid9': 0, 'SavePid10': 0,
                                    'B0NAD': 0,
                                    'reserve17': 0, 'reserve18': 0,
                                    'reserve19': 0, 'reserve20': 0, 'reserve21': 0, 'reserve22': 0, 'reserve23': 0,
                                    'reserve24': 0,
                                    'reserve25': 0, 'reserve26': 0, 'reserve27': 0, 'reserve28': 0, 'reserve29': 0,
                                    'reserve30': 0,
                                    'reserve31': 0, 'reserve32': 0, 'reserve33': 0, 'reserve34': 0, 'reserve35': 255,
                                    'reserve36': 255
                                    }
                    print('版本355B参数有差异的名称：')
                    diff = Check_RxData.keys() & current_RxData
                    for k in diff:
                        if Check_RxData[k] != current_RxData[k]:
                            diff_vals = (k, Check_RxData[k], current_RxData[k])
                            Error_sign += 1
                            print(diff_vals)
                    SerialNo = RxData[51:59]
                    print("SerialNo", SerialNo)
                    calculate_Rx = (RxData[75] + RxData[76] * 256)
                    calculate_Ry = (RxData[77] + RxData[78] * 256)
                    calculate_RY = (RxData[79] + RxData[80] * 256)
                    calculate_Gx = (RxData[81] + RxData[82] * 256)
                    calculate_Gy = (RxData[83] + RxData[84] * 256)
                    calculate_GY = (RxData[85] + RxData[86] * 256)
                    calculate_Bx = (RxData[87] + RxData[88] * 256)
                    calculate_By = (RxData[89] + RxData[90] * 256)
                    calculate_BY = (RxData[91] + RxData[92] * 256)
                    if ((7500 <= calculate_Rx) or (calculate_Rx <= 6500) or
                            (3500 <= calculate_Ry) or (calculate_Ry <= 2500) or
                            (18000 <= calculate_RY) or (calculate_RY <= 8000)):
                        print("StClib Color(R):", calculate_Rx, calculate_Ry, calculate_RY)
                        Error_sign += 1
                    if ((2200 <= calculate_Gx) or (calculate_Gx <= 1200) or
                            (7800 <= calculate_Gy) or (calculate_Gy <= 6800) or
                            (30000 <= calculate_GY) or (calculate_GY <= 20000)):
                        print("StClibColor(G):", calculate_Gx, calculate_Gy, calculate_GY)
                        Error_sign += 1
                    if ((2000 <= calculate_Bx) or (calculate_Bx <= 1000) or
                            (350 <= calculate_By) or (calculate_By <= 150) or
                            (5000 <= calculate_BY) or (calculate_BY <= 2000)):
                        print("StClibColor(B):", calculate_Bx, calculate_By, calculate_BY)
                        Error_sign += 1
                    Buffer = [0x7F, 0x03, 0x22, 0x22, 0xA6, 0x1, 0xFF, 0xFF]
                    if WriteFrame(Buffer) == LIN_EX_PASS:
                        ReadFrame()
                        developMode.extend(CheckRxLINMsg.Data[2:8])
                        if developMode[0] != 34 or developMode[1] != 34 or developMode[2] != 166 or developMode[3] != 19:
                            print("developMode:有反馈")
                            Error_sign += 1
                    if Error_sign == 0:
                        print('参数校验正确')
                    else:
                        print('有 %d 个参数错误' % Error_sign)


def DisplayProductInfos(buffer):
    RomValidFlag = buffer[3] << 8 + buffer[4]
    print("RomValidFlag", RomValidFlag)
    BrigthnessAdjustment = buffer[8] & 0x7F
    print("BrigthnessAdjustment", BrigthnessAdjustment)
    DimmingType = buffer[8] >> 8
    print("DimmingType", DimmingType)
    CoordinateShiftX = buffer[9]
    print("CoordinateShiftX", CoordinateShiftX)
    CoordinateShiftY = buffer[10]
    print("CoordinateShiftY", CoordinateShiftY)
    DimmingWhen0 = buffer[12]
    print("DimmingWhen0", DimmingWhen0)
    DimmingFactor = buffer[13]
    print("DimmingFactor", DimmingFactor)
    gotoSleepDimmingTime = buffer[14]
    print("gotoSleepDimmingTime", gotoSleepDimmingTime)
    waiteForGotoSleep = buffer[15]
    print("waiteForGotoSleep", waiteForGotoSleep)
    p_log_dimmramape = buffer[16:25]
    print("p_log_dimmramape", p_log_dimmramape)
    NAD_EOL = buffer[27]
    print("NAD_EOL", NAD_EOL)
    NAD_write = buffer[28]
    print("NAD_write", NAD_write)
    NAD_AA = buffer[29]
    print("NAD_AA", NAD_AA)
    NAD_Sel = buffer[30]
    print("NAD_Sel", NAD_Sel)
    short_R = buffer[35]
    print("short_R", short_R)
    short_G = buffer[36]
    print("short_G", short_G)
    short_B = buffer[37]
    print("short_B", short_B)
    open_R = buffer[38]
    print("open_R", open_R)
    open_G = buffer[39]
    print("open_G", open_G)
    open_B = buffer[40]
    print("open_B", open_B)
    error_RAM = buffer[41]
    print("error_RAM", error_RAM)
    error_ROM = buffer[42]
    print("error_ROM", error_ROM)
    error_NVM = buffer[43]
    print("error_NVM", error_NVM)
    s_int_software_reset_cnt = buffer[44]
    print("s_int_software_reset_cnt", s_int_software_reset_cnt)
    s_int_hardware_reset_cnt = buffer[45]
    print("s_int_hardware_reset_cnt", s_int_hardware_reset_cnt)
    SerialNo = buffer[51:59]
    print("SerialNo", SerialNo)
    PartNo = buffer[59:69]
    print("PartNo", PartNo)
    HWver = buffer[69:72]
    print("HWver", HWver)
    StClibColor_R_x = (buffer[75] + buffer[76] * 256) / 10000
    print("StClibColor_R_x", StClibColor_R_x)
    StClibColor_R_y = (buffer[77] + buffer[78] * 256) / 10000
    print("StClibColor_R_y", StClibColor_R_y)
    StClibColor_R_Y = (buffer[79] + buffer[80] * 256) / 10000
    print("StClibColor_R_Y", StClibColor_R_Y)
    StClibColor_G_x = (buffer[81] + buffer[82] * 256) / 10000
    print("StClibColor_G_x", StClibColor_G_x)
    StClibColor_G_y = (buffer[83] + buffer[84] * 256) / 10000
    print("StClibColor_G_y", StClibColor_G_y)
    StClibColor_G_Y = (buffer[85] + buffer[86] * 256) / 10000
    print("StClibColor_G_Y", StClibColor_G_Y)
    StClibColor_B_x = (buffer[87] + buffer[88] * 256) / 10000
    print("StClibColor_B_x", StClibColor_B_x)
    StClibColor_B_y = (buffer[89] + buffer[90] * 256) / 10000
    print("StClibColor_B_y", StClibColor_B_y)
    StClibColor_B_Y = (buffer[91] + buffer[92] * 256) / 10000
    print("StClibColor_B_Y", StClibColor_B_Y)
    calibration_flag = buffer[93]
    print("calibration_flag", calibration_flag)
    Brightnessadjust = buffer[94]
    print("Brightnessadjust", Brightnessadjust)


def Parameter_update():
    Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x13, 0x01, 0xFF, 0xFF]
    WakeUp()
    DID_NVM = {
               'brigthness adjustment': 0x07,
               'dimming type': 0x08,
               'Coordinate Shift X_bak': 0x25,
               'Coordinate Shift Y_bak': 0x26,
               'DimmingWhen0': 0x04,
               'DimmingFactor': 0x10,
               'gotoSleepDimmingTime': 0x05,
               'waiteForGotoSleep': 0x06,
               'p_log_dimmramape': 0x09,
               'NAD_EOL': 0x00,
               'NAD_write': 0x01,
               'NAD_AA': 0x02,
               'NAD_Sel': 0x03,
               'short_R': 0x14,
               'short_G': 0x15,
               'short_B': 0x16,
               'open_R': 0x17,
               'open_G': 0x18,
               'open_B': 0x19,
               'error_RAM': 0x20,
               'error_ROM': 0x21,
               'error_NVM': 0x22,
               's_int_software_reset_cnt': 0x3D,
               's_int_hardware_reset_cnt': 0x3E,
               'SerialNo': 0x32,
               'PartNo': 0x33,
               'HWver': 0x35,
               'StClibColor(R)': 0x36,
               'StClibColor(G)': 0x37,
               'StClibColor(B)': 0x38,
               'calibration_flag': 0x41,
               'brightnessadjust': 0x42,
               }
    # 报文发送成功
    if WriteFrame(Buffer) == LIN_EX_PASS:
        # 有报文回复
        if ReadFrame() == LIN_EX_PASS:
            Buffer = [0x7F, 0x10, 0xB4, 0x2E, 0xA6, 0x40, 0x10, 0xFF]
            WriteFrame()


if __name__ == "__main__":
    device = DeviceOperate()
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    device.ScanDevice()
    device.OpenDevice()
    ui.pushButton.clicked.connect(ReadAllParameter)
    ui.pushButton_2.clicked.connect(Parameter_update)
    ui.comboBox.addItems(['355', '355A', '355B'])
    sys.exit(app.exec_())
