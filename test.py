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
    global RxData
    developMode = []
    Buffer = [0x7F, 0x06, 0xB2, 0x33, 0x13, 0x00, 0xFE, 0xCA]
    Error_Sign = 0
    WakeUp()
    # 报文发送成功
    if WriteFrame(Buffer) == LIN_EX_PASS:
        # 有报文回复
        if ReadFrame() == LIN_EX_PASS and CheckRxLINMsg.Data[2] == 0xF2:
            # 发送报文，响应读取NVM中128字节数据
            Buffer = [0x7F, 0x06, 0xB4, 0x08, 0x08, 0x40, 0x00, 0x7F]
            # 发送报文成功，打印所有参数
            if WriteFrame(Buffer) == LIN_EX_PASS:
                for n in range(0, 22):
                    ReadFrame()
                    RxData.extend(CheckRxLINMsg.Data[2:8])
                DisplayProductInfos(RxData)
                print(RxData)
                # 获取当前comboBox的值
                current_text = ui.comboBox.currentText()
                print("当前版本号为:", current_text)
                if current_text == "355":
                    # 开始进行参数对比  从0845开始
                    print("参数有误的名称为：")
                    if RxData[8] != 100:
                        print("Brightness Ratio:", hex(RxData[8]))
                        Error_Sign = 1
                    if RxData[9] != 255:
                        print("Coordinate Shift X_bak:", hex(RxData[9]))
                        Error_Sign = 1
                    if RxData[10] != 255:
                        print("Coordinate Shift Y_bak:", hex(RxData[10]))
                        Error_Sign = 1
                    if RxData[12] != 10:
                        print("Dimming When0:", hex(RxData[12]))
                        Error_Sign = 1
                    if RxData[13] != 1:
                        print("Dimming Factor:", hex(RxData[13]))
                        Error_Sign = 1
                    if RxData[14] != 50:
                        print("goto Sleep Dimming Time:", hex(RxData[14]))
                        Error_Sign = 1
                    if RxData[15] != 4:
                        print("waite For Goto Sleep:", hex(RxData[15]))
                        Error_Sign = 1
                    if (RxData[16] != 1 or RxData[17] != 4 or RxData[18] != 13 or RxData[19] != 26 or RxData[20] != 46
                            or RxData[21] != 71 or RxData[22] != 105 or RxData[23] != 145 or RxData[24] != 196):
                        print("p_log_dimmramape:", hex(RxData[16]), hex(RxData[17]), hex(RxData[18]), hex(RxData[19]),
                              hex(RxData[20]), hex(RxData[21]), hex(RxData[22]), hex(RxData[23]), hex(RxData[24]))
                        Error_Sign = 1
                    if RxData[27] != 1:
                        print("NAD_EOL:", hex(RxData[27]))
                        Error_Sign = 1
                    if RxData[28] != 255:
                        print("NAD_write:", hex(RxData[28]))
                        Error_Sign = 1
                    if RxData[29] != 16:
                        print("NAD_AA:", hex(RxData[29]))
                        Error_Sign = 1
                    if RxData[30] != 1:
                        print("NAD_Sel:", hex(RxData[30]))
                        Error_Sign = 1
                    if RxData[35] != 0:
                        print("short_R:", hex(RxData[35]))
                        Error_Sign = 1
                    if RxData[36] != 0:
                        print("short_G:", hex(RxData[36]))
                        Error_Sign = 1
                    if RxData[37] != 0:
                        print("short_B:", hex(RxData[37]))
                        Error_Sign = 1
                    if RxData[38] != 0:
                        print("open_R:", hex(RxData[38]))
                        Error_Sign = 1
                    if RxData[39] != 0:
                        print("open_G:", hex(RxData[39]))
                        Error_Sign = 1
                    if RxData[40] != 0:
                        print("open_B:", hex(RxData[40]))
                        Error_Sign = 1
                    if RxData[41] != 0:
                        print("error_RAM:", hex(RxData[41]))
                        Error_Sign = 1
                    if RxData[42] != 0:
                        print("error_ROM:", hex(RxData[42]))
                        Error_Sign = 1
                    if RxData[43] != 0:
                        print("error_NVM:", hex(RxData[43]))
                        Error_Sign = 1
                    if RxData[44] != 0:
                        print("s_int_software_reset_cnt:", hex(RxData[44]))
                        Error_Sign = 1
                    if RxData[45] != 0:
                        print("s_int_hardware_reset_cnt:", hex(RxData[45]))
                        Error_Sign = 1
                    print("SerialNo(DEC):",RxData[51:59])
                    if (RxData[59] != 56 or RxData[60] != 53 or RxData[61] != 68 or RxData[62] != 57 or RxData[63] != 52
                            or RxData[64] != 55 or RxData[65] != 51 or RxData[66] != 53 or RxData[67] != 53 or RxData[68] != 32):
                        print("Part No:", hex(RxData[59]), hex(RxData[60]), hex(RxData[61]), hex(RxData[62]), hex(RxData[63]),
                              hex(RxData[64]), hex(RxData[65]), hex(RxData[66]), hex(RxData[67]), hex(RxData[68]), )
                        Error_Sign = 1
                    if RxData[69] != 72 or RxData[70] != 48 or RxData[71] != 50:
                        print("HWver:", hex(RxData[69]), hex(RxData[70]), hex(RxData[71]))
                        Error_Sign = 1
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
                    if ((2200 <= calculate_Gx) or (calculate_Gx <= 1200) or
                            (7800 <= calculate_Gy) or (calculate_Gy <= 6800) or
                            (30000 <= calculate_GY) or (calculate_GY <= 20000)):
                        print("StClibColor(G):", calculate_Gx, calculate_Gy, calculate_GY)
                    if ((2000 <= calculate_Bx) or (calculate_Bx <= 1000) or
                            (350 <= calculate_By) or (calculate_By <= 150) or
                            (5000 <= calculate_BY) or (calculate_BY <= 2000)):
                        print("StClibColor(B):",calculate_Bx, calculate_By, calculate_BY)
                    if RxData[93] != 1:
                        print("calibration_flag:", hex(RxData[93]))
                        Error_Sign = 1
                    if RxData[94] != 100:
                        print("brightness adjust:", hex(RxData[94]))
                        Error_Sign = 1
                    Buffer = [0x7F, 0x03, 0x22, 0x22, 0xA6, 0x13, 0xFF, 0xFF]
                    if WriteFrame(Buffer) == LIN_EX_PASS:
                        ReadFrame()
                        developMode.extend(CheckRxLINMsg.Data[2:8])
                        if developMode[0] != 34 or developMode[1] != 34 or developMode[2] != 166 or developMode[3] != 19:
                            print("developMode:有反馈")
                            Error_Sign = 1
                    if Error_Sign == 0:
                        print("参数正确")
                elif current_text == "355A":
                    # 开始进行参数对比  从0845开始
                    print("参数有误的名称为：")
                    if RxData[8] != 100:
                        print("Brightness Ratio:", hex(RxData[8]))
                        Error_Sign = 1
                    if RxData[9] != 73:
                        print("Coordinate Shift X_bak:", hex(RxData[9]))
                        Error_Sign = 1
                    if RxData[10] != 183:
                        print("Coordinate Shift Y_bak:", hex(RxData[10]))
                        Error_Sign = 1
                    if RxData[12] != 10:
                        print("Dimming When0:", hex(RxData[12]))
                        Error_Sign = 1
                    if RxData[13] != 1:
                        print("Dimming Factor:", hex(RxData[13]))
                        Error_Sign = 1
                    if RxData[14] != 50:
                        print("goto Sleep Dimming Time:", hex(RxData[14]))
                        Error_Sign = 1
                    if RxData[15] != 4:
                        print("waite For Goto Sleep:", hex(RxData[15]))
                        Error_Sign = 1
                    if (RxData[16] != 1 or RxData[17] != 4 or RxData[18] != 13 or RxData[19] != 26 or RxData[20] != 46
                            or RxData[21] != 71 or RxData[22] != 105 or RxData[23] != 145 or RxData[24] != 196):
                        print("p_log_dimmramape:", hex(RxData[16]), hex(RxData[17]), hex(RxData[18]), hex(RxData[19]),
                              hex(RxData[20]), hex(RxData[21]), hex(RxData[22]), hex(RxData[23]), hex(RxData[24]))
                        Error_Sign = 1
                    if RxData[27] != 1:
                        print("NAD_EOL:", hex(RxData[27]))
                        Error_Sign = 1
                    if RxData[28] != 255:
                        print("NAD_write:", hex(RxData[28]))
                        Error_Sign = 1
                    if RxData[29] != 16:
                        print("NAD_AA:", hex(RxData[29]))
                        Error_Sign = 1
                    if RxData[30] != 1:
                        print("NAD_Sel:", hex(RxData[30]))
                        Error_Sign = 1
                    if RxData[35] != 0:
                        print("short_R:", hex(RxData[35]))
                        Error_Sign = 1
                    if RxData[36] != 0:
                        print("short_G:", hex(RxData[36]))
                        Error_Sign = 1
                    if RxData[37] != 0:
                        print("short_B:", hex(RxData[37]))
                        Error_Sign = 1
                    if RxData[38] != 0:
                        print("open_R:", hex(RxData[38]))
                        Error_Sign = 1
                    if RxData[39] != 0:
                        print("open_G:", hex(RxData[39]))
                        Error_Sign = 1
                    if RxData[40] != 0:
                        print("open_B:", hex(RxData[40]))
                        Error_Sign = 1
                    if RxData[41] != 0:
                        print("error_RAM:", hex(RxData[41]))
                        Error_Sign = 1
                    if RxData[42] != 0:
                        print("error_ROM:", hex(RxData[42]))
                        Error_Sign = 1
                    if RxData[43] != 0:
                        print("error_NVM:", hex(RxData[43]))
                        Error_Sign = 1
                    if RxData[44] != 0:
                        print("s_int_software_reset_cnt:", hex(RxData[44]))
                        Error_Sign = 1
                    if RxData[45] != 0:
                        print("s_int_hardware_reset_cnt:", hex(RxData[45]))
                        Error_Sign = 1
                    print("SerialNo(DEC):", RxData[51:59])
                    if (RxData[59] != 56 or RxData[60] != 53 or RxData[61] != 68 or RxData[62] != 57 or RxData[63] != 52
                            or RxData[64] != 55 or RxData[65] != 51 or RxData[66] != 53 or RxData[67] != 53 or RxData[
                                68] != 32):
                        print("Part No:", hex(RxData[59]), hex(RxData[60]), hex(RxData[61]), hex(RxData[62]),
                              hex(RxData[63]),
                              hex(RxData[64]), hex(RxData[65]), hex(RxData[66]), hex(RxData[67]), hex(RxData[68]), )
                        Error_Sign = 1
                    if RxData[69] != 72 or RxData[70] != 48 or RxData[71] != 50:
                        print("HWver:", hex(RxData[69]), hex(RxData[70]), hex(RxData[71]))
                        Error_Sign = 1
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
                    if ((2200 <= calculate_Gx) or (calculate_Gx <= 1200) or
                            (7800 <= calculate_Gy) or (calculate_Gy <= 6800) or
                            (30000 <= calculate_GY) or (calculate_GY <= 20000)):
                        print("StClibColor(G):", calculate_Gx, calculate_Gy, calculate_GY)
                    if ((2000 <= calculate_Bx) or (calculate_Bx <= 1000) or
                            (350 <= calculate_By) or (calculate_By <= 150) or
                            (5000 <= calculate_BY) or (calculate_BY <= 2000)):
                        print("StClibColor(B):", calculate_Bx, calculate_By, calculate_BY)
                    if RxData[93] != 1:
                        print("calibration_flag:", hex(RxData[93]))
                        Error_Sign = 1
                    if RxData[94] != 100:
                        print("brightness adjust:", hex(RxData[94]))
                        Error_Sign = 1
                    Buffer = [0x7F, 0x03, 0x22, 0x22, 0xA6, 0x13, 0xFF, 0xFF]
                    if WriteFrame(Buffer) == LIN_EX_PASS:
                        ReadFrame()
                        developMode.extend(CheckRxLINMsg.Data[2:8])
                        if developMode[0] != 34 or developMode[1] != 34 or developMode[2] != 166 or developMode[
                            3] != 19:
                            print("developMode:有反馈")
                            Error_Sign = 1
                    if Error_Sign == 0:
                        print("参数正确")
                elif current_text == "355B":
                    # 开始进行参数对比  从0845开始
                    print("参数有误的名称为：")
                    if RxData[8] != 100:
                        print("Brightness Ratio:", hex(RxData[8]))
                        Error_Sign = 1
                    if RxData[9] != 255:
                        print("Coordinate Shift X_bak:", hex(RxData[9]))
                        Error_Sign = 1
                    if RxData[10] != 255:
                        print("Coordinate Shift Y_bak:", hex(RxData[10]))
                        Error_Sign = 1
                    if RxData[12] != 10:
                        print("Dimming When0:", hex(RxData[12]))
                        Error_Sign = 1
                    if RxData[13] != 1:
                        print("Dimming Factor:", hex(RxData[13]))
                        Error_Sign = 1
                    if RxData[14] != 50:
                        print("goto Sleep Dimming Time:", hex(RxData[14]))
                        Error_Sign = 1
                    if RxData[15] != 4:
                        print("waite For Goto Sleep:", hex(RxData[15]))
                        Error_Sign = 1
                    if (RxData[16] != 1 or RxData[17] != 4 or RxData[18] != 13 or RxData[19] != 26 or RxData[20] != 46
                            or RxData[21] != 71 or RxData[22] != 105 or RxData[23] != 145 or RxData[24] != 196):
                        print("p_log_dimmramape:", hex(RxData[16]), hex(RxData[17]), hex(RxData[18]), hex(RxData[19]),
                              hex(RxData[20]), hex(RxData[21]), hex(RxData[22]), hex(RxData[23]), hex(RxData[24]))
                        Error_Sign = 1
                    if RxData[27] != 1:
                        print("NAD_EOL:", hex(RxData[27]))
                        Error_Sign = 1
                    if RxData[28] != 255:
                        print("NAD_write:", hex(RxData[28]))
                        Error_Sign = 1
                    if RxData[29] != 16:
                        print("NAD_AA:", hex(RxData[29]))
                        Error_Sign = 1
                    if RxData[30] != 1:
                        print("NAD_Sel:", hex(RxData[30]))
                        Error_Sign = 1
                    if RxData[35] != 0:
                        print("short_R:", hex(RxData[35]))
                        Error_Sign = 1
                    if RxData[36] != 0:
                        print("short_G:", hex(RxData[36]))
                        Error_Sign = 1
                    if RxData[37] != 0:
                        print("short_B:", hex(RxData[37]))
                        Error_Sign = 1
                    if RxData[38] != 0:
                        print("open_R:", hex(RxData[38]))
                        Error_Sign = 1
                    if RxData[39] != 0:
                        print("open_G:", hex(RxData[39]))
                        Error_Sign = 1
                    if RxData[40] != 0:
                        print("open_B:", hex(RxData[40]))
                        Error_Sign = 1
                    if RxData[41] != 0:
                        print("error_RAM:", hex(RxData[41]))
                        Error_Sign = 1
                    if RxData[42] != 0:
                        print("error_ROM:", hex(RxData[42]))
                        Error_Sign = 1
                    if RxData[43] != 0:
                        print("error_NVM:", hex(RxData[43]))
                        Error_Sign = 1
                    if RxData[44] != 0:
                        print("s_int_software_reset_cnt:", hex(RxData[44]))
                        Error_Sign = 1
                    if RxData[45] != 0:
                        print("s_int_hardware_reset_cnt:", hex(RxData[45]))
                        Error_Sign = 1
                    print("SerialNo(DEC):", RxData[51:59])
                    if (RxData[59] != 56 or RxData[60] != 53 or RxData[61] != 68 or RxData[62] != 57 or RxData[63] != 52
                            or RxData[64] != 55 or RxData[65] != 51 or RxData[66] != 53 or RxData[67] != 53 or RxData[
                                68] != 32):
                        print("Part No:", hex(RxData[59]), hex(RxData[60]), hex(RxData[61]), hex(RxData[62]),
                              hex(RxData[63]),
                              hex(RxData[64]), hex(RxData[65]), hex(RxData[66]), hex(RxData[67]), hex(RxData[68]), )
                        Error_Sign = 1
                    if RxData[69] != 72 or RxData[70] != 48 or RxData[71] != 50:
                        print("HWver:", hex(RxData[69]), hex(RxData[70]), hex(RxData[71]))
                        Error_Sign = 1
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
                    if ((2200 <= calculate_Gx) or (calculate_Gx <= 1200) or
                            (7800 <= calculate_Gy) or (calculate_Gy <= 6800) or
                            (30000 <= calculate_GY) or (calculate_GY <= 20000)):
                        print("StClibColor(G):", calculate_Gx, calculate_Gy, calculate_GY)
                    if ((2000 <= calculate_Bx) or (calculate_Bx <= 1000) or
                            (350 <= calculate_By) or (calculate_By <= 150) or
                            (5000 <= calculate_BY) or (calculate_BY <= 2000)):
                        print("StClibColor(B):", calculate_Bx, calculate_By, calculate_BY)
                    if RxData[93] != 1:
                        print("calibration_flag:", hex(RxData[93]))
                        Error_Sign = 1
                    if RxData[94] != 100:
                        print("brightness adjust:", hex(RxData[94]))
                        Error_Sign = 1
                    Buffer = [0x7F, 0x03, 0x22, 0x22, 0xA6, 0x13, 0xFF, 0xFF]
                    if WriteFrame(Buffer) == LIN_EX_PASS:
                        ReadFrame()
                        developMode.extend(CheckRxLINMsg.Data[2:8])
                        if developMode[0] != 34 or developMode[1] != 34 or developMode[2] != 166 or developMode[
                            3] != 19:
                            print("developMode:有反馈")
                            Error_Sign = 1
                    if Error_Sign == 0:
                        print("参数正确")

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
    ui.comboBox.addItems(['355', '355A', '355B'])
    sys.exit(app.exec_())
