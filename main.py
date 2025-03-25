import os
from PyQt5.QtWidgets import QApplication, QMainWindow
from click import clear
from ParameterRead import Ui_MainWindow
import sys
from time import sleep
from usb_device import *
from usb2lin_ex import *
import time
import copy
from compareData import Info_355, Info_355A, Info_355B, Info_355C

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
Error_parameter = []


def ConsoleDisplay(str):
    ui.textEdit.insertPlainText(str)
    ui.textEdit.insertPlainText('\n')
    ui.textEdit.ensureCursorVisible()


class DeviceOperate():
    def __init__(self) -> None:
        pass

    def ScanDevice(self):
        ret = USB_ScanDevice(byref(DevHandles))
        if (ret == 0):
            ConsoleDisplay("No device connected!")
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
    TxLINMsg.Timestamp = 10  # 发送完毕数据之后延时10ms
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


def text_create(name, msg):
    path = 'ParameterRead_record'
    # 检查文件夹是否存在，如果不存在则创建
    if not os.path.exists(path):
        os.makedirs(path)
    # 构建完整的文件路径
    full_path = os.path.join(path, name + '.txt')
    with open(full_path, 'a') as file:
        file.write(msg)
    file.close()


def ReadAllParameter():
    global RxData  # 读取到的数据
    RxDatas = [] # 当前读取到的数据
    global Check_RxData  # 校验的标准参数数据
    global Error_parameter
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
                    RxDatas.extend(CheckRxLINMsg.Data[2:8])
                RxData.extend(RxDatas)
                # 当前读取到的参数数据
                current_RxData = {'RomValid Flag msb': RxDatas[3],
                                  'RomValid Flag lsb': RxDatas[4],
                                  'Brightness Adjustment': RxDatas[8] & 0x7F,
                                  'DimmingType': RxDatas[8] >> 7,
                                  'Coordinate Shift X_bak': RxDatas[9],
                                  'Coordinate Shift Y_bak': RxDatas[10],
                                  'DimmingWhen0': RxDatas[12],
                                  'DimmingFactor': RxDatas[13],
                                  'gotoSleepDimmingTime': RxDatas[14],
                                  'waiteForGotoSleep': RxDatas[15],
                                  'p_log_dimmramape_L': RxDatas[16:21],
                                  'p_log_dimmramape_H': RxDatas[21:25],
                                  'NAD_EOL': RxDatas[27],
                                  'NAD_write': RxDatas[28],
                                  'NAD_AA': RxDatas[29],
                                  'NAD_Sel': RxDatas[30],
                                  'short_R': RxDatas[35],
                                  'short_G': RxDatas[36],
                                  'short_B': RxDatas[37],
                                  'open_R': RxDatas[38],
                                  'open_G': RxDatas[39],
                                  'open_B': RxDatas[40],
                                  'error_RAM': RxDatas[41],
                                  'error_ROM': RxDatas[42],
                                  'error_NVM': RxDatas[43],
                                  's_int_software_reset_cnt': RxDatas[44],
                                  's_int_hardware_reset_cnt': RxDatas[45],
                                  'SerialNo': RxDatas[51:59],
                                  '产品型号': RxDatas[59:69],
                                  'HW ver': RxDatas[69:72],
                                  'StClibColor_R_x': (RxDatas[75] + RxDatas[76] * 256),
                                  'StClibColor_R_y': (RxDatas[77] + RxDatas[78] * 256),
                                  'StClibColor_R_Y': (RxDatas[79] + RxDatas[80] * 256),
                                  'StClibColor_G_x': (RxDatas[81] + RxDatas[82] * 256),
                                  'StClibColor_G_y': (RxDatas[83] + RxDatas[84] * 256),
                                  'StClibColor_G_Y': (RxDatas[85] + RxDatas[86] * 256),
                                  'StClibColor_B_x': (RxDatas[87] + RxDatas[88] * 256),
                                  'StClibColor_B_y': (RxDatas[89] + RxDatas[90] * 256),
                                  'StClibColor_B_Y': (RxDatas[91] + RxDatas[92] * 256),
                                  'calibration_flag': RxDatas[93],
                                  'brightness adjust': RxDatas[94],
                                  }
                # save read data
                # Step 1 create txt name: using data and SerialNo
                txtname = ''
                now = time.localtime()
                txtname = f"{now.tm_year}-{now.tm_mon:02d}-{now.tm_mday:02d}-{now.tm_hour:02d}-{now.tm_min:02d}-{now.tm_sec:02d}"+ '_SN'
                for n in range(0, 8):
                    txtname = str(txtname) + str(current_RxData['SerialNo'][n]).zfill(2)
                    # Step 2 write Rxdata into txt
                text_create(txtname, str(current_RxData))
                for key, value in current_RxData.items():
                    ConsoleDisplay(str(key) + ' ' + 'is' + ':' + str(value))

                # 获取当前选择的版本
                current_text = ui.comboBox.currentText()
                print("当前版本号为:", current_text)
                # 错误标志位
                Error_sign = 0
                # 版本号为 355
                if current_text == "355":
                    Check_RxData = copy.deepcopy(Info_355)
                    print('版本355参数有差异的名称：')
                    ConsoleDisplay('版本355参数有差异的名称：')
                # 版本号为 355A
                if current_text == "355A":
                    Check_RxData = copy.deepcopy(Info_355A)
                    print('版本355A参数有差异的名称：')
                    ConsoleDisplay('版本355A参数有差异的名称：')
                # 版本号为 355B
                if current_text == "355B":
                    Check_RxData = copy.deepcopy(Info_355B)
                    print('版本355B参数有差异的名称：')
                    ConsoleDisplay('版本355B参数有差异的名称：')
                # 版本号为 355C
                if current_text == "355C":
                    Check_RxData = copy.deepcopy(Info_355C)
                    print('版本355C参数有差异的名称：')
                    ConsoleDisplay('版本355C参数有差异的名称：')
                    # 对比当前读到的参数和标准的参数，并将有差异的键提出
                if current_text == '355' or current_text == '355B' or current_text == '355C':
                    diff = Check_RxData.keys() & current_RxData
                    ui.textEdit_2.clear()
                    for k in diff:
                        if Check_RxData[k] != current_RxData[k]:
                            diff_vals = (k, Check_RxData[k], current_RxData[k])
                            Error_sign += 1
                            print(diff_vals)
                            # 确保diff_vals列表中的元素都是字符串
                            str_diff_vals = [str(val) for val in diff_vals]
                            # 插入文本
                            ui.textEdit_2.insertPlainText(
                                '参数:' + str_diff_vals[0]+'\n'+'标准值:'+str_diff_vals[1] +'\n'+'当前值:' + str_diff_vals[2])
                            ui.textEdit_2.insertPlainText('\n')
                            ui.textEdit.insertPlainText(diff_vals[0]+'\n')
                            Error_parameter.append(diff_vals[0])
                # 355A 参数有范围值，需要和其它区分开
                if current_text == '355A':
                    diff = Check_RxData.keys() & current_RxData
                    ui.textEdit_2.clear()
                    for k in diff:
                        if Check_RxData[k] != current_RxData[k]:
                            diff_vals = (k, Check_RxData[k], current_RxData[k])
                            Error_sign += 1
                            print(diff_vals)
                            # 确保diff_vals列表中的元素都是字符串
                            str_diff_vals = [str(val) for val in diff_vals]
                            # 插入文本
                            ui.textEdit_2.insertPlainText(
                                '参数:' + str_diff_vals[0] + '\n' + '标准值:' + str_diff_vals[1] + '\n' + '当前值:' +
                                str_diff_vals[2])
                            ui.textEdit_2.insertPlainText('\n')
                            ui.textEdit.insertPlainText(diff_vals[0] + '\n')
                            Error_parameter.append(diff_vals[0])
                # 计算RGB的值，进行单独校验
                #  Red 标定值
                if ((7500 <= current_RxData['StClibColor_R_x']) or (current_RxData['StClibColor_R_x'] <= 6500) or
                        (3500 <= current_RxData['StClibColor_R_y']) or (current_RxData['StClibColor_R_y'] <= 2500) or
                        (18000 <= current_RxData['StClibColor_R_Y']) or (current_RxData['StClibColor_R_Y'] <= 9000)):
                    print("StClib Color(R):", current_RxData['StClibColor_R_x'], current_RxData['StClibColor_R_y'],
                          current_RxData['StClibColor_R_Y'])
                    ConsoleDisplay("StClib Color(R)")
                    ui.lineEdit_4.setText(str(current_RxData['StClibColor_R_x']))
                    ui.lineEdit_5.setText(str(current_RxData['StClibColor_R_y']))
                    ui.lineEdit_6.setText(str(current_RxData['StClibColor_R_Y']))
                    Error_sign += 1
                #  Green 标定值
                if ((2200 <= current_RxData['StClibColor_G_x']) or (current_RxData['StClibColor_G_x'] <= 1200) or
                        (7800 <= current_RxData['StClibColor_G_y']) or (current_RxData['StClibColor_G_y'] <= 6800) or
                        (30300 <= current_RxData['StClibColor_G_Y']) or (current_RxData['StClibColor_G_Y'] <= 15000)):
                    print("StClibColor(G):", current_RxData['StClibColor_G_x'], current_RxData['StClibColor_G_y'],
                          current_RxData['StClibColor_G_Y'])
                    ConsoleDisplay("StClib Color(G)")
                    ui.lineEdit_9.setText(str(current_RxData['StClibColor_G_x']))
                    ui.lineEdit_8.setText(str(current_RxData['StClibColor_G_y']))
                    ui.lineEdit_7.setText(str(current_RxData['StClibColor_G_Y']))
                    Error_sign += 1
                #  Blue 标定值
                if ((2000 <= current_RxData['StClibColor_B_x']) or (current_RxData['StClibColor_B_x'] <= 1000) or
                        (350 <= current_RxData['StClibColor_B_y']) or (current_RxData['StClibColor_B_y'] <= 150) or
                        (4150 <= current_RxData['StClibColor_B_Y']) or (current_RxData['StClibColor_B_Y'] <= 1750)):
                    print("StClibColor(B):", current_RxData['StClibColor_B_x'], current_RxData['StClibColor_B_y'],
                          current_RxData['StClibColor_B_Y'])
                    ConsoleDisplay("StClib Color(B)")
                    ui.lineEdit_12.setText(str(current_RxData['StClibColor_B_x']))
                    ui.lineEdit_11.setText(str(current_RxData['StClibColor_B_y']))
                    ui.lineEdit_10.setText(str(current_RxData['StClibColor_B_Y']))
                    Error_sign += 1

                # 检查开发者模式，进行单独校验
                Buffer = [0x7F, 0x03, 0x22, 0xA6, 0x13, 0xFF, 0xFF, 0xFF]
                if WriteFrame(Buffer) == LIN_EX_PASS:
                    ReadFrame()
                    developMode.extend(CheckRxLINMsg.Data[2:8])
                    if developMode[0] != 34 or developMode[1] != 166 or developMode[2] != 19 or developMode[3] != 255:
                        print("developMode:有反馈")
                        print(developMode)
                        Error_sign += 1
                if Error_sign == 0:
                    print('参数校验正确')
                    ui.lineEdit.setText('参数校验正确')
                    ui.textEdit_2.setStyleSheet("background-color:green")
                    ui.textEdit_2.setText("校验合格")
                    RxData.clear()
                    ConsoleDisplay('参数校验正确')
                else:
                    print('有 %d 个参数错误' % Error_sign)
                    print(Error_parameter)
                    ui.lineEdit.setText('有 %d 个参数错误' % Error_sign)
                    ui.textEdit_2.setStyleSheet("background-color:red")
                print(RxDatas)
                RxDatas.clear()
        else:
            ConsoleDisplay("pls connect product!")
        Buffer = [0x7F, 0x03, 0xB4, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF]
        WriteFrame(Buffer)
        ReadFrame()
        Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x13, 0x01, 0xFF, 0xFF]
        WriteFrame(Buffer)
        ReadFrame()
        Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x3D, 0x00, 0xFF, 0xFF]  # cnt软硬件计数清零
        WriteFrame(Buffer)
        ReadFrame()
        Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x3E, 0x00, 0xFF, 0xFF]
        WriteFrame(Buffer)
        ReadFrame()

def Parameter_update():
    WakeUp()
    # DID地址和NVM地址对应，并将键对应
    DID_NVM = {
        'Brightness Adjustment': 0x07,
        'DimmingType': 0x08,
        'Coordinate Shift X_bak': 0x25,
        'Coordinate Shift Y_bak': 0x26,
        'DimmingWhen0': 0x04,
        'DimmingFactor': 0x10,
        'gotoSleepDimmingTime': 0x05,
        'waiteForGotoSleep': 0x06,
        'p_log_dimmramape_L': 0x09,
        'p_log_dimmramape_H': 0x0A,
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
        #'产品型号':0x33,   不能随意更改
        'HW ver':0x35,
        'calibration_flag':0x41,
        'brightness adjust':0x42
    }
    if not RxData:
        ui.lineEdit_2.setText("请先校验参数")
        return
    # 将SW的checksum计算
    input_data = [RxData[3], RxData[4], RxData[5], RxData[6], RxData[7], RxData[8],
                  Check_RxData['Coordinate Shift X_bak'], Check_RxData['Coordinate Shift Y_bak'],
                  RxData[11], Check_RxData['DimmingWhen0'], Check_RxData['DimmingFactor'],
                  Check_RxData['gotoSleepDimmingTime'], Check_RxData['waiteForGotoSleep'],
                  Check_RxData['p_log_dimmramape_L'][0], Check_RxData['p_log_dimmramape_L'][1],
                  Check_RxData['p_log_dimmramape_L'][2], Check_RxData['p_log_dimmramape_L'][3],
                  Check_RxData['p_log_dimmramape_L'][4], Check_RxData['p_log_dimmramape_H'][0],
                  Check_RxData['p_log_dimmramape_H'][1], Check_RxData['p_log_dimmramape_H'][2],
                  Check_RxData['p_log_dimmramape_H'][3], RxData[25]]
    crc_result = calc_crc8_0x97(input_data)
    Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x13, 0x01, 0xFF, 0xFF]
    if WriteFrame(Buffer) == LIN_EX_PASS:
        # 有报文回复
        if ReadFrame() == LIN_EX_PASS:
            Buffer = [0x7F, 0x03, 0x22, 0xA6, 0x13, 0xFF, 0xFF, 0xFF]
            WriteFrame(Buffer)
            read_cnt = 0
            while ReadFrame() != LIN_EX_PASS or CheckRxLINMsg.Data[5] != 0x01:
                print('进入开发者模式失败，重新进入......')
                Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x13, 0x01, 0xFF, 0xFF]
                WriteFrame(Buffer)
                Buffer = [0x7F, 0x03, 0x22, 0xA6, 0x13, 0xFF, 0xFF, 0xFF]
                WriteFrame(Buffer)
                read_cnt += 1
                if read_cnt > 4:
                    print('超时，请重新更新参数')
                    ui.lineEdit_2.setText('超时，请重新更新参数')
                    return
            print('进入开发者模式')
            for Error_num in Error_parameter:
                for num in DID_NVM.keys():
                    if Error_num == num:
                        print(hex(DID_NVM[Error_num]).upper())
                        # p_log_dimmramape_L 有差异
                        if Error_num == 'p_log_dimmramape_L':
                            Buffer = [0x7F, 0x10, 0x08, 0x2E, 0xA6, DID_NVM[Error_num],
                                      Check_RxData['p_log_dimmramape_L'][0], Check_RxData['p_log_dimmramape_L'][1],]
                            WriteFrame(Buffer)
                            Buffer = [0x7F, 0x21,
                                      Check_RxData['p_log_dimmramape_L'][2], Check_RxData['p_log_dimmramape_L'][3],
                                      Check_RxData['p_log_dimmramape_L'][4], 0xFF, 0xFF, 0xFF]
                            WriteFrame(Buffer)
                            ReadFrame()
                        # p_log_dimmramape_H 有差异
                        elif Error_num == 'p_log_dimmramape_H':
                            Buffer = [0x7F, 0x10, 0x07, 0x2E, 0xA6, DID_NVM[Error_num],
                                      Check_RxData['p_log_dimmramape_H'][0], Check_RxData['p_log_dimmramape_H'][1]]
                            WriteFrame(Buffer)
                            Buffer = [0x7F, 0x21, Check_RxData['p_log_dimmramape_H'][2],
                                      Check_RxData['p_log_dimmramape_H'][3], 0xFF, 0xFF, 0xFF, 0xFF]
                            WriteFrame(Buffer)
                            ReadFrame()
                        # PartNo 有差异
                        elif Error_num == '产品型号':
                            sleep(0.1)
                            Buffer = [0x7F, 0x10, 0x0D, 0x2E, 0xA6, DID_NVM[Error_num], 0x38, 0x35]
                            WriteFrame(Buffer)
                            Buffer = [0x7F, 0x21, 0x44, 0x39, 0x34, 0x37, 0x33, 0x35]
                            WriteFrame(Buffer)
                            Buffer = [0x7F, 0x22, 0x35, Check_RxData['产品型号'][9], 0xFF, 0xFF, 0xFF, 0xFF]
                            WriteFrame(Buffer)
                            ReadFrame()
                        # HW ver 有差异
                        elif Error_num == 'HW ver':
                            Buffer = [0x7F, 0x06, 0x2E, 0xA6, DID_NVM[Error_num], 0x48, 0x30, 0x32]
                            WriteFrame(Buffer)
                            ReadFrame()
                            # 超出范围的SW Parameters 更新
                        elif Error_num == 'Coordinate Shift X_bak' or Error_num == 'Coordinate Shift Y_bak' :
                            Buffer = [0x7F, 0x06, 0xB2, 0x33, 0x13, 0x00, 0xFE, 0xCA]  # 进入底层开发者模式
                            # 报文发送成功
                            if WriteFrame(Buffer) == LIN_EX_PASS:
                                # 有报文回复
                                if ReadFrame() == LIN_EX_PASS and CheckRxLINMsg.Data[2] == 0xF2:
                                    Buffer = [0x7F, 0x06, 0xB4, 0x0D, 0x17, 0xA8, 0x00, 0x0A]  # 写秘钥
                                    WriteFrame(Buffer)
                                    if ReadFrame() == LIN_EX_PASS and CheckRxLINMsg.Data[2] == 0xF4:
                                        Buffer = [0x7F, 0x10, 0x0C, 0xB4, 0xD3, 0x48, 0x56, 0xE4]  # 写秘钥
                                        WriteFrame(Buffer)
                                        Buffer = [0x7F, 0x21, 0xA5, 0xE3, 0xA5, 0x07, 0x00, 0x4A]  # 写秘钥
                                        WriteFrame(Buffer)
                                        Buffer = [0x7F, 0x22, 0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]  # 写秘钥
                                        WriteFrame(Buffer)
                                        if ReadFrame() == LIN_EX_PASS and CheckRxLINMsg.Data[2] == 0xF4:
                                            Buffer = [0x7F, 0x06, 0xB4, 0x47, 0x08, 0x40, 0x00, 0x08]  # 写 EEPROM
                                            WriteFrame(Buffer)
                                            ReadFrame()
                                            # 写 DATA
                                            Buffer = [0x7F, 0x10, 0x0A, 0xB4, 0xD3, Check_RxData['RomValid Flag msb'],
                                                      Check_RxData['RomValid Flag lsb'], RxData[5]]
                                            WriteFrame(Buffer)
                                            Buffer = [0x7F, 0x21, RxData[6], RxData[7],
                                                      RxData[8],
                                                      Check_RxData['Coordinate Shift X_bak'],
                                                      Check_RxData['Coordinate Shift Y_bak'], 0xFF]
                                            WriteFrame(Buffer)
                                            ReadFrame()
                                            Buffer = [0x7F, 0x06, 0xB4, 0x47, 0x08, 0x48, 0x00, 0x08]  # 写 EEPROM
                                            WriteFrame(Buffer)
                                            ReadFrame()
                                            # 写 DATA
                                            Buffer = [0x7F, 0x10, 0x0A, 0xB4, 0xD3, RxData[11],
                                                      Check_RxData['DimmingWhen0'], Check_RxData['DimmingFactor']]
                                            WriteFrame(Buffer)
                                            Buffer = [0x7F, 0x21, Check_RxData['gotoSleepDimmingTime'],
                                                      Check_RxData['waiteForGotoSleep'],
                                                      Check_RxData['p_log_dimmramape_L'][0],
                                                      Check_RxData['p_log_dimmramape_L'][1],
                                                      Check_RxData['p_log_dimmramape_L'][2],  0xFF]
                                            WriteFrame(Buffer)
                                            ReadFrame()
                                            Buffer = [0x7F, 0x06, 0xB4, 0x47, 0x08, 0x50, 0x00, 0x08]  # 写 EEPROM
                                            WriteFrame(Buffer)
                                            ReadFrame()
                                            # 写 DATA
                                            Buffer = [0x7F, 0x10, 0x0A, 0xB4, 0xD3, Check_RxData['p_log_dimmramape_L'][3],
                                                      Check_RxData['p_log_dimmramape_L'][4], Check_RxData['p_log_dimmramape_H'][0]]
                                            WriteFrame(Buffer)
                                            Buffer = [0x7F, 0x21, Check_RxData['p_log_dimmramape_H'][1],
                                                      Check_RxData['p_log_dimmramape_H'][2],
                                                      Check_RxData['p_log_dimmramape_H'][3], 0xFF, crc_result, 0xFF]
                                            WriteFrame(Buffer)
                                            ReadFrame()
                                            Buffer = [0x7F, 0x03, 0xB4, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF]
                                            WriteFrame(Buffer)
                        else:
                            # 单个数据出错进行单帧更新
                            Buffer = [0x7F, 0x04, 0x2E, 0xA6, DID_NVM[Error_num], Check_RxData[Error_num], 0xFF,
                                      0xFF]
                            WriteFrame(Buffer)
                            ReadFrame()
            RxData.clear()
            Error_parameter.clear()
    Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x3D, 0x00, 0xFF, 0xFF]  # cnt软硬件计数清零
    WriteFrame(Buffer)
    Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x3E, 0x00, 0xFF, 0xFF]
    WriteFrame(Buffer)
    Buffer = [0x7F, 0x04, 0x2E, 0xA6, 0x13, 0x00, 0xFF, 0xFF]  # 退出开发者模式
    WriteFrame(Buffer)
    ReadFrame()
    print("参数更新完成，退出开发者模式")
    ui.lineEdit_2.setText('参数更新完成')
    ConsoleDisplay("参数更新完成，退出开发者模式")


# checksum的计算方法
def calc_crc8_0x97(data):
    crc = 0xFF
    if data is not None:
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x97
                else:
                    crc = crc << 1
    return crc & 0xFF


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
    ui.comboBox.addItems(['355', '355A', '355B', '355C'])
    sys.exit(app.exec_())
