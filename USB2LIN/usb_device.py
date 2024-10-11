"""
文件说明：USB2XXX设备操作相关函数集合
更多帮助：www.usbxyz.com
"""

from ctypes import *
import platform


# Device info define
class DEVICE_INFO(Structure):
    _fields_ = [
        ("FirmwareName", c_char * 32),  # firmware name string
        ("BuildDate", c_char * 32),  # firmware build date and time string
        ("HardwareVersion", c_uint),  # hardware version
        ("FirmwareVersion", c_uint),  # firmware version
        ("SerialNumber", c_uint * 3),  # USB2XXX serial number
        ("Functions", c_uint)  # USB2XXX functions
    ]


# 定义电压输出值
POWER_LEVEL_NONE = 0  # 不输出
POWER_LEVEL_1V8 = 1  # 输出1.8V
POWER_LEVEL_2V5 = 2  # 输出2.5V
POWER_LEVEL_3V3 = 3  # 输出3.3V
POWER_LEVEL_5V0 = 4  # 输出5.0V

# Import library
if (platform.system() == "Windows"):
    if "64bit" in platform.architecture():
        windll.LoadLibrary("./USB2LIN/lib/windows/x86_64/libusb-1.0.dll")
        USB2XXXLib = windll.LoadLibrary("./USB2LIN/lib/windows/x86_64/USB2XXX.dll")
    else:
        windll.LoadLibrary("./USB2LIN/lib/windows/x86_32/libusb-1.0.dll")
        USB2XXXLib = windll.LoadLibrary("./USB2LIN/lib/windows/x86_32/USB2XXX.dll")
elif (platform.system() == "Darwin"):
    cdll.LoadLibrary("./USB2LIN/lib/macos/libusb-1.0.dylib")
    USB2XXXLib = cdll.LoadLibrary("./USB2LIN/lib/macos/libUSB2XXX.dylib")
elif (platform.system() == "Linux"):
    if 'arm' in platform.machine():
        cdll.LoadLibrary("./USB2LIN/lib/linux/ARMv7/libusb-1.0.so")
        USB2XXXLib = cdll.LoadLibrary("./USB2LIN/lib/linux/ARMv7/libUSB2XXX.so")
    else:
        if "64bit" in platform.architecture():
            cdll.LoadLibrary("./USB2LIN/lib/linux/x86_64/libusb-1.0.so")
            USB2XXXLib = cdll.LoadLibrary("./USB2LIN/lib/linux/x86_64/libUSB2XXX.so")
        else:
            cdll.LoadLibrary("./USB2LIN/lib/linux/x86_64/libusb-1.0.so")
            USB2XXXLib = cdll.LoadLibrary("./USB2LIN/lib/linux/x86_32/libUSB2XXX.so")
else:
    print("unsupported system")
    exit()


# Scan device
def USB_ScanDevice(pDevHandle):
    return USB2XXXLib.USB_ScanDevice(pDevHandle)


# Open device
def USB_OpenDevice(DevHandle):
    return USB2XXXLib.USB_OpenDevice(DevHandle)


# Get USB2XXX infomation
def DEV_GetDeviceInfo(DevHandle, pDevInfo, pFunctionStr):
    return USB2XXXLib.DEV_GetDeviceInfo(DevHandle, pDevInfo, pFunctionStr)


# Close device
def USB_CloseDevice(DevHandle):
    return USB2XXXLib.USB_CloseDevice(DevHandle)


def DEV_EraseUserData(DevHandle):
    return USB2XXXLib.DEV_EraseUserData(DevHandle)


def DEV_WriteUserData(DevHandle, OffsetAddr, pWriteData, DataLen):
    return USB2XXXLib.DEV_WriteUserData(DevHandle, OffsetAddr, pWriteData, DataLen)


def DEV_ReadUserData(DevHandle, OffsetAddr, pReadData, DataLen):
    return USB2XXXLib.DEV_ReadUserData(DevHandle, OffsetAddr, pReadData, DataLen)


def DEV_SetPowerLevel(DevHandle, PowerLevel):
    return USB2XXXLib.DEV_SetPowerLevel(DevHandle, PowerLevel)
