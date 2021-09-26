# -*- coding: utf-8 -*
'''
  @file DFRobot_URM13.py
  @brief Define the infrastructure of DFRobot_URM13 class.
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0.0
  @date  2021-09-22
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_URM13
'''
import sys
import time

import smbus

import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import logging
from ctypes import *


logger = logging.getLogger()
logger.setLevel(logging.INFO)  #Display all print information
#logger.setLevel(logging.FATAL)  #If you don’t want to display too many prints, only print errors, please use this option
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter) 
logger.addHandler(ph)

URM13_DEFAULT_ADDR_IIC   = 0x12        # 默认的IIC通信地址
URM13_DEFAULT_ADDR_RTU   = 0x000D      # 默认的RTU通信地址

# URM13 IIC register address
URM13_ADDR_REG_IIC               = 0x00   # 传感器IIC地址寄存器, 断电保存, 重启后生效, 默认值0x12
URM13_PID_REG_IIC                = 0x01   # 传感器PID寄存器, 该位用于产品校验[可实现传感器类型的检测], 默认值0x02
URM13_VID_REG_IIC                = 0x02   # 传感器VID寄存器, 固件版本号: 默认值0x10代表V1.0
URM13_DISTANCE_MSB_REG_IIC       = 0x03   # 距离值寄存器高位, 刻度为1cm
URM13_DISTANCE_LSB_REG_IIC       = 0x04   # 距离值寄存器低位
URM13_INTERNAL_TEMP_MSB_REG_IIC  = 0x05   # 板载温度值寄存器高位, 刻度为0.1℃, 数据类型有符号
URM13_INTERNAL_TEMP_LSB_REG_IIC  = 0x06   # 板载温度值寄存器低位
URM13_EXTERNAL_TEMP_MSB_REG_IIC  = 0x07   # 外部温度补偿数据寄存器高位, 写入环境温度数据到该寄存器用于外部温度补偿, 刻度为0.1℃, 数据类型有符号
URM13_EXTERNAL_TEMP_LSB_REG_IIC  = 0x08   # 外部温度补偿数据寄存器低位
URM13_CONFIG_REG_IIC             = 0x09   # 配置寄存器, 断电保存,立即生效, 默认值0x04
URM13_COMMAND_REG_IIC            = 0x0A   # 命令寄存器, 向该位写1, 触发一次测距, 向该位写0被忽略
# 电源噪声等级寄存器, 0x00-0x0A对应噪声等级0-10。 该参数能够反映供电电源以及环境对传感器的影响程度。 噪声等级越小, 传感器得到的距离值将更精准
URM13_NOISE_REG_IIC              = 0x0B
# 测距灵敏度设置寄存器, 0x00-0x0A:灵敏度等级0-10。 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高。 断电保存, 立即生效
URM13_SENSITIVITY_REG_IIC        = 0x0C

# URM13 RTU register address
URM13_PID_REG_RTU                 = 0x00   # 模块的PID存储寄存器, 该位用于产品校验[可实现模块类型的检测], 默认值0x0003
URM13_VID_REG_RTU                 = 0x01   # 模块的VID存储寄存器, 该位用于版本校验[0x0010表示V0.0.1.0]
URM13_ADDR_REG_RTU                = 0x02   # 模块地址寄存器, 默认值0x000D, 模块的设备地址(1~247), 断电保存, 重启后生效
URM13_BAUDRATE_REG_RTU            = 0x03   # 模块的波特率存储寄存器, 默认值0x0005, 断电保存, 重启后生效
URM13_CHECKBIT_STOPBIT_REG_RTU    = 0x04   # 模块校验位和停止位存储寄存器, 默认值0x0001, 断电保存, 重启后生效
URM13_DISTANCE_REG_RTU            = 0x05   # 距离值寄存器, 刻度为1cm
URM13_INTERNAL_TEMP_REG_RTU       = 0x06   # 板载温度值寄存器, 刻度为0.1℃, 数据类型有符号
URM13_EXTERNAL_TEMP_REG_RTU       = 0x07   # 外部温度补偿数据寄存器, 写入环境温度数据到该寄存器用于外部温度补偿, 刻度为0.1℃, 数据类型有符号
URM13_CONFIG_REG_RTU              = 0x08   # 配置寄存器, 断电保存,立即生效, 默认值0x04
# 电源噪声等级寄存器, 0x00-0x0A对应噪声等级0-10。 该参数能够反映供电电源以及环境对传感器的影响程度。 噪声等级越小, 传感器得到的距离值将更精准
URM13_NOISE_REG_RTU               = 0x09
# 测距灵敏度设置寄存器, 0x00-0x0A:灵敏度等级0-10。 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高。 断电保存, 立即生效
URM13_SENSITIVITY_REG_RTU         = 0x0A


class DFRobot_URM13(object):

    E_INTERNAL_TEMP = 0
    E_EXTERNAL_TEMP = 1

    E_TEMP_COMP_MODE_EN = 0
    E_TEMP_COMP_MODE_DIS = 1<<1

    E_AUTO_MEASURE_MODE_EN = 0
    E_AUTO_MEASURE_MODE_DIS = 1<<2

    E_MEASURE_RANGE_MODE_LONG = 0
    E_MEASURE_RANGE_MODE_SHORT = 1<<4

    E_RTU_INTERFACE = 0
    E_I2C_INTERFACE = 1
    # E_TRIG_INTERFACE = 2

    '''
      @brief Module init
    '''
    def __init__(self, interface_URM13):
        '''初始化配置参数'''
        self._device_interface = interface_URM13

    '''
      @brief Initialize sensor
      @return Return True indicate initialization succeed, return False indicate failed
    '''
    def begin(self):
        ret = True

        if self._device_interface == self.E_RTU_INTERFACE:
            id = self._read_reg(URM13_PID_REG_RTU, 1)[0]
        elif self._device_interface == self.E_I2C_INTERFACE:
            id = self._read_reg(URM13_PID_REG_IIC, 1)[0]
        else:
          id = 0

        logger.info(id)
        if id not in  [0x02, 0x0003]:
            ret = False

        return ret

    '''
      @brief 读取模块基本信息
      @param pbuf 读取到的数据的存放地址
                  RTU接口模式:
                    第一个元素为: 模块的PID
                    第二个元素为: 模块的VID, 固件版本号
                    第三个元素为: 模块的通信地址
                    第四个元素为：模块的波特率
                    第五个元素为：模块校验位和停止位
                  IIC接口模式:
                    第一个元素为: 模块的通信地址
                    第二个元素为: 模块的PID
                    第三个元素为: 模块的VID, 固件版本号
    '''
    def read_basic_info(self):
        if self._device_interface == self.E_RTU_INTERFACE:
            info_buf = self._read_reg(URM13_PID_REG_RTU, 5)   # rtu基本信息长度为5个16位数据
        elif self._device_interface == self.E_I2C_INTERFACE:
            info_buf = self._read_reg(URM13_ADDR_REG_IIC, 3)   # IIC基本信息长度为3个字节
        else:
            info_buf = []

        return info_buf

    '''
      @brief 设置模块的通信地址, 断电保存, 重启后生效
      @param addr 要设置的设备地址, IIC地址范围(1~127即0x01~0x7F), RTU地址范围(1~247即0x0001-0x00F7)
    '''
    def set_addr(self, addr):
        if self._device_interface == self.E_RTU_INTERFACE:
            if(0x0001 <= addr) and (0x00F7 >= addr):
              self._write_reg(URM13_ADDR_REG_RTU, [addr])

        elif self._device_interface == self.E_I2C_INTERFACE:
            if(0x01 <= addr) and (0x7F >= addr):
              self._write_reg(URM13_ADDR_REG_IIC, [addr])

    '''
      @brief 读取当前距离测量值
      @return 当前距离测量值, 单位cm, 分辨率1cm, 大量程测距范围(40 - 900cm)小量程测距范围(15-150cm)
    '''
    def get_distance_cm(self):
        if self._device_interface == self.E_RTU_INTERFACE:
            distance = self._read_reg(URM13_DISTANCE_REG_RTU, 1)[0]

        elif self._device_interface == self.E_I2C_INTERFACE:
            data = self._read_reg(URM13_DISTANCE_MSB_REG_IIC, 2)
            distance = (data[0] << 8) | data[1]
        else:
            distance = 0

        return distance

    '''
      @brief 读取当前板载温度
      @return 当前板载温度值, 单位℃, 分辨率0.1℃,有符号数
    '''
    def get_internal_tempreture_C(self):
        if self._device_interface == self.E_RTU_INTERFACE:
            internal_temp = self._read_reg(URM13_INTERNAL_TEMP_REG_RTU, 1)[0]

        elif self._device_interface == self.E_I2C_INTERFACE:
            data = self._read_reg(URM13_INTERNAL_TEMP_MSB_REG_IIC, 2)
            internal_temp = (data[0] << 8) | data[1]
        else:
            internal_temp = 0

        return self._uint16_to_int(internal_temp) / 10.0

    '''
      @brief 写入环境温度数据用于外部温度补偿
      @param temp 写入的环境温度数据, 单位℃, 分辨率0.1℃,有符号数
    '''
    def set_external_tempreture_C(self, temp=0.0):
        external_temp = int(temp * 10)

        if self._device_interface == self.E_RTU_INTERFACE:
            self._write_reg(URM13_EXTERNAL_TEMP_REG_RTU, [external_temp])
        elif self._device_interface == self.E_I2C_INTERFACE:
            data = [(external_temp & 0xFF00) >> 8, external_temp & 0x00FF]
            self._write_reg(URM13_EXTERNAL_TEMP_MSB_REG_IIC, data)

    '''
      @brief 设置测量相关模式
      @param mode 需要设置的测量相关模式, 下列模式相加为mode:
               E_INTERNAL_TEMP: 使用板载温度补偿功能, E_EXTERNAL_TEMP: 使用外部温度补偿功能(需用户写入外部温度)
               E_TEMP_COMP_MODE_EN: 开启温度补偿功能, E_TEMP_COMP_MODE_DIS: 关闭温度补偿功能
               E_AUTO_MEASURE_MODE_EN: 自动测距, E_AUTO_MEASURE_MODE_DIS: 被动测距
               E_MEASURE_RANGE_MODE_LONG: 大量程测距(40 - 900cm), E_MEASURE_RANGE_MODE_SHORT: 小量程测距(15-150cm)
    '''
    def set_measure_mode(self, mode=0):
        if self._device_interface == self.E_RTU_INTERFACE:
            self._write_reg(URM13_CONFIG_REG_RTU, [mode])
        elif self._device_interface == self.E_I2C_INTERFACE:
            self._write_reg(URM13_CONFIG_REG_IIC, [mode])


    '''
      @brief 被动测量模式下的触发测量函数
      @n 在被动测量模式下, 调用一次此函数, 发送一次测距命令, 模块测量一次距离并将测量的距离值存入距离寄存器
    '''
    def passive_measurement_TRIG(self):
        if self._device_interface == self.E_RTU_INTERFACE:
            data = self._read_reg(URM13_CONFIG_REG_RTU, 1)
            data[0] |= (1 << 3)
            self._write_reg(URM13_CONFIG_REG_RTU, data)

        elif self._device_interface == self.E_I2C_INTERFACE:
            data = [0x01]
            self._write_reg(URM13_COMMAND_REG_IIC, data)

        time.sleep(0.1)

    '''
      @brief 获取电源噪声等级, 0x00-0x0A对应噪声等级0-10
      @n 该参数能够反映供电电源以及环境对传感器的影响程度。噪声等级越小, 传感器得到的距离值将更精准。
    '''
    def get_noise_level(self):
        if self._device_interface == self.E_RTU_INTERFACE:
            noise_level = self._read_reg(URM13_NOISE_REG_RTU, 1)[0]

        elif self._device_interface == self.E_I2C_INTERFACE:
            noise_level = self._read_reg(URM13_NOISE_REG_IIC, 1)[0]
        else:
            noise_level = 0

        return noise_level

    '''
      @brief 测距灵敏度设置, 0x00-0x0A:灵敏度等级0-10
      @param mode 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高, 断电保存, 立即生效
    '''
    def set_measure_sensitivity(self, measure_sensitivity):
        if self._device_interface == self.E_RTU_INTERFACE:
            self._write_reg(URM13_SENSITIVITY_REG_RTU, [measure_sensitivity])
        elif self._device_interface == self.E_I2C_INTERFACE:
            self._write_reg(URM13_SENSITIVITY_REG_IIC, [measure_sensitivity])

    '''
      @brief Convert the incoming uint16 type data to int type
      @return data converted to int type
    '''
    def _uint16_to_int(self, num):
        if(num > 32767):
            num = num - 65536
        return num

    '''
      @brief writes data to a register
      @param reg register address
             data written data
    '''
    def _write_reg(self, reg, data):
        '''Low level register writing, not implemented in base class'''
        raise NotImplementedError()

    '''
      @brief read the data from the register
      @param reg register address
             length read data length
    '''
    def _read_reg(self, reg, length):
        '''Low level register writing, not implemented in base class'''
        raise NotImplementedError()


'''@brief An example of an i2c interface module'''

class DFRobot_URM13_I2C(DFRobot_URM13):
    '''
      @brief Module I2C communication init
    '''
    def __init__(self, i2c_addr=0x12, bus=1, interface_URM13=DFRobot_URM13.E_I2C_INTERFACE):
        self._i2c_addr = i2c_addr
        self._i2c = smbus.SMBus(bus)
        super(DFRobot_URM13_I2C, self).__init__(interface_URM13)

    '''
      @brief writes data to a register
      @param reg register address
             data written data
    '''
    def _write_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        self._i2c.write_i2c_block_data(self._i2c_addr, reg, data)

    '''
      @brief read the data from the register
      @param reg register address
             length read data length
    '''
    def _read_reg(self, reg, length):
        return self._i2c.read_i2c_block_data(self._i2c_addr, reg, length)


'''@brief An example of an modbus-rtu interface module'''

class DFRobot_URM13_RTU(DFRobot_URM13):

    E_BAUDRATE_2400 = 0x0001
    E_BAUDRATE_4800 = 0x0002
    E_BAUDRATE_9600 = 0x0003
    E_BAUDRATE_14400 = 0x0004
    E_BAUDRATE_19200 = 0x0005
    E_BAUDRATE_38400 = 0x0006
    E_BAUDRATE_57600 = 0x0007
    E_BAUDRATE_115200 = 0x0008

    E_CHECKBIT_NONE = 0x0000<<8
    E_CHECKBIT_EVEN = 0x0001<<8
    E_CHECKBIT_ODD = 0x0002<<8

    E_STOPBIT_0P5 = 0x0000
    E_STOPBIT_1 = 0x0001
    E_STOPBIT_1P5 = 0x0002
    E_STOPBIT_2 = 0x0003

    '''
      @brief Module RTU communication init
    '''
    def __init__(self, addr=0x000D, port="/dev/ttyAMA0", baud = 19200, bytesize = 8, parity = 'N', stopbit = 1, xonxoff=0, interface_URM13=DFRobot_URM13.E_RTU_INTERFACE):
        self._modbus_addr = addr

        self._DFRobot_RTU = modbus_rtu.RtuMaster(
            serial.Serial(port, baud, bytesize, parity, stopbit, xonxoff)
        )
        self._DFRobot_RTU.set_timeout(0.5)
        self._DFRobot_RTU.set_verbose(False)

        super(DFRobot_URM13_RTU, self).__init__(interface_URM13)

    '''
      @brief UART接口模式下，设置模块的波特率, 断电保存, 重启后生效
      @param mode 要设置的波特率：
             E_BAUDRATE_2400---2400, E_BAUDRATE_4800---4800, E_BAUDRATE_9600---9600, 
             E_BAUDRATE_14400---14400, E_BAUDRATE_19200---19200, E_BAUDRATE_38400---38400, 
             E_BAUDRATE_57600---57600, E_BAUDRATE_115200---115200
    '''
    def set_baudrate_mode(self, baudrate_mode=E_BAUDRATE_19200):
        self._write_reg(URM13_BAUDRATE_REG_RTU, [baudrate_mode])

    '''
      @brief UART接口模式下，设置模块的校验位和停止位
      @param mode 要设置的模式：
             校验位：
                  E_CHECKBIT_NONE
                  E_CHECKBIT_EVEN
                  E_CHECKBIT_ODD
             停止位：
                  E_STOPBIT_0P5
                  E_STOPBIT_1
                  E_STOPBIT_1P5
                  E_STOPBIT_2
    '''
    def set_checkbit_stopbit(self, checkbit_stopbit=E_CHECKBIT_NONE+E_STOPBIT_1):
        self._write_reg(URM13_CHECKBIT_STOPBIT_REG_RTU, [checkbit_stopbit])

    '''
      @brief writes data to a register
      @param reg register address
             data written data
      @return Write register address, and write length
    '''
    def _write_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
            #logger.info(data)
        ret = self._DFRobot_RTU.execute(self._modbus_addr, cst.WRITE_MULTIPLE_REGISTERS, reg, output_value=data)
        # logger.info(ret)
        return ret

    '''
      @brief read the data from the register
      @param reg register address
             length read data length
      @return list: The value list of the holding register.
    '''
    def _read_reg(self, reg, length):
        return list(self._DFRobot_RTU.execute(self._modbus_addr, cst.READ_HOLDING_REGISTERS, reg, length))
