# -*- coding: utf-8 -*
'''
  # @file      URM13_work_in_TRIG.py
  # @brief     这个demo演示了URM13在TRIG接口模式下工作
  # @n         可以获取和修改传感器基本信息、配置测量参数、获取当前距离测量值和当前温度测量值
  # @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  # @licence   The MIT License (MIT)
  # @author    [qsjhyy](yihuan.huang@dfrobot.com)
  # @version   V1.0.0
  # @date      2021-09-24
  # @get       from https://www.dfrobot.com
  # @url       https://github.com/DFRobot/DFRobot_URM13
'''
from __future__ import print_function
import sys
import RPi.GPIO as GPIO
sys.path.append('../')
from DFRobot_URM13 import *


'''
  # UART(Modbus-RTU)与I2C/TRIG模式切换
  # URM13传感器默认出厂设置为UART模式。 传感器可以在I2C、UART两种模式间通过上电前短接不同的引脚实现简单的模式切换：

  # I2C/TRIG: 在传感器上电前, 将TRIG与ECHO引脚短接, 上电后LED闪烁2次, 表示传感器已切换为I2C模式。
  # UART(Modbus-RTU): 在传感器上电前, 将TRIG与BUSY引脚短接, 上电后LED闪烁1次, 表示传感器已切换为UART(Modbus-RTU)模式。

  # 在模式切换成功后, 用户即可断开对应引脚的短接, 切换后的模式将被传感器记录保存, 永久生效。

  # 可以通过实例化一个IIC接口对象, 来配置传感器测量参数。
'''
# sensor = DFRobot_URM13_IIC(i2c_addr = 0x12, bus = 1)


global flag, echo_pin, echo_pin_high_start_ticks, echo_pin_high_end_ticks, speed_of_sound
flag = 0
echo_pin_high_start_ticks = 0
echo_pin_high_end_ticks = 0
speed_of_sound = 0
trig_pin = 20
echo_pin = 21


def int_callback(channel):
  global flag, echo_pin, echo_pin_high_start_ticks, echo_pin_high_end_ticks   # 全局变量声明
  if 1 == GPIO.input(echo_pin) and 0 == flag:
    echo_pin_high_start_ticks = time.time()
    flag = 1
  if 0 == GPIO.input(echo_pin) and 1 == flag:
    echo_pin_high_end_ticks = time.time()
    flag = 2


def delay_microsecond(microsecond):   # 微秒级延时函数
    start, end = 0, 0   # 声明变量
    start = time.time()       # 记录开始时间
    microsecond = (microsecond - 3) / 1000000   # 将输入t的单位转换为秒, -3是时间补偿
    while end-start < microsecond:  # 循环至时间差值大于或等于设定值时
        end = time.time()   # 记录结束时间



def setup():
  global echo_pin, trig_pin, speed_of_sound   # 全局变量声明
  GPIO.setwarnings(False)   # 关闭引脚设置等警告
  GPIO.setmode(GPIO.BCM)   # 设置引脚编码模式为BCM
  GPIO.setup(trig_pin, GPIO.OUT, initial=0)   # 设置测量触发引脚为输出模式, 初始化输出低电平
  GPIO.setup(echo_pin, GPIO.IN)   # 设置测量数据引脚为输入模式
  GPIO.add_event_detect(echo_pin, GPIO.BOTH, callback=int_callback)   # Use GPIO port to monitor sensor interrupt

  environment_temperature = 30.0   # 当前环境温度
  speed_of_sound = (331.5 + 0.6 * environment_temperature ) * 100  # 由给定的当前环境温度计算出的音速

  # while (sensor.begin() == False):
  #   print ('Please check that the device is properly connected')
  #   time.sleep(3)
  # print("sensor begin successfully!!!\n")

  '''
  * 设置测量相关模式
  * mode 需要设置的测量相关模式, 下列模式相加为mode:
  *   E_INTERNAL_TEMP: 使用板载温度补偿功能, E_EXTERNAL_TEMP: 使用外部温度补偿功能(需用户写入外部温度)
  *   E_TEMP_COMP_MODE_EN: 开启温度补偿功能, E_TEMP_COMP_MODE_DIS: 关闭温度补偿功能
  *   E_AUTO_MEASURE_MODE_EN: 自动测距, E_AUTO_MEASURE_MODE_DIS: 被动测距
  *   E_MEASURE_RANGE_MODE_LONG: 大量程测距(40 - 900cm), E_MEASURE_RANGE_MODE_SHORT: 小量程测距(15-150cm)
  '''
  # sensor.set_measure_mode(sensor.E_INTERNAL_TEMP + 
  #                         sensor.E_TEMP_COMP_MODE_EN + 
  #                         sensor.E_AUTO_MEASURE_MODE_DIS + 
  #                         sensor.E_MEASURE_RANGE_MODE_LONG)

  '''
  * 写入环境温度数据用于外部温度补偿
  * temp 写入的环境温度数据, 单位℃, 分辨率0.1℃,有符号数
  '''
  # sensor.set_external_tempreture_C(30.0)

  '''
  * 测距灵敏度设置, 0x00-0x0A:灵敏度等级0-10
  * mode 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高, 断电保存, 立即生效
  '''
  # sensor.set_measure_sensitivity(0x00)

  print()
  time.sleep(1.5)


def loop():
  global flag, echo_pin, trig_pin, speed_of_sound, echo_pin_high_start_ticks, echo_pin_high_end_ticks   # 全局变量声明
  GPIO.output(trig_pin, GPIO.HIGH)   # Set the trig_pin High
  delay_microsecond(50)   # Delay of 50 microseconds
  GPIO.output(trig_pin, GPIO.LOW)   # Set the trig_pin Low

  for i in range(1000):
    if flag == 2:
        break

  if flag == 2:
    # Measure echo high level time, the output high level time represents the ultrasonic flight time (unit: us)
    measuring_time = echo_pin_high_end_ticks - echo_pin_high_start_ticks
    flag = 0
    # print(measuring_time)
    # print(speed_of_sound)

    '''
    * 计算当前距离测量值
    * 注意：当物体所在的位置不在传感器测距范围内, 会使读出的测量数据无意义
    * 当前距离测量值, 单位cm, 大量程测距范围(40 - 900cm)小量程测距范围(15-150cm)
    * The distance can be calculated according to the flight time of ultrasonic wave,
    * and the ultrasonic sound speed can be compensated according to the actual ambient temperature
    '''
    measuring_distance = speed_of_sound * measuring_time / 2.0
    print("Current distance measurement: %d cm" %measuring_distance)
    print()

  time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()