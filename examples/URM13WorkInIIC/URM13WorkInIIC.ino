/*!
 * @file        URM13WorkInIIC.ino
 * @brief       这个demo演示了URM13在IIC接口模式下工作
 * @n           可以获取和修改传感器基本信息、配置测量参数、获取当前距离测量值和当前温度测量值
 * 
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author      [qsjhyy](yihuan.huang@dfrobot.com)
 * @version     V1.0.0
 * @date        2021-09-18
 * @get from    https://www.dfrobot.com
 * @url         https://github.com/DFRobot/DFRobot_URM13
 */
#include <DFRobot_URM13.h>

/*!
 * UART(Modbus-RTU)与I2C/TRIG模式切换
 * URM13传感器默认出厂设置为UART模式。 传感器可以在I2C、UART两种模式间通过上电前短接不同的引脚实现简单的模式切换：

 * I2C/TRIG: 在传感器上电前, 将TRIG与ECHO引脚短接, 上电后LED闪烁2次, 表示传感器已切换为I2C模式。
 * UART(Modbus-RTU): 在传感器上电前, 将TRIG与BUSY引脚短接, 上电后LED闪烁1次, 表示传感器已切换为UART(Modbus-RTU)模式。

 * 在模式切换成功后, 用户即可断开对应引脚的短接, 切换后的模式将被传感器记录保存, 永久生效。
 */

/*
实例化一个对象, 来驱动我们的传感器;
*/
DFRobot_URM13_IIC sensor(/*iicAddr = */0x12, /*iicBus = */&Wire);

// 如果需要使用busyPin发出的数据测量完成信号作为测量等待, 就打开这个宏, 注意:需要根据使用的主控设置可以使用的引脚编号
// #define BUSYPIN_SIGNAL
#ifdef BUSYPIN_SIGNAL
  int16_t    busyPin = 4;   // "数据测量就绪信号"引脚, 可以根据需要选择可以使用的引脚
  #define    isSensorBusy()           (digitalRead(busyPin))
#endif

void setup()
{
  Serial.begin(115200);

#ifdef BUSYPIN_SIGNAL
  pinMode(busyPin, INPUT);   // 初始化"数据测量就绪信号"引脚为输入
#endif

  // 初始化传感器
  while( NO_ERR != sensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  uint8_t buf[3]={0};
  /**
  * 读取模块基本信息
  * pbuf 读取到的数据的存放地址, IIC接口模式:
  *                 第一个元素为: 模块的通信地址
  *                 第二个元素为: 模块的PID
  *                 第三个元素为: 模块的VID, 固件版本号
  */
  sensor.readBasicInfo(buf);

  /*模块的I2C从机地址, 默认值0x12, 模块的设备地址(1~127)*/
  Serial.print("mailing address: 0x");
  Serial.println(buf[0], HEX);

  /* 模块的PID, 默认值0x02 该位用于产品校验[可实现传感器类型的检测] */
  Serial.print("PID: 0x0");
  Serial.println(buf[1], HEX);

  /* 模块的VID, 固件版本号：0x10代表V1.0 */
  Serial.print("VID: 0x");
  Serial.println(buf[2], HEX);

  /**
  * 设置模块的通信地址, 断电保存, 重启后生效
  * addr 要设置的设备地址, IIC地址范围(1~127即0x01~0x7F)
  */
  sensor.setADDR(0x12);

  /**
  * 设置测量相关模式
  * mode 需要设置的测量相关模式, 下列模式相加为mode:
  *   eInternalTemp: 使用板载温度补偿功能, eExternalTemp: 使用外部温度补偿功能(需用户写入外部温度)
  *   eTempCompModeEn: 开启温度补偿功能, eTempCompModeDis: 关闭温度补偿功能
  *   eAutoMeasureModeEn: 自动测距, eAutoMeasureModeDis: 被动测距
  *   eMeasureRangeModeLong: 大量程测距(40 - 900cm), eMeasureRangeModeShort: 小量程测距(15-150cm)
  */
  sensor.setMeasureMode(sensor.eInternalTemp + 
                        sensor.eTempCompModeEn + 
                        sensor.eAutoMeasureModeDis + 
                        sensor.eMeasureRangeModeLong);

  /**
  * 写入环境温度数据用于外部温度补偿
  * temp 写入的环境温度数据, 单位℃, 分辨率0.1℃,有符号数
  */
  sensor.setExternalTempretureC(30.0);

  /**
  * 测距灵敏度设置, 0x00-0x0A:灵敏度等级0-10
  * mode 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高, 断电保存, 立即生效
  */
  sensor.setMeasureSensitivity(0x00);

  Serial.println();
  delay(1000);
}

void loop()
{
  /**
  * 被动测量模式下的触发测量函数
  * 在被动测量模式下, 调用一次此函数, 发送一次测距命令, 模块测量一次距离并将测量的距离值存入距离寄存器
  */
  sensor.passiveMeasurementTRIG();

#ifdef BUSYPIN_SIGNAL
  //You can replace the delay with these two lines of code
  while(isSensorBusy()== HIGH);  //Wait for the sensor to start ranging
  while(isSensorBusy()== LOW);   //Wait for sensor ranging to complete
#else
  delay(100);//delay 100ms
#endif

  /**
  * 获取电源噪声等级, 0x00-0x0A对应噪声等级0-10
  * 该参数能够反映供电电源以及环境对传感器的影响程度。噪声等级越小, 传感器得到的距离值将更精准。
  */
  Serial.print("Current ambient noise level: 0x0");
  Serial.println(sensor.getNoiseLevel(), HEX);

  /**
  * 读取当前板载温度
  * 当前板载温度值, 单位℃, 分辨率0.1℃,有符号数
  */
  Serial.print("The onboard temperature: ");
  Serial.print(sensor.getInternalTempretureC());
  Serial.println(" C");

  /**
  * 读取当前距离测量值
  * 注意：当物体所在的位置不在传感器测距范围内, 会使读出的测量数据无意义
  * 当前距离测量值, 单位cm, 大量程测距范围(40 - 900cm)小量程测距范围(15-150cm)
  */
  Serial.print("Current distance measurement: ");
  Serial.print(sensor.getDistanceCm());
  Serial.println(" cm");

  Serial.println();
  delay(1000);
}
