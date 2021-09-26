/*!
 * @file        URM13WorkInTRIG.ino
 * @brief       这个demo演示了URM13在TRIG接口模式下工作
 * @n           可以获取传感器当前距离测量值, 并且可以通过IIC修改传感器测量参数
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

// 如果需要使用IIC更改传感器测量参数及模式, 就打开这个宏, 并配置下面的那些api
// #define IIC_CONFIG
#ifdef IIC_CONFIG
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
#endif

#if defined(ESP32) || defined(ESP8266)
  uint8_t trigPin = D2;   // I2C接口下作为外部端口测距触发引脚, 上升沿触发。
  uint8_t echoPin = D3;   // I2C接口下, 当传感器测距功能被触发后, 该脚引输出一个高脉宽, 此脉宽代表超声波传播时间（单位：us）。
#elif defined(ARDUINO_SAM_ZERO)
  uint8_t trigPin= 6;
  uint8_t echoPin= 7;
#else
  uint8_t trigPin= 8;
  uint8_t echoPin= 9;
#endif

// The ultrasonic velocity (cm/us) compensated by temperature
#define   VELOCITY_TEMP(temp)   ( ( 331.5 + 0.6 * (float)( temp ) ) * 100 / 1000000.0 )

void setup()
{
  Serial.begin(115200);

  pinMode(trigPin,OUTPUT);   // 初始化trigPin和echoPin两个IO口
  digitalWrite(trigPin,LOW);
  pinMode(echoPin,INPUT);

  Serial.println("TRIG pin begin ok!");

#ifdef IIC_CONFIG
  // 初始化传感器
  while( NO_ERR != sensor.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("IIC begin ok!");

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
#endif

  delay(1000);
}

void loop()
{
  uint16_t  pulseWidthUs, distanceCm;
  digitalWrite(trigPin,HIGH);   //Set the tirgPin High
  delayMicroseconds(50);         //Delay of 50 microseconds
  digitalWrite(trigPin,LOW);    //Set the tirgPin Low

  /*
  Measure echo high level time, the output high level time represents the ultrasonic flight time (unit: us)
  The distance can be calculated according to the flight time of ultrasonic wave, 
  and the ultrasonic sound speed can be compensated according to the actual ambient temperature
  */
  pulseWidthUs = pulseIn(echoPin,HIGH);
  delayMicroseconds(50);   //Delay of 50 microseconds
  distanceCm = (uint16_t)(pulseWidthUs * VELOCITY_TEMP(30.0) / 2.0);

  Serial.print("This distance measurement value: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  Serial.println();
  delay(1000);
}
