/*!
 * @file DFRobot_URM13.h
 * @brief DFRobot_URM13.h detailed description for DFRobot_URM13.cpp
 * @n DFRobot_URM13.h 定义了设备信息寄存器和设备功能寄存器的读写函数, 声明了传感器的功能API
 * 
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @Version V1.0.0
 * @date 2021-09-16
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_URM13
 */
#ifndef __DFROBOT_URM13_H__
#define __DFROBOT_URM13_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <Stream.h>
#include <DFRobot_RTU.h>


// 设置宏ENABLE_DBG为1时, 可以看到程序的调试打印信息
#define ENABLE_DBG 1

#if ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif


#define URM13_DEFAULT_ADDR_IIC   uint8_t(0x12)        // 默认的IIC通信地址
#define URM13_DEFAULT_ADDR_RTU   uint16_t(0x000D)     // 默认的RTU通信地址

// URM13 IIC register address
#define URM13_ADDR_REG_IIC               uint8_t(0x00)   // 传感器IIC地址寄存器, 断电保存, 重启后生效, 默认值0x12
#define URM13_PID_REG_IIC                uint8_t(0x01)   // 传感器PID寄存器, 该位用于产品校验[可实现传感器类型的检测], 默认值0x02
#define URM13_VID_REG_IIC                uint8_t(0x02)   // 传感器VID寄存器, 固件版本号: 默认值0x10代表V1.0
#define URM13_DISTANCE_MSB_REG_IIC       uint8_t(0x03)   // 距离值寄存器高位, 刻度为1cm
#define URM13_DISTANCE_LSB_REG_IIC       uint8_t(0x04)   // 距离值寄存器低位
#define URM13_INTERNAL_TEMP_MSB_REG_IIC  uint8_t(0x05)   // 板载温度值寄存器高位, 刻度为0.1℃, 数据类型有符号
#define URM13_INTERNAL_TEMP_LSB_REG_IIC  uint8_t(0x06)   // 板载温度值寄存器低位
#define URM13_EXTERNAL_TEMP_MSB_REG_IIC  uint8_t(0x07)   // 外部温度补偿数据寄存器高位, 写入环境温度数据到该寄存器用于外部温度补偿, 刻度为0.1℃, 数据类型有符号
#define URM13_EXTERNAL_TEMP_LSB_REG_IIC  uint8_t(0x08)   // 外部温度补偿数据寄存器低位
#define URM13_CONFIG_REG_IIC             uint8_t(0x09)   // 配置寄存器, 断电保存,立即生效, 默认值0x04
#define URM13_COMMAND_REG_IIC            uint8_t(0x0A)   // 命令寄存器, 向该位写1, 触发一次测距, 向该位写0被忽略
// 电源噪声等级寄存器, 0x00-0x0A对应噪声等级0-10。 该参数能够反映供电电源以及环境对传感器的影响程度。 噪声等级越小, 传感器得到的距离值将更精准
#define URM13_NOISE_REG_IIC              uint8_t(0x0B)
// 测距灵敏度设置寄存器, 0x00-0x0A:灵敏度等级0-10。 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高。 断电保存, 立即生效
#define URM13_SENSITIVITY_REG_IIC        uint8_t(0x0C)

// URM13 RTU register address
#define URM13_PID_REG_RTU                 uint16_t(0x00)   // 模块的PID存储寄存器, 该位用于产品校验[可实现模块类型的检测], 默认值0x0003
#define URM13_VID_REG_RTU                 uint16_t(0x01)   // 模块的VID存储寄存器, 该位用于版本校验[0x0010表示V0.0.1.0]
#define URM13_ADDR_REG_RTU                uint16_t(0x02)   // 模块地址寄存器, 默认值0x000D, 模块的设备地址(1~247), 断电保存, 重启后生效
#define URM13_BAUDRATE_REG_RTU            uint16_t(0x03)   // 模块的波特率存储寄存器, 默认值0x0005, 断电保存, 重启后生效
#define URM13_CHECKBIT_STOPBIT_REG_RTU    uint16_t(0x04)   // 模块校验位和停止位存储寄存器, 默认值0x0001, 断电保存, 重启后生效
#define URM13_DISTANCE_REG_RTU            uint16_t(0x05)   // 距离值寄存器, 刻度为1cm
#define URM13_INTERNAL_TEMP_REG_RTU       uint16_t(0x06)   // 板载温度值寄存器, 刻度为0.1℃, 数据类型有符号
#define URM13_EXTERNAL_TEMP_REG_RTU       uint16_t(0x07)   // 外部温度补偿数据寄存器, 写入环境温度数据到该寄存器用于外部温度补偿, 刻度为0.1℃, 数据类型有符号
#define URM13_CONFIG_REG_RTU              uint16_t(0x08)   // 配置寄存器, 断电保存,立即生效, 默认值0x04
// 电源噪声等级寄存器, 0x00-0x0A对应噪声等级0-10。 该参数能够反映供电电源以及环境对传感器的影响程度。 噪声等级越小, 传感器得到的距离值将更精准
#define URM13_NOISE_REG_RTU               uint16_t(0x09)
// 测距灵敏度设置寄存器, 0x00-0x0A:灵敏度等级0-10。 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高。 断电保存, 立即生效
#define URM13_SENSITIVITY_REG_RTU         uint16_t(0x0A)

#define    VELOCITY_TEMP(temp)       ( ( 331.5 + 0.6 * (float)( temp ) ) * 100 / 1000000.0 ) // The ultrasonic velocity (cm/us) compensated by temperature


class DFRobot_URM13
{
public:
  #define NO_ERR             0   // No error
  #define ERR_DATA_BUS      (-1)   // 数据总线错误
  #define ERR_IC_VERSION    (-2)   // 芯片版本不匹配

  /*
    控制寄存器
    * ----------------------------------------------------------------------------------------------------
    * | b7 | b6 | b5 |        b4        |       b3       |        b2       |      b1      |      b0      |
    * ----------------------------------------------------------------------------------------------------
    * |   reserved   | measureRangeMode | measureTrigger | autoMeasureMode | tempCompMode | tempCompPick |
    * ----------------------------------------------------------------------------------------------------
  */
  typedef struct
  {
    uint8_t   tempCompPick: 1; /*!< 上电为0, 0:使用板载温度补偿功能, 1:使用外部温度补偿功能(需用户写入外部温度) */
    uint8_t   tempCompMode: 1; /*!< 上电为0, 0:开启温度补偿功能, 1:关闭温度补偿功能 */
    uint8_t   autoMeasureMode: 1; /*!< 上电为0, 0:自动测距, 1:被动测距 */
    uint8_t   measureTrigger: 1; /*!< 上电为0, 被动模式下, 向该位写入1, 传感器将完成一次测距, 该位置1后将自动清0 */
    uint8_t   measureRangeMode: 1; /*!<  上电为0, 0:大量程测距(40-900cm), 1:小量程测距(15-150cm), 断电保存, 立即生效 */
    uint8_t   reserved: 3; /*!< 保留位 */
  } __attribute__ ((packed)) sPWRCTRL_t;

  /*
    温度补偿的温度来源选择
  */
  typedef enum
  {
    eInternalTemp,   /**< 使用板载温度补偿功能 */
    eExternalTemp = 1,  /**< 使用外部温度补偿功能(需用户写入外部温度) */
  }eTempCompPick_t;

  /*
    温度补偿使能
  */
  typedef enum
  {
    eTempCompModeEn,   /**< 开启温度补偿功能 */
    eTempCompModeDis = 1<<1,   /**< 关闭温度补偿功能 */
  }eTempCompMode_t;

  /*
    测量模式选择
  */
  typedef enum
  {
    eAutoMeasureModeEn,   /**< 自动测距 */
    eAutoMeasureModeDis = 1<<2,   /**< 被动测距 */
  }eAutoMeasureMode_t;

  /*
    测量范围选择
  */
  typedef enum
  {
    eMeasureRangeModeLong,   /**< 大量程测距(40 - 900cm) */
    eMeasureRangeModeShort = 1<<4,   /**< 小量程测距(15-150cm) */
  }eMeasureRangeMode_t;

  /*
    接口模式选择
  */
  typedef enum
  {
    eRtuInterface = 0,   /**< modbusRTU通信模式 */
    eI2cInterface,   /**< IIC通信模式 */
    // eTrigInterface,   /**< TRIG通信模式 */
  }eInterfaceMode_t;

public:
  /**
  * @brief 构造函数
  */
  DFRobot_URM13(eInterfaceMode_t interfaceURM13);

  /**
  * @brief 初始化函数
  * @return 返回0表示初始化成功, 返回其他值表示初始化失败, 返回错误码
  */
  virtual int begin(void);

  /**
  * @brief 读取模块基本信息
  * @param pbuf 读取到的数据的存放地址
  *             RTU接口模式:
  *               第一个元素为: 模块的PID
  *               第二个元素为: 模块的VID, 固件版本号
  *               第三个元素为: 模块的通信地址
  *               第四个元素为：模块的波特率
  *               第五个元素为：模块校验位和停止位
  *             IIC接口模式:
  *               第一个元素为: 模块的通信地址
  *               第二个元素为: 模块的PID
  *               第三个元素为: 模块的VID, 固件版本号
  */
  void readBasicInfo(uint8_t* pbuf);

  /**
  * @brief 设置模块的通信地址, 断电保存, 重启后生效
  * @param addr 要设置的设备地址, IIC地址范围(1~127即0x01~0x7F), RTU地址范围(1~247即0x0001-0x00F7)
  */
  void setADDR(uint8_t addr);

  /**
  * @brief 读取当前距离测量值
  * @return 当前距离测量值, 单位cm, 大量程测距范围(40 - 900cm)小量程测距范围(15-150cm)
  */
  uint16_t getDistanceCm(void);

  /**
  * @brief 读取当前板载温度
  * @return 当前板载温度值, 单位℃, 分辨率0.1℃,有符号数
  */
  float getInternalTempretureC(void);

  /**
  * @brief 写入环境温度数据用于外部温度补偿
  * @param temp 写入的环境温度数据, 单位℃, 分辨率0.1℃,有符号数
  */
  void setExternalTempretureC(float temp);

  /**
  * @brief 设置测量相关模式
  * @param mode 需要设置的测量相关模式, 下列模式相加为mode:
  *          eInternalTemp: 使用板载温度补偿功能, eExternalTemp: 使用外部温度补偿功能(需用户写入外部温度)
  *          eTempCompModeEn: 开启温度补偿功能, eTempCompModeDis: 关闭温度补偿功能
  *          eAutoMeasureModeEn: 自动测距, eAutoMeasureModeDis: 被动测距
  *          eMeasureRangeModeLong: 大量程测距(40 - 900cm), eMeasureRangeModeShort: 小量程测距(15-150cm)
  */
  void setMeasureMode(uint8_t mode);

  /**
  * @brief 被动测量模式下的触发测量函数
  * @n 在被动测量模式下, 调用一次此函数, 发送一次测距命令, 模块测量一次距离并将测量的距离值存入距离寄存器
  */
  void passiveMeasurementTRIG(void);

  /**
  * @brief 获取电源噪声等级, 0x00-0x0A对应噪声等级0-10
  * @n 该参数能够反映供电电源以及环境对传感器的影响程度。噪声等级越小, 传感器得到的距离值将更精准。
  */
  uint8_t getNoiseLevel(void);

  /**
  * @brief 测距灵敏度设置, 0x00-0x0A:灵敏度等级0-10
  * @param mode 用于设置传感器大量程段(40-900cm)的测距灵敏度, 该值越小, 灵敏度越高, 断电保存, 立即生效
  */
  void setMeasureSensitivity(uint8_t mode);


protected:
/***************** 寄存器读写接口 ******************************/

  /**
  * @brief 写寄存器函数, 设计为纯虚函数, 由派生类实现函数体
  * @param reg  寄存器地址 8bits
  *        pBuf 要写入数据的存放缓存
  *        size 要写入数据的长度
  */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size)=0;

  /**
  * @brief 读取寄存器函数, 设计为纯虚函数, 由派生类实现函数体
  * @param reg  寄存器地址 8bits
  *        pBuf 要读取数据的存放缓存
  *        size 要读取数据的长度
  * @return 返回读取的长度, 返回0表示读取失败
  */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size)=0;

private:
  // 定义私有变量
  uint8_t deviceInterface;
};


/***************** IIC接口的初始化和读写 ******************************/

class DFRobot_URM13_IIC:public DFRobot_URM13
{
public:
  /**
  * @brief 构造函数, 根据模块拨码开关状态, 配置传感器IIC通信地址
  * @param IIC_addr RotaryEncoder IIC communication address
  *        pWire Wire.h里定义了Wire对象, 因此使用&Wire就能够指向并使用Wire中的方法
  */
  DFRobot_URM13_IIC(uint8_t IIC_addr=URM13_DEFAULT_ADDR_IIC, TwoWire *pWire = &Wire, eInterfaceMode_t interfaceURM13=eI2cInterface);

  /**
  * @brief 子类初始化函数
  * @return 返回0表示初始化成功, 返回其他值表示初始化失败
  */
  virtual int begin(void);


protected:
  /**
  * @brief 通过IIC总线写入寄存器值
  * @param reg  寄存器地址 8bits
  *        pBuf 要写入数据的存放缓存
  *        size 要写入数据的长度
  */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size);

  /**
  * @brief 通过IIC总线读取寄存器值
  * @param reg  寄存器地址 8bits
  *        pBuf 要读取数据的存放缓存
  *        size 要读取数据的长度
  * @return 返回读取的长度, 返回0表示读取失败
  */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size);

private:
  TwoWire *_pWire;   // IIC通信方式的指针
  uint8_t _deviceAddr;   // IIC通信的设备地址
};


/***************** Modbus-RTU接口的初始化和读写 ******************************/

class DFRobot_URM13_RTU:public DFRobot_URM13
{
public:
  /*
    模块可设置波特率值
  */
  typedef enum
  {
    eBaudrate_2400 = 0x0001,
    eBaudrate_4800 = 0x0002,
    eBaudrate_9600 = 0x0003,
    eBaudrate_14400 = 0x0004,
    eBaudrate_19200 = 0x0005,
    eBaudrate_38400 = 0x0006,
    eBaudrate_57600 = 0x0007,
    eBaudrate_115200 = 0x0008,
  }eBaudrateMode_t;

  /*
    模块可设置校验位模式
  */
  typedef enum
  {
    eCheckBit_None = 0x0000<<8,
    eCheckBit_Even = 0x0001<<8,
    eCheckBit_Odd = 0x0002<<8,
  }eCheckBitMode_t;

  /*
    模块可设置停止位模式
  */
  typedef enum
  {
    eStopBit_0P5 = 0x0000,
    eStopBit_1 = 0x0001,
    eStopBit_1P5 = 0x0002,
    eStopBit_2 = 0x0003,
  }eStopBitMode_t;

  /**
  * @brief 构造函数
  * @param _serial 通信所需串口, 支持硬串口和软串口
  *        addr RS485通信的设备地址
  */
  DFRobot_URM13_RTU(Stream *_serial, uint16_t RTU_addr=URM13_DEFAULT_ADDR_RTU, eInterfaceMode_t interfaceURM13=eRtuInterface);

  /**
  * @brief UART接口模式下, 设置模块的波特率, 断电保存, 重启后生效
  * @param mode 要设置的波特率：
  *        eBaudrate_2400---2400, eBaudrate_4800---4800, eBaudrate_9600---9600, 
  *        eBaudrate_14400---14400, eBaudrate_19200---19200, eBaudrate_38400---38400, 
  *        eBaudrate_57600---57600, eBaudrate_115200---115200
  */
  void setBaudrateMode(eBaudrateMode_t mode);

  /**
  * @brief UART接口模式下, 设置模块的校验位和停止位
  * @param mode 要设置的模式：
  *        校验位：
  *              eCheckBit_None
  *              eCheckBit_Even
  *              eCheckBit_Odd
  *        停止位：
  *              eStopBit_0P5
  *              eStopBit_1
  *              eStopBit_1P5
  *              eStopBit_2
  */
  void setCheckbitStopbit(uint16_t mode);

protected:
  /**
  * @brief 写保持寄存器函数（多个寄存器数据的写入）
  * @param addr 要写入的设备地址 16bits
  *        reg  寄存器地址 16bits
  *        pBuf 要写入数据的存放缓存
  *        size 要写入数据的长度
  */
  virtual void writeReg(uint8_t reg, const void * pBuf, size_t size);

  /**
  * @brief 读取保持寄存器函数
  * @param addr 要读取的设备地址 16bits
  *        reg  寄存器地址 16bits
  *        pBuf 要读取数据的存放缓存
  *        size 要读取数据的长度
  * @return 返回读取的长度, 返回0表示读取失败
  */
  virtual size_t readReg(uint8_t reg, void * pBuf, size_t size);

private:
  DFRobot_RTU _DFRobot_RTU;   // RS485通信方式的对象
  uint16_t _deviceAddr;   // RTU通信的设备地址
};


/***************** TRIG接口的初始化和读写 ******************************/

// class DFRobot_URM13_TRIG:public DFRobot_URM13
// {
// public:
//   /**
//   * @brief 构造函数, 根据模块拨码开关状态, 配置传感器IIC通信地址
//   * @param trigPin TRIG接口模式下的测量触发引脚
//   *        echoPin TRIG接口模式下的数据引脚
//   *        interfaceURM13 接口模式, 共三种: eRtuInterface, eI2cInterface, eTrigInterface
//   */
//   DFRobot_URM13_TRIG(uint8_t trigPin=5, uint8_t echoPin=6, eInterfaceMode_t interfaceURM13=eI2cInterface);

//   /**
//   * @brief 子类初始化函数
//   * @return 返回0表示初始化成功, 返回其他值表示初始化失败
//   */
//   virtual int begin(void);

//   /**
//   * @brief TRIG接口模式下, 触发一次测量, 并读取此次距离测量值
//   * @param temp 用于补偿校准测量值的环境温度数据, 单位℃, 分辨率0.1℃,有符号数
//   */
//   uint16_t getDistanceCmTRIG(float temp=30.0);


// protected:
//   /**
//   * @brief 空函数, 满足父类纯虚函数设定
//   */
//   virtual void writeReg(uint8_t reg, const void * pBuf, size_t size){};

//   /**
//   * @brief 空函数, 满足父类纯虚函数设定
//   */
//   virtual size_t readReg(uint8_t reg, void * pBuf, size_t size){};

// private:
//   uint8_t _trigPin;
//   uint8_t _echoPin;

// };

#endif
