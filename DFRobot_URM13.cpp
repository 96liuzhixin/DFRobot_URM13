/*!
 * @file DFRobot_URM13.CPP
 * @brief DFRobot_URM13.cpp Initialize the IIC,
 * @n 获取URM13基本信息、测量距离和板载温度, 选择传感器通信接口, 设置传感器测量参数
 * 
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @Version V1.0.0
 * @date 2021-09-15
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_URM13
 */
#include "DFRobot_URM13.h"

DFRobot_URM13::DFRobot_URM13(eInterfaceMode_t interfaceURM13)
{
  deviceInterface = interfaceURM13;
}


int DFRobot_URM13::begin(void)
{
  DBG(deviceInterface);
  if (deviceInterface == eRtuInterface){
    uint8_t idBuf[2];
    if(0 == readReg(URM13_PID_REG_RTU, idBuf, sizeof(idBuf))){   // Judge whether the data bus is successful
      DBG("ERR_DATA_BUS");
      return ERR_DATA_BUS;
    }

    DBG("real sensor id="); DBG( (uint16_t)idBuf[0] << 8 | (uint16_t)idBuf[1], HEX );
    if( 0x0003 != ( (uint16_t)idBuf[0] << 8 | (uint16_t)idBuf[1] ) ){   // Judge whether the chip version matches
      DBG("ERR_IC_VERSION");
      return ERR_IC_VERSION;
    }

  }else if (deviceInterface == eI2cInterface){
    uint8_t id;
    if(0 == readReg(URM13_PID_REG_IIC, &id, sizeof(id))){   // Judge whether the data bus is successful
      DBG("ERR_DATA_BUS");
      return ERR_DATA_BUS;
    }

    DBG("real sensor id="); DBG(id, HEX );
    if( 0x02 != id ){   // Judge whether the chip version matches
      DBG("ERR_IC_VERSION");
      return ERR_IC_VERSION;
    }

  }else{

  }
  delay(200);

  DBG("begin ok!");

  return NO_ERR;
}

void DFRobot_URM13::readBasicInfo(uint8_t* pbuf)
{
  if (deviceInterface == eRtuInterface){
    readReg(URM13_PID_REG_RTU, pbuf, 10);   // rtu基本信息长度为10个字节

  }else if (deviceInterface == eI2cInterface){
    readReg(URM13_ADDR_REG_IIC, pbuf, 3);   // IIC基本信息长度为3个字节

  }else{

  }
}

void DFRobot_URM13::setADDR(uint8_t addr)
{
  if (deviceInterface == eRtuInterface){
    if((0x01 <= addr) && (0xF7 >= addr)){
      uint8_t buf[2] = {0};
      buf[1] = addr;
      writeReg(URM13_ADDR_REG_RTU, buf, sizeof(buf));
    }

  }else if (deviceInterface == eI2cInterface){
    if((0x01 <= addr) && (0x7F >= addr)){
      writeReg(URM13_ADDR_REG_IIC, &addr, sizeof(addr));
    }

  }else{

  }
}

uint16_t DFRobot_URM13::getDistanceCm(void)
{
  uint8_t buf[2] = {0};
  if (deviceInterface == eRtuInterface){
    readReg(URM13_DISTANCE_REG_RTU, buf, sizeof(buf));

  }else if (deviceInterface == eI2cInterface){
    readReg(URM13_DISTANCE_MSB_REG_IIC, buf, sizeof(buf));

  }else{

  }

  return (buf[0] << 8) | buf[1];
}

float DFRobot_URM13::getInternalTempretureC(void)
{
  uint8_t buf[2] = {0};
  if (deviceInterface == eRtuInterface){
    readReg(URM13_INTERNAL_TEMP_REG_RTU, buf, sizeof(buf));

  }else if (deviceInterface == eI2cInterface){
    readReg(URM13_INTERNAL_TEMP_MSB_REG_IIC, buf, sizeof(buf));

  }else{

  }
  // DBG(buf[0], HEX);
  // DBG(buf[1], HEX);

  return (float)(int16_t)((buf[0] << 8) | buf[1]) / 10;
}

void DFRobot_URM13::setExternalTempretureC(float temp)
{
  int16_t temperature = (int16_t)(temp * 10);   // 写入的环境温度数据, 单位℃, 分辨率0.1℃,有符号数
  uint8_t buf[2] = {0};
  // DBG(temp);
  // DBG(temperature, HEX);
  buf[0] = (uint8_t)((temperature & 0xFF00) >> 8);
  buf[1] = (uint8_t)(temperature & 0x00FF);
  // DBG(buf[0], HEX);
  // DBG(buf[1], HEX);
  // DBG((float)(int16_t)((buf[0] << 8) | buf[1]) / 10);

  if (deviceInterface == eRtuInterface){
    writeReg(URM13_EXTERNAL_TEMP_REG_RTU, buf, sizeof(buf));

  }else if (deviceInterface == eI2cInterface){
    writeReg(URM13_EXTERNAL_TEMP_MSB_REG_IIC, buf, sizeof(buf));

  }else{

  }
}

void DFRobot_URM13::setMeasureMode(uint8_t mode)
{
  if (deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    // readReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));
    // DBG(buf[1]);
    buf[1] = mode;
    writeReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));

  }else if (deviceInterface == eI2cInterface){
    // uint8_t data = 0;
    // readReg(URM13_CONFIG_REG_IIC, &data, sizeof(data));
    // DBG(data);
    writeReg(URM13_CONFIG_REG_IIC, &mode, sizeof(mode));
  }else{

  }
}

void DFRobot_URM13::passiveMeasurementTRIG(void)
{
  if (deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    readReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));

    // 被动模式下, 向该位写入1, 传感器将完成一次测距, 测距完成后(约100ms)可从距离寄存器读出距离值, 自动测距模式下该位保留。该位置1后将自动清0
    buf[1] |= (1 << 3);
    writeReg(URM13_CONFIG_REG_RTU, buf, sizeof(buf));
    delay(300);

  }else if (deviceInterface == eI2cInterface){
    uint8_t mode = 0x01;
    writeReg(URM13_COMMAND_REG_IIC, &mode, sizeof(mode));

  }else{

  }
}

uint8_t DFRobot_URM13::getNoiseLevel(void)
{
  uint8_t mode = 0;
  if (deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    readReg(URM13_NOISE_REG_RTU, buf, sizeof(buf));
    mode = buf[1];

  }else if (deviceInterface == eI2cInterface){
    readReg(URM13_NOISE_REG_IIC, &mode, sizeof(mode));

  }else{

  }

  return mode;
}

void DFRobot_URM13::setMeasureSensitivity(uint8_t mode)
{
  if (deviceInterface == eRtuInterface){
    uint8_t buf[2] = {0};
    buf[1] = mode;
    writeReg(URM13_SENSITIVITY_REG_RTU, buf, sizeof(buf));

  }else if (deviceInterface == eI2cInterface){
    writeReg(URM13_SENSITIVITY_REG_IIC, &mode, sizeof(mode));

  }else{

  }
}


/************ Initialization of IIC interfaces reading and writing ***********/

DFRobot_URM13_IIC::DFRobot_URM13_IIC(uint8_t IIC_addr, TwoWire *pWire, eInterfaceMode_t interfaceURM13)
  :DFRobot_URM13(interfaceURM13)
{
  _deviceAddr = IIC_addr;
  _pWire = pWire;
}

int DFRobot_URM13_IIC::begin(void)
{
  _pWire->begin();   // Wire.h（IIC）library function initialize wire library
  delay(50);

  return DFRobot_URM13::begin();   // Use the initialization function of the parent class
}

void DFRobot_URM13_IIC::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(pBuf == NULL){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);

  for(size_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }

  _pWire->endTransmission();
}

size_t DFRobot_URM13_IIC::readReg(uint8_t reg, void* pBuf, size_t size)
{
  size_t count = 0;
  if(NULL == pBuf){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t*)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire -> write(reg);
  if(0 != _pWire->endTransmission()){
    // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
    DBG("endTransmission ERROR!!");
  }else{
    // Master device requests size bytes from slave device, which can be accepted by master device with read() or available()
    _pWire->requestFrom( _deviceAddr, (uint8_t)size );

    while (_pWire->available()){
      _pBuf[count++] = _pWire->read();   // Use read() to receive and put into buf
      // DBG(_pBuf[count-1], HEX);
    }
  }

  return count;
}


/************ Initialization of modbus-RTU interfaces reading and writing ***********/

DFRobot_URM13_RTU::DFRobot_URM13_RTU(Stream *_serial, uint16_t RTU_addr, eInterfaceMode_t interfaceURM13)
  : _DFRobot_RTU(_serial), DFRobot_URM13(interfaceURM13)
{
  _deviceAddr = RTU_addr;
  _DFRobot_RTU.setTimeoutTimeMs(500);
}

void DFRobot_URM13_RTU::setBaudrateMode(eBaudrateMode_t mode)
{
  uint8_t buf[2] = {0};
  buf[0] = (uint8_t)((mode & 0xFF00) >> 8);
  buf[1] = (uint8_t)(mode & 0x00FF);

  writeReg(URM13_BAUDRATE_REG_RTU, buf, sizeof(buf));
  delay(200);
  // switch(mode){
  // case 1:
  //   _ModbusRTUClient.begin(2400);break;
  // case 2:
  //   _ModbusRTUClient.begin(4800);break;
  // case 3:
  //   _ModbusRTUClient.begin(9600);break;
  // case 4:
  //   _ModbusRTUClient.begin(14400);break;
  // case 5:
  //   _ModbusRTUClient.begin(19200);break;
  // case 6:
  //   _ModbusRTUClient.begin(38400);break;
  // case 7:
  //   _ModbusRTUClient.begin(57600);break;
  // case 8:
  //   _ModbusRTUClient.begin(115200);break;
  // default:
  //   _ModbusRTUClient.begin(115200);break;
  // }
  // delay(200);
}

void DFRobot_URM13_RTU::setCheckbitStopbit(uint16_t mode)
{
  uint8_t buf[2] = {0};
  buf[0] = (uint8_t)((mode & 0xFF00) >> 8);
  buf[1] = (uint8_t)(mode & 0x00FF);
  writeReg(URM13_CHECKBIT_STOPBIT_REG_RTU, buf, sizeof(buf));
}

void DFRobot_URM13_RTU::writeReg(uint8_t reg, const void * pBuf, size_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }

  // uint16_t *dBuf = (uint16_t *)pBuf;
  // uint8_t buf[size*2];
  // for(int i = 0; i < size; i++){
  //   buf[i] = dBuf[i+1];
  //   buf[i+1] = dBuf[i];
  // }

  uint8_t ret = _DFRobot_RTU.writeHoldingRegister(_deviceAddr, (uint16_t)reg, (uint8_t *)pBuf, (uint16_t)size);
  if (ret != 0){
    DBG(ret);
  }
}

size_t DFRobot_URM13_RTU::readReg(uint8_t reg, void * pBuf, size_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }

  // _DFRobot_RTU.setTimeoutTimeMs(500);
  // uint8_t buf[size*2];
  uint8_t ret = _DFRobot_RTU.readHoldingRegister(_deviceAddr, (uint16_t)reg, pBuf, (uint16_t)size);
  if (ret != 0){
    DBG(ret);
    return 0;
  }
  // for (int i = 0; i < size; i++){
  //    pBuf[i] = (buf[i*2] << 8)+ buf[i*2 + 1];
  // }

  return size;
}


/******************** Initialization of TRIG interfaces ***********************/

// DFRobot_URM13_TRIG::DFRobot_URM13_TRIG(uint8_t trigPin, uint8_t echoPin, eInterfaceMode_t interfaceURM13)
//   : DFRobot_URM13(interfaceURM13)
// {
//   _trigPin = trigPin;
//   _echoPin = echoPin;
// }

// int DFRobot_URM13_TRIG::begin(void)
// {
//   pinMode(_trigPin,OUTPUT);
//   digitalWrite(_trigPin,LOW);
//   pinMode(_echoPin,INPUT);
//   delay(50);

//   return DFRobot_URM13::begin();   // Use the initialization function of the parent class
// }

// uint16_t DFRobot_URM13_TRIG::getDistanceCmTRIG(float temp)
// {
//   uint16_t  pulseWidthUs;
//   digitalWrite(_trigPin,HIGH);   //Set the tirgPin High
//   delayMicroseconds(50);   //Delay of 50 microseconds
//   digitalWrite(_trigPin,LOW);   //Set the tirgPin Low

//   /*
//   Measure echo high level time, the output high level time represents the ultrasonic flight time (unit: us)
//   The distance can be calculated according to the flight time of ultrasonic wave, 
//   and the ultrasonic sound speed can be compensated according to the actual ambient temperature
//   */
//   pulseWidthUs = pulseIn(_echoPin,HIGH);
//   delayMicroseconds(50);   //Delay of 50 microseconds
//   return (uint16_t)(pulseWidthUs * VELOCITY_TEMP(temp) / 2.0);
// }

