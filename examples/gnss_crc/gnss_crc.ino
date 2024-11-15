 /*!
  * @file  getGNSS.ino
  * @brief Get gnss simple data
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V0.5.0
  * @date 2023-04-23
  * @url https://github.com/DFRobot/DFRobot_RTK_LoRa
  */

#include "DFRobot_RTK_LoRa.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
  DFRobot_RTK_LoRa_I2C rtk(&Wire ,DEVICE_ADDR);
#else
/* -----------------------------------------------------------------------------------------------------
 * |  Sensor  | Connect line | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 * |   VCC    |=============>|        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 * |   GND    |=============>|        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 * |   RX     |=============>|     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 * |   TX     |=============>|     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed */
  #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial mySerial(4, 5);
    DFRobot_RTK_LoRa_UART rtk(&mySerial, 57600);
  #elif defined(ESP32)
    DFRobot_RTK_LoRa_UART rtk(&Serial1, 115200 ,/*rx*/D2 ,/*tx*/D3);
  #else
    DFRobot_RTK_LoRa_UART rtk(&Serial1, 115200);
  #endif
#endif

// 校验 NMEA 0183 数据是否有效
bool validateNMEA(String nmea) {
  int checksum = 0;
  
  // 查找 $ 和 * 的位置
  int start = nmea.indexOf('$');
  int end = nmea.indexOf('*');

  // 检查数据格式
  if (start == -1 || end == -1 || end <= start + 1) {
    return false;
  }

  // 计算校验和
  for (int i = start + 1; i < end; i++) {
    checksum ^= nmea[i];
  }

  // 提取提供的校验和并转换为大写的两位十六进制
  String provided_checksum = nmea.substring(end + 1, end + 3);
  String computed_checksum = String(checksum, HEX);
  computed_checksum.toUpperCase();
  
  if (computed_checksum.length() < 2) {
    computed_checksum = "0" + computed_checksum;
  }

  // 返回校验结果
  return (computed_checksum == provided_checksum);
}

static sTim_t oldtime;
static sTim_t old[100];
static uint8_t timerFlushError = 0; 
static uint8_t nmeaError = 0;
void setup()
{
  Serial.begin(115200);
  Serial.println("setup !");
  while(!rtk.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");
  rtk.setModule(eMoudleLora);
  while(rtk.getModule() != eMoudleLora){
    Serial.println("Module type is not lora!  please wait!");
    delay(1000);
  }
  oldtime = rtk.getUTC();
  delay(1000);
}

void loop()
{
  
  bool state = rtk.getDataFlush();
  if(state == true)
  {
  #if 1
    sTim_t utc = rtk.getUTC();
    sTim_t date = rtk.getDate();
    sLonLat_t lat = rtk.getLat();
    sLonLat_t lon = rtk.getLon();
    double high = rtk.getAlt();
    uint8_t starUserd = rtk.getNumSatUsed();
    double hdop = rtk.getHdop();
    double sep = rtk.getSep();
    uint8_t mode = rtk.getQuality();
    uint16_t siteID = rtk.getSiteID();
    double diftime = rtk.getDifTime();
    Serial.println("");
    Serial.print(date.year);
    Serial.print("/");
    Serial.print(date.month);
    Serial.print("/");
    Serial.print(date.date);
    Serial.print("/");
    Serial.print(utc.hour);
    Serial.print(":");
    Serial.print(utc.minute);
    Serial.print(":");
    Serial.print(utc.second);
    Serial.println();
    #endif
    // 简单对比秒数不等于

    if(oldtime.second == 59 && utc.second == 0){
      // 数据正确
      oldtime.second = utc.second;
    }else if((utc.second - oldtime.second) == 1){ // 增加一秒
      // 数据正确
      oldtime.second = utc.second;
    }else{
      Serial.print("utc = ");
      Serial.print(utc.second);
      Serial.print(" old = ");
      Serial.println(oldtime.second);
      oldtime.second = utc.second;
      old[timerFlushError] = utc;
      timerFlushError++;
    }
    
    if(timerFlushError){
      Serial.print("time error count = ");
      Serial.println(timerFlushError);
      for(uint8_t i = 0; i < timerFlushError; i++){
        Serial.print("time error ");
        Serial.print(i);
        Serial.print("/");
        Serial.print(old[i].hour);
        Serial.print(":");
        Serial.print(old[i].minute);
        Serial.print(":");
        Serial.println(old[i].second);
      }
    }

  #if 0
    Serial.println((char)lat.latDirection);
    Serial.println((char)lon.lonDirection);
    // Serial.print("lat DDMM.MMMMM = ");
    // Serial.println(lat.latitude, 5);
    // Serial.print(" lon DDDMM.MMMMM = ");
    // Serial.println(lon.lonitude, 5);
    Serial.print("lat degree = ");
    Serial.println(lat.latitudeDegree,6);
    Serial.print("lon degree = ");
    Serial.println(lon.lonitudeDegree,6);
    Serial.print("star userd = ");
    Serial.println(starUserd);
    Serial.print("alt high = ");
    Serial.println(high);
    Serial.print("sep  = ");
    Serial.println(sep);
    Serial.print("hdop = ");
    Serial.println(hdop);
    Serial.print("message mode  = ");
    Serial.println(mode);
    Serial.print("siteID = ");
    Serial.println(siteID);
    Serial.print("diftime = ");
    Serial.println(diftime);
  #endif

    String temp0 = rtk.getGnssMessage(eGGA);
    String temp1 = rtk.getGnssMessage(eRMC);
    String temp2 = rtk.getGnssMessage(eGLL);
    String temp3 = rtk.getGnssMessage(eVTG);

    if (validateNMEA(temp0)) {
      Serial.println(temp0);
    }else{
      nmeaError++;
    }

    if (validateNMEA(temp1)) {
      Serial.println(temp1);
    }else{
      nmeaError++;
    }
    
    if (validateNMEA(temp2)) {
      Serial.println(temp2);
    }else{
      nmeaError++;
    }
    
    if (validateNMEA(temp3)) {
      Serial.println(temp3);
    }else{
      nmeaError++;
    }

    if(nmeaError){
      Serial.print("nmeaError count = ");
      Serial.println(nmeaError);
    }
  }
}