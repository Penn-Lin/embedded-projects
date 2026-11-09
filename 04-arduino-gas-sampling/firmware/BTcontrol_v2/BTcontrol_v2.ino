#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>  

SoftwareSerial bluetooth(2, 4); 

#define DHTPIN 6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define UV_LIGHT_TRIGGER 8  
#define RELAY_IN 10  

#define AFS01_ADDR 0x40  
#define OFFSET_FLOW 32000.0F  
#define SCALE_FLOW 140.0F  
const uint8_t CMD1000[2] = {0x10, 0x00};  

void setup() {
  Serial.begin(9600);
  
  bluetooth.begin(9600);
  delay(2000);
  bluetooth.println("蓝牙连接成功！");
  
  Wire.begin();
  
  dht.begin();
  pinMode(UV_LIGHT_TRIGGER, OUTPUT);
  pinMode(RELAY_IN, OUTPUT);
  

  digitalWrite(UV_LIGHT_TRIGGER, LOW);
  digitalWrite(RELAY_IN, LOW);  
}

void loop() {
  static unsigned long previousMillis = 0;
  const long interval = 2000;
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
  
    if (isnan(humidity) || isnan(temperature)) {
      bluetooth.println("温湿度读取失败！");
    } 
    else {
      bluetooth.print("T:");
      bluetooth.print(temperature);
      bluetooth.print("℃, H:");  
      bluetooth.print(humidity);
      bluetooth.println("%");
        Serial.print("蓝牙发送：T:");
  Serial.print(temperature);
  Serial.print("℃, H:"); 
  Serial.print(humidity);  
  Serial.println("%");   
    }


    float flowRate = readFlowValue();
    if (flowRate >= 0) {  
      bluetooth.print("Flow:");
      bluetooth.print(flowRate, 2);  
      bluetooth.println(" L/min");
    } else {
      bluetooth.println("流量读取失败！");
    }
  }
  
  if (bluetooth.available() > 0) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    
    Serial.print("收到指令: ");
    Serial.println(command);
    
    if (command == "UV ON") {  
      digitalWrite(UV_LIGHT_TRIGGER, HIGH);  
      bluetooth.println("紫光灯已开启");
      } 
    else if (command == "UV OFF") {  
      digitalWrite(UV_LIGHT_TRIGGER, LOW);   
      bluetooth.println("紫光灯已关闭");
      }
    else if (command == "PUMP ON") {
      digitalWrite(RELAY_IN, HIGH);  
      bluetooth.println("水泵已开启");
    }
    else if (command == "PUMP OFF") {
      digitalWrite(RELAY_IN, LOW);  
      bluetooth.println("水泵已关闭");
    }
    else if (command == "STATUS") {
      bluetooth.print("紫光灯:");
      bluetooth.print(digitalRead(UV_LIGHT_TRIGGER) ? "开启" : "关闭");
      bluetooth.print(",水泵:");
      bluetooth.println(digitalRead(RELAY_IN) ? "开启" : "关闭");
    }
    else {
      bluetooth.println("未知指令，请使用:");
      bluetooth.println("UV ON / UV OFF");
      bluetooth.println("PUMP ON / PUMP OFF");
      bluetooth.println("STATUS");
    }
  }
}

float readFlowValue() {
  uint8_t iicRecvBuf[3];
  
  if (!i2cWriteData(CMD1000, 2)) {
    return -1;
  }
  
  delay(1);  
  
  if (!i2cReadData(iicRecvBuf, 3)) {
    return -1;
  }
  if (calcCRC8(iicRecvBuf, 2) != iicRecvBuf[2]) {
    Serial.println("流量数据CRC校验失败");
    return -1;
  }
  
  uint16_t adValue = (iicRecvBuf[0] << 8) | iicRecvBuf[1];
  return ((float)adValue - OFFSET_FLOW) / SCALE_FLOW;
}

bool i2cWriteData(const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(AFS01_ADDR);
  Wire.write(data, len);
  return (Wire.endTransmission() == 0);
}

bool i2cReadData(uint8_t *data, uint8_t len) {
  uint8_t bytesRead = Wire.requestFrom(AFS01_ADDR, len);
  if (bytesRead != len) {
    return false;
  }
  
  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
  return true;
}

uint8_t calcCRC8(uint8_t *data, uint8_t num) {
  uint8_t bit, byte, crc = 0x00;
  for (byte = 0; byte < num; byte++) {
    crc ^= data[byte];
    for (bit = 8; bit > 0; bit--) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}