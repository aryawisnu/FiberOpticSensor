#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// #include "esp_adc_cal.h"
#include "EasyNextionLibrary.h"
// #include <driver/adc.h>


//gpio
#define ledTX 23
#define ledStats 19
#define tempSens 32
#define irLED 33
#define ptTX 35
#define ptRX 34

//nextion
EasyNex display(Serial2);

//ws2812
#define NUM_LEDS 1
Adafruit_NeoPixel ws2812b(NUM_LEDS, ledStats, NEO_GRB + NEO_KHZ800);

//ds18b20
OneWire oneWire(tempSens);
DallasTemperature temp(&oneWire);
uint8_t sensorRX[8] = {0x28, 0x7D, 0x99, 0x28, 0x00, 0x00, 0x00, 0xE4}; //ptRX
uint8_t sensorTX[8] = {0x28, 0x6F, 0x9B, 0x28, 0x00, 0x00, 0x00, 0x52}; //ptTX

//variabel
float tempRX, tempTX, voltTX;
float dbmRX = -50.0;
float dbmTX = -50.0;
float loadKG = 0;
double voltRX;
byte data[13];
uint8_t ledBrightness = 0;
#define ptTXrVal 100000.0
#define ptRXrVal 98000.0
unsigned long millis1, millis2, millis3, millis4, millis5, millis6, millis7, millis8;
float dbmrxx, zeroVal;

//fungsi
void taskCore0(void *pvParameters);
void taskCore1(void *pvParameters);
void getTemperature();
void txLED(uint8_t pwm);
float calculateOptPwr(uint8_t sensorPin);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t readADC_Cal(int ADC_Raw);
void startLED();
void sendData();
void readData();
void debug();
double ReadVoltage(uint8_t pin);
float calculateKg(float dbm, float zeroValue);
float precision( float f, int places);
void recordData();

//rtos
TaskHandle_t core0;
TaskHandle_t core1;

void setup() {
  delay(3000);
  pinMode(ledTX, OUTPUT);
  // pinMode(irLED, OUTPUT);
  // pinMode(ptTX, INPUT);
  // pinMode(ptRX, INPUT);
  Serial.begin(9600);
  Serial.setTimeout(20);
  Serial1.begin(9600, SERIAL_8N1, 18, 17); //rx tx
  display.begin(9600);
  temp.begin();
  ws2812b.begin();
  ws2812b.setBrightness(255);
  // ledcSetup(0, 5000, 8);
  // ledcAttachPin(irLED, 0);
  startLED();
  getTemperature();
  display.writeStr("t2.txt", String(tempTX));
  // adc1_config_width(ADC_WIDTH_12Bit);
  // adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
 
  xTaskCreatePinnedToCore(
    taskCore0, 
    "core0", 
    10000,
    NULL,
    1,
    &core0,
    0 
  );

  xTaskCreatePinnedToCore(
    taskCore1, 
    "core1", 
    10000,
    NULL,
    1,
    &core1,
    1 
  );
}

void loop() {
  
}

void taskCore0(void *pvParameters){
  for(;;){
    ws2812b.setPixelColor(0, ws2812b.Color(0,255,0));
    ws2812b.show();
    delay(50);
    ws2812b.setPixelColor(0, ws2812b.Color(0,0,0));
    ws2812b.show();
    delay(1000);
    // getTemperature();
    // delay(100);
  }
}

void taskCore1(void *pvParameters){
  for(;;){
    // txLED(90);
    analogWrite(irLED, ledBrightness);
    // analogWrite(irLED, 90);
    // txLED(ledBrightness);
    millis2 = millis();
    if(millis2 - millis1 >= 60000){
      getTemperature();
      display.writeStr("t2.txt", String(tempRX));
      millis1 = millis2;
    }
    dbmRX = calculateOptPwr(ptRX);
    dbmrxx = precision(dbmRX, 1);
    loadKG = calculateKg(dbmrxx, zeroVal);
    millis8 = millis();
    if(millis8 - millis7 >= 1000){
      display.writeStr("t1.txt", String(dbmrxx));
      display.writeStr("t0.txt", String(loadKG));
      display.writeStr("t2.txt", String(tempRX));
      millis7 = millis8;
    }
    // analogReadResolution(12);
    // analogSetAttenuation(ADC_11db);
    // analogSetPinAttenuation(ptRX, ADC_11db);
    // int val = analogRead(ptRX);
    // double voltage = ReadVoltage(val);
    // Serial1.print("RAW : ");
    // Serial1.println(val);
    // Serial1.print("Voltage : ");
    // Serial1.println(voltage);
    // Serial1.println(readADC_Cal(val));
    readData();
    sendData();
    recordData();
    delay(80);
  }
}

float precision( float f, int places ){
    float n = std::pow(10.0f, places ) ;
    return std::round(f * n) / n ;
}

double ReadVoltage(uint8_t pin){ //byte pin
  double reading = analogRead(pin);
  // Serial1.print("ADC : ");
  // Serial1.println(reading, 5);
  if(reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
}

void debug(){
    Serial1.println("----------------------------------");
    Serial1.print("Temperature RX : ");
    Serial1.println(tempRX);
    Serial1.print("Temperature TX : ");
    Serial1.println(tempTX);
    Serial1.println("----------------------------------");
    // Serial.println(voltageRead/1000.0, 4);
    Serial1.println("----------------------------------");
    Serial1.print("PT TX :");
    // Serial.println(calculateOptPwr(ptRX, ptRXrVal), 4);
    delay(50);
    Serial.print("PT RX :");
    // Serial.println(calculateOptPwr(ptRX, ptTXrVal), 4);
    Serial1.print("PT TX Voltage :");
    Serial1.println(readADC_Cal(analogRead(ptTX)) / 1000.0, 4);
    Serial1.print("PT RX Voltage :");
    Serial1.println(readADC_Cal(analogRead(ptRX)) / 1000.0, 4);
    Serial1.println("----------------------------------");
}

void getTemperature(){
  temp.setResolution(sensorTX, 10);
  temp.setResolution(sensorRX, 10);
  temp.requestTemperatures();
  tempTX = temp.getTempC(sensorTX);
  tempRX = temp.getTempC(sensorRX);
}

void txLED(uint8_t pwm){
  ledcWrite(0, pwm);
  // analogWrite(ledTX, pwm);
}

// uint32_t readADC_Cal(int ADC_Raw){
//   esp_adc_cal_characteristics_t adc_chars;
//   esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
//   return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
// }

float calculateOptPwr(uint8_t sensorPin){
  // float voltageRead = (readADC_Cal(analogRead(sensorPin))) / 1000.0;
  double offsetVoltage = 0.641;
  double offsetDBM = -7.611;
  double voltageRead = ReadVoltage(sensorPin) - offsetVoltage;
  if(voltageRead <= 0){voltageRead = 0;}
  if(sensorPin == ptRX){voltRX = voltageRead;}
  else{voltTX = voltageRead;}
  double ampere = voltageRead / 100000.0; //50100.0
  double microamps = ampere * 1000000;
  // float lux = mapFloat(microamps, 5.5, 59, 10, 100);
  double opticalPower = microamps / (0.62*1000000);
  float dBm = (10 * log10(opticalPower)) + 30;
  float realdBm;
  if(dBm <= -50.0){
    realdBm = dbmRX;
    millis4 = millis();
    if(millis4 - millis3 >= 1000){
      realdBm = -50.0;
    }
  }
  else{
    realdBm = dBm + offsetDBM;
    millis3 = millis4;
  }
  // voltageRead = 0;
  return realdBm;
}

float xOld, yOld, yHold;

float calculateKg(float dbm, float zeroValue){
  float x, x0, x0cal, x1, y, y0, y1, dev;
  x = dbm;
  x0 = zeroValue; //calibrate on reset
  x0cal = -17.60; //17.60
  y0 = 0; //zero value of Kg
  x1 = -18.30; //calibrate known value of dBm  -18.30
  y1 = 23.39; //calibrate known value of Kg
  dev = x1 - x0cal;
  x1 = x0 + dev; //shift the x1 value of interpolation
  y = y0 + (x-x0) * ((y1-y0)/(x1-x0)); //linear interpolation
  if(x != xOld){
    float xxx = x - xOld;
    // Serial1.print("xxx : ");
    // Serial1.println(xxx);
    if((xxx > -0.2f) && (xxx < 0) && (y < 1)){
      // Serial1.print("update zero :");
      // Serial1.println(x);
      zeroVal = x;
    }
    xOld = x;
  }
  if(y != yOld){
    millis6 = millis();
    if(millis6 - millis5 >= 60000){
      yHold = y;
    }
    millis5 = millis6;
  }
  if(x > x0 || x <= -50){return 0;}
  return y;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void startLED(){
  uint8_t r, g, b;
  ws2812b.setBrightness(50);
  for(int i=0; i<20; i++){
      g = random(255);
      b = random(150);
      ws2812b.setPixelColor(0, ws2812b.Color(0, g, b));
      ws2812b.show();
      delay(50);
  }
}

void sendData(){
  uint16_t dbmRXint ,dbmTXint, voltRXint, voltTXint;
  data[0] = 0x0A; //start byte
  // dbmRXint = (dbmRX * (-100));
  dbmRXint = (dbmrxx * (-100));
  data[1] = dbmRXint >> 8;
  data[2] = dbmRXint & 0xFF;
  dbmTXint = (dbmTX * (-100));
  data[3] = dbmTXint >> 8;
  data[4] = dbmTXint & 0xFF;
  voltRXint = voltRX;
  data[5] = voltRXint >> 8;
  data[6] = voltRXint & 0xFF;
  voltTXint = voltTX;
  data[7] = voltTXint >> 8;
  data[8] = voltTXint & 0xFF;
  data[9] = loadKG * 10;
  data[10] = tempRX;
  data[11] = tempTX;
  data[12] = ledBrightness;
  data[13] = 0x0A; //stop byte
  Serial.write(data, sizeof(data));
}

void readData(){
  int tmp = Serial.parseInt();
  if(tmp > 0){
  ledBrightness = tmp;
  dbmTX = calculateOptPwr(ptTX);
  delay(100);
  zeroVal = dbmrxx;
  }
  else if(tmp == 1){
    ledBrightness = 0;
  }

  if(ledBrightness > 1){
    display.writeNum("p1.pic", 41);
  }
  else{
    display.writeNum("p1.pic", 42);
  }
}

void recordData(){
  // String data = String(ledBrightness) + "," + String(dbmRX) + "," + String(dbmrxx) + "," + String(dbmTX) + "," + String(tempRX) + "," + String(tempTX) + "," + String(voltRX) + "," + String(voltTX);
  Serial1.print(ledBrightness);
  Serial1.print(";");
  Serial1.print(dbmRX);
  Serial1.print(";");
  Serial1.print(dbmrxx);
  Serial1.print(";");
  Serial1.print(dbmTX);
  Serial1.print(";");
  Serial1.print(tempRX);
  Serial1.print(";");
  Serial1.print(tempTX);
  Serial1.print(";");
  Serial1.print(voltRX);
  Serial1.print(";");
  Serial1.println(voltTX);
}