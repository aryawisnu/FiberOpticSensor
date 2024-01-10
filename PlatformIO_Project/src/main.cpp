#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "esp_adc_cal.h"


//gpio
#define ledTX 23
#define ledStats 19
#define tempSens 32
#define irLED 33
#define ptTX 34
#define ptRX 35

//ws2812
#define NUM_LEDS 1
Adafruit_NeoPixel ws2812b(NUM_LEDS, ledStats, NEO_GRB + NEO_KHZ800);

//ds18b20
OneWire oneWire(tempSens);
DallasTemperature temp(&oneWire);
uint8_t sensorRX[8] = {0x28, 0x7D, 0x99, 0x28, 0x00, 0x00, 0x00, 0xE4}; //ptRX
uint8_t sensorTX[8] = {0x28, 0x6F, 0x9B, 0x28, 0x00, 0x00, 0x00, 0x52}; //ptTX

//variabel
float tempRX, tempTX, voltRX, voltTX, dbmRX, dbmTX, loadKG;
byte data[13];

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
void debug();

//rtos
TaskHandle_t core0;
TaskHandle_t core1;

void setup() {
  pinMode(ledTX, OUTPUT);
  pinMode(irLED, OUTPUT);
  pinMode(ptTX, INPUT);
  pinMode(ptRX, INPUT);
  Serial.begin(9600);
  temp.begin();
  ws2812b.begin();
  ws2812b.setBrightness(255);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(irLED, 0);
  startLED();
 
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
  }
}

void taskCore1(void *pvParameters){
  for(;;){
    txLED(0);
    getTemperature();
    voltTX = readADC_Cal(analogRead(ptTX));
    voltRX = readADC_Cal(analogRead(ptRX));
    dbmRX = calculateOptPwr(ptRX);
    dbmTX = calculateOptPwr(ptTX);
    loadKG = 0;
    // float aFloat = 18.543;
    // int a = aFloat * 1000;
    // int aLSB = a % 100;
    // int aMSB = a / 100 % 1000;
    // Serial.println(aLSB);
    // debug();
    sendData();
    delay(500);
  }
}

void debug(){
    Serial.println("----------------------------------");
    Serial.print("Temperature RX : ");
    Serial.println(tempRX);
    Serial.print("Temperature TX : ");
    Serial.println(tempTX);
    Serial.println("----------------------------------");
    // Serial.println(voltageRead/1000.0, 4);
    Serial.println("----------------------------------");
    Serial.print("PT TX :");
    Serial.println(calculateOptPwr(ptTX), 4);
    delay(50);
    Serial.print("PT RX :");
    Serial.println(calculateOptPwr(ptRX), 4);
    Serial.print("PT TX Voltage :");
    Serial.println(readADC_Cal(analogRead(ptTX)) / 1000.0, 4);
    Serial.print("PT RX Voltage :");
    Serial.println(readADC_Cal(analogRead(ptRX)) / 1000.0, 4);
    Serial.println("----------------------------------");
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
  analogWrite(ledTX, pwm);
}

uint32_t readADC_Cal(int ADC_Raw){
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

float calculateOptPwr(uint8_t sensorPin){
  float voltageRead = (readADC_Cal(analogRead(sensorPin))) / 1000.0;
  float ampere = voltageRead / 100000.0; //50100.0
  float microamps = ampere * 1000000;
  // float lux = mapFloat(microamps, 5.5, 59, 10, 100);
  float opticalPower = microamps / (0.62*1000000);
  float dBm = (10 * log10(opticalPower)) + 30;
  voltageRead = 0;
  return dBm;
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
  uint16_t dbmRXlsb, dbmRXmsb, dbmTXlsb, dbmTXmsb, voltRXlsb, voltRXmsb, voltTXlsb, voltTXmsb;
  data[0] = 0x0A; //start byte
  data[1] = dbmRX *= (-1000) / 100 % 1000;
  dbmRXlsb = dbmRX * 100;
  data[2] = dbmRXlsb % 100;
  data[3] = dbmTX *= (-1000) / 100 % 1000;
  dbmTXlsb = dbmTX * 100;
  data[4] = dbmTXlsb % 100;
  data[5] = loadKG * 10;
  data[6] = voltRX /= 10 % 1000;
  voltRXlsb = voltRX * 100;
  data[7] = voltRXlsb % 100;
  data[8] = voltTX /= 10 % 1000;
  voltTXlsb = voltTX * 100;
  data[9] = voltTXlsb % 100;
  data[10] = tempRX;
  data[11] = tempTX;
  data[12] = 0;
  data[13] = 0x0A; //stop byte
  Serial.write(data, sizeof(data));
}



