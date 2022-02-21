#include <Arduino.h>
#include <Wire.h> 
#include <../lib/LCD/MyLiquidCrystalI2C.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "RTClib.h"
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <esp32ModbusRTU.h>
#include <algorithm>
#include <WiFi.h>
#include "secrets.h"
#include "ESPAsyncWebServer.h"

#define DEBOUNCE_TIME 50
#define EEPROM_SIZE 512

#define I2C_SDA2 32
#define I2C_SCL2 33

#define RELAY 16
#define KBD1 4
#define KBD2 0
#define KBD3 2
#define KBD4 15

#define RX 26
#define TX 25
#define RSE 27
#define DEVICE_ID 1

#define CURRENT_PRODUCTION 0
#define CURRENT_TIME 1
#define CURRENT_DATE 2
#define CURRENT_TEMP 3
#define MIN_TEMP 4
#define MAX_TEMP 5
#define MIN_PRODUCTION 6
#define EXPECTED_HOUR 7
#define HOUR_SETTING 8
#define MINUTE_SETTING 9 
#define DAY_SETTING 10
#define MONTH_SETTING 11
#define YEAR_SETTING 12

#define CURRENT_PRODUCTION_READ 0
#define TODAY_PRODUCTION_READ 1
#define TODAY_GENERATION_READ 2
#define A_PHASE_VOLTAGE_READ 3
#define B_PHASE_VOLTAGE_READ 4
#define C_PHASE_VOLTAGE_READ 5
#define TOTAL_PRODUCTION_HIGH_READ 6
#define TOTAL_PRODUCTION_LOW_READ 7


LiquidCrystal_I2C lcd(0x27,20,4);

RTC_PCF8563 rtc;

HardwareSerial SerialModbus(2);
esp32ModbusRTU MODBUS_INTERFACE(&SerialModbus, RSE);

char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// GPIO where the DS18B20 is connected to
const int oneWireBus = 17;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

int pageNumber = 0;
double currentProduction = 0.0;
double currentDisplayProduction = 0.0;
volatile double waterTemp = 0.0;
volatile double oldWaterTemp = 0.0;
int maxWaterTemp = 80;
int minWaterTemp = 50;
int minPrduction = 1;
int hourTempExpected = 18;
int lastReadTime = 0;

double todayProductionValue = 0.0;
double todayGenerationTime = 0.0;

volatile double a_phase_voltage = 0.0;
volatile double b_phase_voltage = 0.0;
volatile double c_phase_voltage = 0.0;

volatile double total_production_high = 0.0;
volatile double total_production_low = 0.0;
volatile double total_production = 0.0;
/*
* defines which value is read from inverter
* 0 - current production
* 1 - today production
* 2 - today generation time
* 3 - A-phase voltage
* 4 - B-phase voltage
* 5 - C-phase voltage
* 6 - total production high
* 7 - total production low
*/
int currentReadingValue = 0;

bool valDownMarker = false;
bool valUpMarker = false;

bool pageUpMarker = false;
bool pageDownMarker = false;

bool heaterState = false;
bool HEAT_MAX_FLAG = false;
bool HEAT_MIN_FLAG = true;

DateTime oldMeasureTime;
DateTime oldMeasureTimeMODBUS;
DateTime oldMeasureTimeProductionUpdate;
DateTime oldMeasureTimeProductionUpdateDisplay;

char pageTitles[13][17] = {
  "Akt. produkcja",
  "Akt. godzina",
  "Akt. data",
  "Temp. wody",
  "Zad. temp. wody",
  "Tol. temp. wody",
  "Min. produkcja",
  "Godz. oczekiwana",
  "Ustaw. godziny",
  "Ustaw. minuty",
  "Ustaw. dnia",
  "Ustaw. miesiaca",
  "Ustaw. roku",
};

uint8_t valuesTab[13] = {0,0,0,0,50,80,1,18,50,50,0,0};

TwoWire I2C2 = TwoWire(1); //I2C2 bus

const char *ssid = SSID;
const char *password = PASSWORD;

AsyncWebServer server(80);

void IRAM_ATTR SETT_UP_ISR() { 
  pageUpMarker = true;
}

void IRAM_ATTR SETT_DOWN_ISR() {
  pageDownMarker = true;
}

void IRAM_ATTR VAL_UP_ISR() { 
  if (pageNumber > 3) {
    valUpMarker = true;
  }
}

void IRAM_ATTR VAL_DOWN_ISR() {
  if (pageNumber > 3) {
    valDownMarker = true;
  }
}

void printTimeString() {
  DateTime now = rtc.now();
  lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute());
}

void printDateString() {
  DateTime now = rtc.now();
  lcd.print(now.day());
  lcd.print(".");
  lcd.print(now.month());
  lcd.print(".");
  lcd.print(now.year());
  lcd.print(" (");
  lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
  lcd.print(")");
}

uint32_t getCurrentTimestamp() {
  DateTime now = rtc.now();
  return now.unixtime();
}

void printTempString() {
    lcd.print(waterTemp);
    lcd.print(" oC");
}

void printCurrentProductionString() {
    lcd.print(currentDisplayProduction);
    lcd.print(" kW");
}

void printMinTempString() {
  lcd.print(valuesTab[MIN_TEMP]);
  lcd.print(" oC");
}

void printMaxTempString() {
  lcd.print(valuesTab[MAX_TEMP]);
  lcd.print(" oC");
}

void printMinProductionString() {
  lcd.print(valuesTab[MIN_PRODUCTION]);
  lcd.print(" kW");
}

void printExpectedHourString() {
  lcd.print(valuesTab[EXPECTED_HOUR]);
}

void getCurrentTemp() {
  sensors.requestTemperatures(); 
  float cachedTemp = sensors.getTempCByIndex(0);
  waterTemp = cachedTemp;
}

String stringify(double value) {
  return String(value);
}

String stringify(int value) {
  return String(value);
}

String stringify(bool value) {
  return String(value ? "True" : "False");
}

void setup() {
  EEPROM.begin(EEPROM_SIZE);
  delay(2000);
  uint8_t memoryMinTemp = EEPROM.read(0);
  uint8_t memoryMaxTemp = EEPROM.read(1);
  uint8_t memoryMinProduction = EEPROM.read(2);
  uint8_t memoryExpectedHour = EEPROM.read(3);

  valuesTab[MIN_TEMP] = memoryMinTemp;
  valuesTab[MAX_TEMP] = memoryMaxTemp;
  valuesTab[MIN_PRODUCTION] = memoryMinProduction;
  valuesTab[EXPECTED_HOUR] = memoryExpectedHour;

  I2C2.begin(21, 22, 100000); // Start I2C2 on pins 21 and 22

  lcd.init(I2C_SDA2, I2C_SCL2);                      // initialize the lcd 
  lcd.backlight();
  sensors.begin();
  Serial.begin(9600);
  SerialModbus.begin(9600, SERIAL_8N1, RX, TX);
  delay(100);

  MODBUS_INTERFACE.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc, uint8_t* data, size_t length) {
    Serial.printf("id 0x%02x fc 0x%02x len %u: 0x", serverAddress, fc, length);
    for (size_t i = 0; i < length; ++i) {
      Serial.printf("%02x", data[i]);
    }
    Serial.println("");
    Serial.print("Raw data: ");
    for (size_t i = 0; i < length; ++i) {
      Serial.printf("%d", data[i]);
    }
    Serial.printf("Value reading: %d", currentReadingValue);
    int temporaryIntValue = 0;

    double temporaryValue = 0;

    
    temporaryValue = (((double) data[0] * 256 + (double) data[1]) / 100);

    if (currentReadingValue == A_PHASE_VOLTAGE_READ || currentReadingValue == B_PHASE_VOLTAGE_READ || currentReadingValue == C_PHASE_VOLTAGE_READ) {
      temporaryValue = ((((double) data[0] * 256) + (double) data[1]) / 10);
    }

    if (currentReadingValue == TOTAL_PRODUCTION_HIGH_READ || currentReadingValue == TOTAL_PRODUCTION_LOW_READ) {
      temporaryValue = ((((double) data[0] * 256) + (double) data[1]) / 1);
    }

    Serial.printf("\nval: %f", temporaryValue);
    Serial.print("\n\n");
    DateTime currentTime = rtc.now();
    if (currentTime.unixtime() - oldMeasureTimeProductionUpdate.unixtime() >= 1800 && currentReadingValue == CURRENT_PRODUCTION_READ) { //30 minutes
    //if (currentTime.unixtime() - oldMeasureTimeProductionUpdate.unixtime() >= 5) { //5 seconds
      currentProduction = temporaryValue;
      oldMeasureTimeProductionUpdate = currentTime;
    }
    if (currentReadingValue == CURRENT_PRODUCTION_READ) {
      currentDisplayProduction = temporaryValue;
    }
    if (currentReadingValue == TODAY_PRODUCTION_READ) {
      todayProductionValue = temporaryValue;
    }
    if (currentReadingValue == TODAY_GENERATION_READ) {
      todayGenerationTime = temporaryValue;
    }
    if (currentReadingValue == A_PHASE_VOLTAGE_READ) {
      a_phase_voltage = temporaryValue;
    }
    if (currentReadingValue == B_PHASE_VOLTAGE_READ) {
      b_phase_voltage = temporaryValue;
    }
    if (currentReadingValue == C_PHASE_VOLTAGE_READ) {
      c_phase_voltage = temporaryValue;
    }
    if (currentReadingValue == TOTAL_PRODUCTION_HIGH_READ) {
      total_production_high = temporaryValue * (2 << 16);
    }
    if (currentReadingValue == TOTAL_PRODUCTION_LOW_READ) {
      total_production_low = temporaryValue;
      total_production = total_production_high + total_production_low;
    }
    Serial.printf("current production: %f\n", currentDisplayProduction);
    Serial.printf("today production: %f\n", todayProductionValue);
    Serial.printf("today generation time: %f\n", todayGenerationTime);
    Serial.printf("a phase voltage: %f\n", a_phase_voltage);
    Serial.printf("b phase voltage: %f\n", b_phase_voltage);
    Serial.printf("c phase voltage: %f\n", c_phase_voltage);
    Serial.printf("total production high-byte: %f\n", total_production_high);
    Serial.printf("total production low-byte: %f\n", total_production_low);
    Serial.printf("total production: %f\n", total_production);
  });
  MODBUS_INTERFACE.onError([](esp32Modbus::Error error) {
    Serial.printf("error: 0x%02x\n\n", static_cast<uint8_t>(error));
    currentProduction = 0;
  });
  MODBUS_INTERFACE.begin();

  // initialization of simple inputs and outputs

  pinMode(RELAY, OUTPUT);
  pinMode(RSE, OUTPUT);
  pinMode(KBD1, INPUT);
  pinMode(KBD2, INPUT);
  pinMode(KBD3, INPUT);
  pinMode(KBD4, INPUT);

  attachInterrupt(KBD2, SETT_UP_ISR, RISING);
  attachInterrupt(KBD4, SETT_DOWN_ISR, RISING);
  attachInterrupt(KBD1, VAL_UP_ISR, RISING);
  attachInterrupt(KBD3, VAL_DOWN_ISR, RISING);

  Wire.begin(21,22);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();

  oldMeasureTime = rtc.now();
  oldMeasureTimeMODBUS = rtc.now();
  oldMeasureTimeProductionUpdate = rtc.now();
  oldMeasureTimeProductionUpdateDisplay = rtc.now();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int t1 = millis();
  int t2 = t1;
  while (WiFi.status() != WL_CONNECTED) {
    t1=millis();
    if (t1-t2 >= 5000) {
      Serial.println("WiFi not connected!");
      break;
    }
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("dom241", password);
    t1=millis();
    t2=t1;
    while (WiFi.status() != WL_CONNECTED) {
    t1=millis();
    if (t1-t2 >= 5000) {
      Serial.println("WiFi not connected!");
      break;
    }
    delay(500);
    Serial.print(".");
    }
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/production/current", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(currentDisplayProduction).c_str());
  });
  
  server.on("/production/today", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(todayProductionValue).c_str());
  });

  server.on("/production/time", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(todayGenerationTime).c_str());
  });
  
  server.on("/temperature/current", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(waterTemp).c_str());
  });

  server.on("/temperature/expected", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(valuesTab[MIN_TEMP]).c_str());
  });

  server.on("/temperature/delta", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(valuesTab[MAX_TEMP]).c_str());
  });
    
  server.on("/production/expected", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(valuesTab[MIN_PRODUCTION]).c_str());
  });
  
  server.on("/heater/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(heaterState).c_str());
  });
  
  server.on("/voltage/a", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(a_phase_voltage).c_str());
  });
  
  server.on("/voltage/b", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(b_phase_voltage).c_str());
  });
  
  server.on("/voltage/c", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(c_phase_voltage).c_str());
  });
  
  server.on("/production/total", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", stringify(total_production).c_str());
  });

  server.begin();
}

void loop() {
  lcd.clear();
  lcd.print(pageTitles[pageNumber]);
  lcd.setCursor(0, 1);

  Serial.print("Current page: ");
  Serial.print(pageNumber);
  Serial.println();

  if (pageUpMarker) {
    pageNumber++;
    pageNumber = pageNumber%13;
    delay(DEBOUNCE_TIME);
    pageUpMarker = false;
  }

  if (pageDownMarker) {
    pageNumber--;
    if (pageNumber < 0) {
      pageNumber = 12;
    }
    delay(DEBOUNCE_TIME);
    pageDownMarker = false;
  }

  switch (pageNumber){
  
  case CURRENT_PRODUCTION: {
    printCurrentProductionString();
    valDownMarker = false;
    valUpMarker = false;
    break;
  }

  case CURRENT_TIME: {
    printTimeString();
    valDownMarker = false;
    valUpMarker = false;
    break;
  }

  case CURRENT_DATE: {
    printDateString();
    valDownMarker = false;
    valUpMarker = false;
    break;
  }
  
  case CURRENT_TEMP: {
    printTempString();
    valDownMarker = false;
    valUpMarker = false;
    break;
  }

  case MIN_TEMP: {

    if (valUpMarker) {
      valuesTab[MIN_TEMP]++;
      if (valuesTab[MIN_TEMP] > 80) {
        valuesTab[MIN_TEMP] = 80;
      }
      valUpMarker = false;
    }
    
    if (valDownMarker) {
      valuesTab[MIN_TEMP]--;
      if (valuesTab[MIN_TEMP] < 30) {
        valuesTab[MIN_TEMP] = 30;
      }
      valDownMarker = false;
    }

    printMinTempString();
    if (valuesTab[MIN_TEMP] != minWaterTemp) {
      minWaterTemp = valuesTab[MIN_TEMP];
      EEPROM.write(0, (uint8_t)valuesTab[MIN_TEMP]);
      EEPROM.commit();
    }
    break;
  }

  case MAX_TEMP: {

    if (valUpMarker) {
      valuesTab[MAX_TEMP]++;
      if (valuesTab[MAX_TEMP] > 90) {
        valuesTab[MAX_TEMP] = 90;
      }
      valUpMarker = false;
    }
    
    if (valDownMarker) {
      if (valuesTab[MAX_TEMP] == 0) {
        valuesTab[MAX_TEMP] = 0;
      } else {
        valuesTab[MAX_TEMP]--;
      }
      valDownMarker = false;
    }

    printMaxTempString();
    if (valuesTab[MAX_TEMP] != maxWaterTemp) {
      maxWaterTemp = valuesTab[MAX_TEMP];
      EEPROM.write(1, (uint8_t)valuesTab[MAX_TEMP]);
      EEPROM.commit();
    }
    break;
  }

  case MIN_PRODUCTION: {

    if (valUpMarker) {
      valuesTab[MIN_PRODUCTION]++;
      valUpMarker = false;
    }

    if (valDownMarker) {
      valuesTab[MIN_PRODUCTION]--;
      valDownMarker = false;
    }

    printMinProductionString();
    if (valuesTab[MIN_PRODUCTION] != minPrduction) {
      minPrduction = valuesTab[MIN_PRODUCTION];
      EEPROM.write(2, (uint8_t)valuesTab[MIN_PRODUCTION]);
      EEPROM.commit();
    }
    break;
  }

  case EXPECTED_HOUR: {

    if (valUpMarker) {
      valuesTab[EXPECTED_HOUR]++;
      if (valuesTab[EXPECTED_HOUR] > 23) {
        valuesTab[EXPECTED_HOUR] = 0;
      }
      valUpMarker = false;
    }

    if (valDownMarker) {
      if (valuesTab[EXPECTED_HOUR] == 0) {
          valuesTab[EXPECTED_HOUR] = 23;
      } else {
        valuesTab[EXPECTED_HOUR]--;
      }
      valDownMarker = false;
    }

    printExpectedHourString();
    if (valuesTab[EXPECTED_HOUR] != hourTempExpected) {
      hourTempExpected = valuesTab[EXPECTED_HOUR];
      EEPROM.write(3, (uint8_t)valuesTab[EXPECTED_HOUR]);
      EEPROM.commit();
    }
    break;
  }

  case HOUR_SETTING: {
    DateTime currentTime(rtc.now());
    int temporaryHour = currentTime.hour();
    if ( valUpMarker ) {
      if (currentTime.hour() + 1 < 24){
        rtc.adjust(DateTime(currentTime.year(), currentTime.month(), currentTime.day(), currentTime.hour() + 1, currentTime.minute(), currentTime.second()));
        valUpMarker = false;
      }
    }
    if (valDownMarker) {
      if (currentTime.hour() - 1 > 0) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month(), currentTime.day(), currentTime.hour() - 1, currentTime.minute(), currentTime.second()));
        valDownMarker = false;
      }
    }
    lcd.print(currentTime.hour());
    break;
  }

  case MINUTE_SETTING: {
    DateTime currentTime(rtc.now());
    if (valUpMarker) {
      if (currentTime.minute() + 1 < 59) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month(), currentTime.day(), currentTime.hour(), currentTime.minute() + 1, currentTime.second()));
        valUpMarker = false;
      } 
    }
    if (valDownMarker) {
      if (currentTime.minute() - 1 >= 0) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month(), currentTime.day(), currentTime.hour(), currentTime.minute() - 1, currentTime.second()));
        valDownMarker = false;
      } 
    }
    lcd.print(currentTime.minute());
    break;
  }

  case DAY_SETTING: {
    DateTime currentTime(rtc.now());
    if (valUpMarker) {
      if (currentTime.day() + 1 < 32) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month(), currentTime.day() + 1, currentTime.hour(), currentTime.minute(), currentTime.second()));
        valUpMarker = false;
      } 
    }
    if (valDownMarker) {
      if (currentTime.day() - 1 > 0) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month(), currentTime.day() - 1, currentTime.hour(), currentTime.minute(), currentTime.second()));
        valDownMarker = false;
      } 
    }
    lcd.print(currentTime.day());
    break;
  }

  case MONTH_SETTING: {
    DateTime currentTime(rtc.now());
    if (valUpMarker) {
      if (currentTime.month() + 1 < 13) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month() + 1, currentTime.day(), currentTime.hour(), currentTime.minute(), currentTime.second()));
        valUpMarker = false;
      } 
    }
    if (valDownMarker) {
      if (currentTime.month() - 1 > 0) {
        rtc.adjust(DateTime(currentTime.year(), currentTime.month() - 1, currentTime.day(), currentTime.hour(), currentTime.minute(), currentTime.second()));
        valDownMarker = false;
      } 
    }
    lcd.print(currentTime.month());
    break;
  }

  case YEAR_SETTING: {
    DateTime currentTime(rtc.now());
    if (valUpMarker) {
      if (currentTime.year() + 1 < 3000) {
        rtc.adjust(DateTime(currentTime.year() + 1, currentTime.month(), currentTime.day(), currentTime.hour(), currentTime.minute(), currentTime.second()));
        valUpMarker = false;
      } 
    }
    if (valDownMarker) {
      if (currentTime.month() - 1 > 1970) {
        rtc.adjust(DateTime(currentTime.year() + 1, currentTime.month(), currentTime.day(), currentTime.hour(), currentTime.minute(), currentTime.second()));
        valDownMarker = false;
      } 
    }
    lcd.print(currentTime.year());
    break;
  }

  default: {
    lcd.print("ERROR");
    break;
  }
  }
  DateTime currentTime(rtc.now());
  if (currentTime.unixtime() - oldMeasureTimeMODBUS.unixtime() >= 2) {
    ++currentReadingValue;
    currentReadingValue=currentReadingValue%8;
    if (currentReadingValue == CURRENT_PRODUCTION_READ){
      Serial.print("sending Modbus request (read current production)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x0C, 2);
    }
    if (currentReadingValue == TODAY_PRODUCTION_READ) {
      Serial.print("sending Modbus request (read today production)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x19, 2);
    }
    if (currentReadingValue == TODAY_GENERATION_READ){
      Serial.print("sending Modbus request (read today generation time)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x1A, 2);
    }
    if (currentReadingValue == A_PHASE_VOLTAGE_READ){
      Serial.print("sending Modbus request (read a-phase voltage)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x0F, 2);
    }
    if (currentReadingValue == B_PHASE_VOLTAGE_READ){
      Serial.print("sending Modbus request (read a-phase voltage)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x11, 2);
    }
    if (currentReadingValue == C_PHASE_VOLTAGE_READ){
      Serial.print("sending Modbus request (read a-phase voltage)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x13, 2);
    }
    if (currentReadingValue == TOTAL_PRODUCTION_HIGH_READ){
      Serial.print("sending Modbus request (read total production high byte)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x15, 2);
    }
    if (currentReadingValue == TOTAL_PRODUCTION_LOW_READ){
      Serial.print("sending Modbus request (read total production low byte)...\n");
      MODBUS_INTERFACE.readHoldingRegisters(0x01, 0x16, 2);
    }
    oldMeasureTimeMODBUS = currentTime;
  }

  if (currentTime.unixtime() - oldMeasureTime.unixtime() >= 1) {
    getCurrentTemp();
    oldMeasureTime = currentTime;
  }

  if (currentProduction < valuesTab[MIN_PRODUCTION]) {
    heaterState = false;
    if (valuesTab[EXPECTED_HOUR] - 3 <= currentTime.hour()) {
      heaterState = true;
      if ((int)waterTemp >= valuesTab[MIN_TEMP] + valuesTab[MAX_TEMP]) {
        HEAT_MAX_FLAG = true;
      }
      if ((int)waterTemp <= valuesTab[MIN_TEMP] - valuesTab[MAX_TEMP]) {
        HEAT_MAX_FLAG = false;
      }
      if (abs(valuesTab[MIN_TEMP] - (int)waterTemp) <= valuesTab[MAX_TEMP]) {
        if (HEAT_MAX_FLAG) {
          heaterState = false;
        } else {
          heaterState = true;
        }
      }
      if ((int)waterTemp < valuesTab[MIN_TEMP] - valuesTab[MAX_TEMP]) {
        heaterState = true;
      }
    }
  // } else {
  //     heaterState = true;
  //     if ((int)waterTemp >= valuesTab[MIN_TEMP] + valuesTab[MAX_TEMP]) {
  //       HEAT_MAX_FLAG = true;
  //     }
  //     if ((int)waterTemp <= valuesTab[MIN_TEMP] - valuesTab[MAX_TEMP]) {
  //       HEAT_MAX_FLAG = false;
  //     }
  //     if (abs(valuesTab[MIN_TEMP] - (int)waterTemp) <= valuesTab[MAX_TEMP]) {
  //       if (HEAT_MAX_FLAG) {
  //         heaterState = false;
  //       } else {
  //         heaterState = true;
  //       }
  //     }
  //     if ((int)waterTemp < valuesTab[MIN_TEMP] - valuesTab[MAX_TEMP]) {
  //       heaterState = true;
  //     }
  // }
  } else {
      heaterState = true;
      if ((int)waterTemp >= 75 + 5) {
        HEAT_MAX_FLAG = true;
      }
      if ((int)waterTemp <= 75 - 5) {
        HEAT_MAX_FLAG = false;
      }
      if (abs(75 - (int)waterTemp) <= 5) {
        if (HEAT_MAX_FLAG) {
          heaterState = false;
        } else {
          heaterState = true;
        }
      }
      if ((int)waterTemp < 75 - 5) {
        heaterState = true;
      }
  }

  if ((int)waterTemp >= valuesTab[MIN_TEMP] + valuesTab[MAX_TEMP]) {
    heaterState = false;
  }

  if ((int)waterTemp <= 40) {
    heaterState = true;
  }

  if (currentTime.hour() >= 22 || currentTime.hour() <= 6) {
    heaterState = true;
      if ((int)waterTemp >= valuesTab[MIN_TEMP] + valuesTab[MAX_TEMP]) {
        HEAT_MAX_FLAG = true;
      }
      if ((int)waterTemp <= valuesTab[MIN_TEMP] - valuesTab[MAX_TEMP]) {
        HEAT_MAX_FLAG = false;
      }
      if (abs(valuesTab[MIN_TEMP] - (int)waterTemp) <= valuesTab[MAX_TEMP]) {
        if (HEAT_MAX_FLAG) {
          heaterState = false;
        } else {
          heaterState = true;
        }
      }
      if ((int)waterTemp < valuesTab[MIN_TEMP] - valuesTab[MAX_TEMP]) {
        heaterState = true;
      }
  }

  if (heaterState) {
    digitalWrite(RELAY, LOW);
  } else {
    digitalWrite(RELAY, HIGH);
  }

  delay(50);

}

/*
MAX_TEMP = DELTA TEMP
*/