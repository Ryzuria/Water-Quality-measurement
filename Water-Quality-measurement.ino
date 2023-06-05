#define SIM800L_IP5306_VERSION_20190610
#include "utilities.h"
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#define TINY_GSM_DEBUG Serial
#define SerialAT Serial1
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient mqtt(client);

#include "DFRobot_ESP_EC.h"
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include <EEPROM.h>
#include <SmoothThermistor.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#define RXPin (19)
#define TXPin (18)
TinyGPSPlus gps;
HardwareSerial ss(2);

static const uint32_t GPSBaud = 19200;
const char *mqtt_server = "broker.hivemq.com";
const char *mqttpub = "esp32/waterQA";
const char *mqttsub = "esp32/callback"; 
const char apn[] = "www.dtac.co.th"; //edit to fit your apn
const char gprsUser[] = "";
const char gprsPass[] = "";
char json[100];
int id = 1001;

DFRobot_ESP_EC ec;
DFRobot_ESP_PH_WITH_ADC ph;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_ADS1115 ads;
SmoothThermistor smoothThermistor(14,              // the analog pin to read from
                                  ADC_SIZE_12_BIT, // the ADC size
                                  10000,           // the nominal resistance
                                  10000,           // the series resistance
                                  3950,            // the beta coefficient of the thermistor
                                  25,              // the temperature for nominal resistance
                                  10);

#define SW1_PIN 34
#define SW2_PIN 35
#define SW3_PIN 32
#define SW4_PIN 33
#define SW5_PIN 25

String msgcallback = "DWN to send data";
unsigned int current_mode = 0;
int max_mode = 4;
int select_mode = 1;
int calibrate_mode = 0;
int networkstatus = 0;

int but_1_last_state = HIGH;
int but_2_last_state = HIGH;
int but_3_last_state = HIGH;
int but_4_last_state = HIGH;
int but_5_last_state = HIGH;
int but_1_state = HIGH;
int but_2_state = HIGH;
int but_3_state = HIGH;
int but_4_state = HIGH;
int but_5_state = HIGH;

const int DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

int doCalVol;
int tempCal = 25;
float ecVoltage, phVoltage, doVoltage, temp;
float ecValue, phValue, doValue, saltValue, latValue, lonValue;

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  EEPROM.begin(512);
  Serial.println("Setup begin");
  // Sensor
  ec.begin();
  ph.begin();
  doCalVol = (EEPROM.read(20) * 256) + EEPROM.read(21); // read from address 20
  if (doCalVol == 0xFF)
  { // if the value is not set, use default value
    doCalVol = 700;
  }
  EEPROM.end(); // end the EEPROM read
  // GSP
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  // Switch
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  pinMode(SW4_PIN, INPUT_PULLUP);
  pinMode(SW5_PIN, INPUT_PULLUP);
  // ADC
  ads.setGain(GAIN_ONE);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  // initialize LCD
  lcd.begin();
  lcd.backlight();
  // temperature
  smoothThermistor.useAREF(false);
  lcd.clear();
  Serial.println("Setup complete");
}

void loop()
{
  //gps
  gpsCordinate(); //get cordinate
  but_1_state = digitalRead(SW1_PIN);
  but_2_state = digitalRead(SW2_PIN);
  but_3_state = digitalRead(SW3_PIN);
  but_4_state = digitalRead(SW4_PIN);
  but_5_state = digitalRead(SW5_PIN);
  if (current_mode == 0)
  {
    // 0 main screen
    lcd.setCursor(0, 0);
    lcd.print("Water Measuring");
    lcd.setCursor(0, 2);
    lcd.print("1: EC");
    lcd.setCursor(10, 2);
    lcd.print("2: pH");
    lcd.setCursor(0, 3);
    lcd.print("3: DO");
    lcd.setCursor(10, 3);
    lcd.print("4: Send");
    if (select_mode == 1)
    {
      lcd.setCursor(6, 2);
      lcd.print("<");
      lcd.setCursor(16, 2);
      lcd.print(" ");
      lcd.setCursor(6, 3);
      lcd.print(" ");
      lcd.setCursor(18, 3);
      lcd.print(" ");
      if ((but_1_state != but_1_last_state && but_1_state == LOW) || (but_4_state != but_4_last_state && but_4_state == LOW))
      {
        select_mode = 2;
      }
      else if ((but_2_state != but_2_last_state && but_2_state == LOW) || (but_3_state != but_3_last_state && but_3_state == LOW))
      {
        select_mode = 3;
      }
    }
    else if (select_mode == 2)
    {
      lcd.setCursor(6, 2);
      lcd.print(" ");
      lcd.setCursor(16, 2);
      lcd.print("<");
      lcd.setCursor(6, 3);
      lcd.print(" ");
      lcd.setCursor(18, 3);
      lcd.print(" ");
      if ((but_1_state != but_1_last_state && but_1_state == LOW) || (but_4_state != but_4_last_state && but_4_state == LOW))
      {
        select_mode = 1;
      }
      else if ((but_2_state != but_2_last_state && but_2_state == LOW) || (but_3_state != but_3_last_state && but_3_state == LOW))
      {
        select_mode = 4;
      }
    }
    else if (select_mode == 3)
    {
      lcd.setCursor(6, 2);
      lcd.print(" ");
      lcd.setCursor(16, 2);
      lcd.print(" ");
      lcd.setCursor(6, 3);
      lcd.print("<");
      lcd.setCursor(18, 3);
      lcd.print(" ");
      if ((but_1_state != but_1_last_state && but_1_state == LOW) || (but_4_state != but_4_last_state && but_4_state == LOW))
      {
        select_mode = 4;
      }
      else if ((but_2_state != but_2_last_state && but_2_state == LOW) || (but_3_state != but_3_last_state && but_3_state == LOW))
      {
        select_mode = 1;
      }
    }
    else if (select_mode == 4)
    {
      lcd.setCursor(6, 2);
      lcd.print(" ");
      lcd.setCursor(16, 2);
      lcd.print(" ");
      lcd.setCursor(6, 3);
      lcd.print(" ");
      lcd.setCursor(18, 3);
      lcd.print("<");
      if ((but_1_state != but_1_last_state && but_1_state == LOW) || (but_4_state != but_4_last_state && but_4_state == LOW))
      {
        select_mode = 3;
      }
      else if ((but_2_state != but_2_last_state && but_2_state == LOW) || (but_3_state != but_3_last_state && but_3_state == LOW))
      {
        select_mode = 2;
      }
    }
    if (but_5_state != but_5_last_state && but_5_state == LOW)
    {
      current_mode = select_mode;
      lcd.clear();
    }
    calibrate_mode = 0;
    but_1_last_state = but_1_state;
    but_2_last_state = but_2_state;
    but_3_last_state = but_3_state;
    but_4_last_state = but_4_state;
    but_5_last_state = but_5_state;
    msgcallback = "DWN to send data";
  }

  //EC screen
  if (current_mode == 1)
  {
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U)
    {
      timepoint = millis();
      ecVoltage = ads.readADC_SingleEnded(0) / 10;
      temp = readTemperature();
      ecValue = ec.readEC(ecVoltage, temp);
      if (ecValue < 0)
        ecValue = 0.00;
      saltValue = ecValue * 0.574;
      Serial.print("EC: ");
      Serial.print(ecValue, 2);
      Serial.println("ms/cm");
      Serial.print("Salinity: ");
      Serial.print(saltValue, 2);
      Serial.println("ppt");
    }
    if (calibrate_mode)
    {
      lcd.setCursor(0, 0);
      lcd.print("EC Calibration mode");
      lcd.setCursor(0, 1);
      lcd.print("put probe in buffer");
      lcd.setCursor(0, 2);
      lcd.print("DWN to calculate");
      if (but_3_state != but_3_last_state && but_3_state == LOW)
      {
        ec.ecCalibration(2);
        String calStatus;
        if (ec.ecReturnStatus())
        {
          calStatus = "Success        ";
        }
        else if (!ec.ecReturnStatus())
        {
          calStatus = "Fail, try again";
        }
        lcd.setCursor(0, 3);
        lcd.print(calStatus);
      }
      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        calibrate_mode = 0;
        ec.ecCalibration(3);
        lcd.clear();
      }
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("EC sensor");
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.print(temp, 1);
      lcd.setCursor(0, 2);
      lcd.print("EC value: ");
      lcd.print(ecValue, 2);
      lcd.setCursor(15, 2);
      lcd.print("ms/cm");
      lcd.setCursor(0, 3);
      lcd.print("Salinity : ");
      lcd.print(saltValue);
      ec.calibration(ecVoltage, temp);
      if (but_3_state != but_3_last_state && but_3_state == LOW)
      {
        calibrate_mode = 1;
        ec.ecCalibration(1);
        lcd.clear();
      }
      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        current_mode = 0;
        lcd.clear();
      }
    }
    but_3_last_state = but_3_state;
    but_5_last_state = but_5_state;
  }

  // pH screen
  if (current_mode == 2)
  {
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) // time interval: 1s
    {
      timepoint = millis();
      phVoltage = ads.readADC_SingleEnded(1) / 10;
      temp = readTemperature();
      phValue = ph.readPH(phVoltage, temp);
      if (phValue < 0) phValue = 0.00;
      Serial.print("pH:");
      Serial.println(phValue, 2);
    }
    if (calibrate_mode)
    {
      lcd.setCursor(0, 0);
      lcd.print("pH Calibration mode");
      lcd.setCursor(0, 1);
      lcd.print("put probe in buffer");
      lcd.setCursor(0, 2);
      lcd.print("DWN to calculate");
      if (but_3_state != but_3_last_state && but_3_state == LOW)
      {
        ph.phCalibration(2);
        String calStatus;
        if (ph.phReturnStatus())
        {
          calStatus = "Success        ";
        }
        else if (!ph.phReturnStatus())
        {
          calStatus = "Fail, try again";
        }
        lcd.setCursor(0, 3);
        lcd.print(calStatus);
      }
      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        calibrate_mode = 0;
        ph.phCalibration(3);
        lcd.clear();
      }
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("pH sensor");
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.print(temp, 1);
      lcd.setCursor(0, 2);
      lcd.print("pH value: ");
      lcd.print(phValue, 2);
      ph.calibration(phVoltage, temp);
      if (but_3_state != but_3_last_state && but_3_state == LOW)
      {
        calibrate_mode = 1;
        ph.phCalibration(1);
        lcd.clear();
      }
      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        current_mode = 0;
        lcd.clear();
      }
    }
    but_3_last_state = but_3_state;
    but_5_last_state = but_5_state;
  }

  //DO screen
  if (current_mode == 3)
  {
    // DO screen
    if (calibrate_mode)
    {
      static unsigned long timepoint = millis();
      if (millis() - timepoint > 1000U) // time interval: 1s
      {
        timepoint = millis();
        doCalVol = ads.computeVolts(ads.readADC_SingleEnded(2)) * 1000;
        Serial.println("DO calibrate voltage(mv) :" + String(doCalVol));
      }
      lcd.setCursor(0, 0);
      lcd.print("DO calibration");
      lcd.setCursor(0, 1);
      lcd.print("DOCAL vol: ");
      lcd.print(doCalVol);
      lcd.print(" mV ");

      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        EEPROM.begin(512);
        EEPROM.write(20, doCalVol / 256); // write value to address
        EEPROM.write(21, doCalVol % 256);
        EEPROM.commit();
        calibrate_mode = 0;
        lcd.clear();
        EEPROM.end();
      }
    }
    else
    {
      static unsigned long timepoint = millis();
      if (millis() - timepoint > 1000U) // time interval: 1s
      {
        temp = readTemperature();
        timepoint = millis();
        doVoltage = ads.computeVolts(ads.readADC_SingleEnded(3)) * 1000;
        doValue = readDO(doVoltage, temp);
        if (doValue < 0)doValue = 0.00;
        Serial.println("DO calibrate voltage(mv) :" + String(doCalVol));
        Serial.println("DO :" + String(doValue) + " mg/L");
      }
      lcd.setCursor(0, 0);
      lcd.print("DO sensor");
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.print(temp, 1);
      lcd.setCursor(0, 2);
      lcd.print("DO value: ");
      lcd.print(doValue, 2);
      lcd.print("mg/L");
      if (but_3_state != but_3_last_state && but_3_state == LOW)
      {
        calibrate_mode = 1;
        lcd.clear();
      }
      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        current_mode = 0;
        lcd.clear();
      }
    }
    but_3_last_state = but_3_state;
    but_5_last_state = but_5_state;
  }

  // Network screen
  if (current_mode == 4)
  {
    // Connecting
    if (networkstatus == 0)
    {
      setupModem();
      lcd.setCursor(0, 0);
      lcd.print("Setup modem         ");
      Serial.println("Wait...");
      // Set GSM module baud rate and UART pins
      SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
      Serial.println("Initializing modem...");
      modem.restart();
      Serial.print("Modem Info: ");
      Serial.println(modem.getModemInfo());
      lcd.setCursor(0, 0);
      lcd.print("Setup timezone      ");
      Serial.print("Waiting for network...");
      if (!modem.waitForNetwork()) {
        Serial.println(" fail");
        lcd.setCursor(0, 1);
        lcd.print("fail to setup    ");
        delay(1000);
        current_mode = 0;
      }
      Serial.println(" success");

      if (modem.isNetworkConnected()) {
        Serial.println("Network connected");
      }
      // GPRS connection parameters are usually set after network registration
      lcd.setCursor(0, 0);
      lcd.print("Connect to network ");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println(" fail");
        lcd.setCursor(0, 1);
        lcd.print("fail to setup    ");
        delay(1000);
        current_mode = 0;
      }
      Serial.println(" success");
      lcd.setCursor(0, 0);
      lcd.print("Connect to MQTT   ");
      if (modem.isGprsConnected()) {
        Serial.println("GPRS connected");
      }
      setupMQTT();
      networkstatus = 1;
      delay(500);
      lcd.clear();
    }
    // Sending data
    else if (networkstatus == 1)
    {
      //      gpsCordinate();
      if (!mqtt.connected()) {
        mqttReconnect();
      }
      sprintf(json, "{\"id\":%d,\"temp\":%.2f,\"salt\":%.2f,\"pH\":%.2f,\"oxy\":%.2f,\"lat\":%.6f,\"lon\":%.6f}",
              id, temp, saltValue, phValue, doValue, latValue, lonValue);
      Serial.println(json);
      lcd.setCursor(0, 0);
      lcd.print("Send data to MQTT");
      lcd.setCursor(0, 1);
      if (latValue > 0.0 && lonValue > 0.0) {
        lcd.print("Location recieved");
      } else {
        lcd.print("No location");
      }
      lcd.setCursor(0, 3);
      lcd.print(msgcallback);

      if (but_3_state != but_3_last_state && but_3_state == LOW)
      {
        mqtt.publish(mqttpub, json);
      }
      if (but_5_state != but_5_last_state && but_5_state == LOW)
      {
        current_mode = 0;
        lcd.clear();
      }
      mqtt.loop();
    }
    but_3_last_state = but_3_state;
    but_5_last_state = but_5_state;
  }

  // clears the display to print new message
  delay(20);
}
//gps
void gpsCordinate() {
  while (ss.available() > 0)
    if (gps.encode(ss.read())) {
      if (gps.location.isValid())
      {
        latValue = gps.location.lat();
        lonValue = gps.location.lng();
        Serial.print("LAT: ");
        Serial.print(gps.location.lat());
        Serial.print(", LON: ");
        Serial.print(gps.location.lng());
        Serial.println();
      }
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No GPS detected");
    while (true);
  }
}

float readTemperature()
{
  return smoothThermistor.temperature();
}

float readDO(int voltage_mv, int temp)
{
  int V_saturation = doCalVol + (35 * temp) - (tempCal * 35);
  return (voltage_mv * DO_Table[temp] / V_saturation) / 1000.0;
}

void setupMQTT()
{
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(mqttCallback);
}

void mqttReconnect()
{
  Serial.println("Connecting to MQTT Broker...");
  while (!mqtt.connected())
  {
    Serial.println("Reconnecting to MQTT Broker..");
    String clientId = "WaterQualityESP32-" + String(id);

    if (mqtt.connect(clientId.c_str()))
    {
      Serial.println("Connected.");
      // subscribe to topic
      mqtt.subscribe(mqttsub);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();
  if (String(topic) == mqttsub)
    msgcallback = "Data sent!          ";
}
