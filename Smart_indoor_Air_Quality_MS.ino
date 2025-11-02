/***************************************************
   Smart Indoor Air Quality Management System
   Hardware: ESP32-WROOM-DA + DHT22 + MQ135 + MQ2 +
             Voltage Sensor + ACS712 Current Sensor +
             16x2 LCD (I2C) + 3x Relay
   Dashboard: Blynk IoT (blynk.cloud)
****************************************************/

#define BLYNK_TEMPLATE_ID "TMPL3cCtiJVnD"
#define BLYNK_TEMPLATE_NAME "Smart Air Quality"
#define BLYNK_AUTH_TOKEN "xMnDBq1ckdYZaNL-pmNDnmx3ko-G0ENi"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Sensor Pins ---
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define MQ135_PIN 36
#define MQ2_PIN 39
#define VOLTAGE_PIN 34
#define CURRENT_PIN 35

// --- Relay Pins ---
#define RELAY_AC 18
#define RELAY_FAN 19
#define RELAY_PURIFIER 15

// --- WiFi Credentials ---
char ssid[] = "indoorAirquality";
char pass[] = "123456789";

// --- Blynk Virtual Pins ---
const int VPIN_TEMP       = V0;
const int VPIN_HUM        = V1;
const int VPIN_AIR        = V2;
const int VPIN_GAS        = V3;
const int VPIN_VOLTAGE    = V4;
const int VPIN_CURRENT    = V5;
const int VPIN_AC         = V7;
const int VPIN_FAN        = V8;
const int VPIN_PURIFIER   = V9;
const int VPIN_TEMP_SET   = V10;
const int VPIN_HUM_SET    = V11;

// --- Sensor Variables ---
float temperature = 0.0;
float humidity = 0.0;
int airQualityRaw = 0;
int gasRaw = 0;
float measuredVoltage = 0.0;
float measuredCurrent = 0.0;
float setTemp = 28.0;
float setHum = 65.0;
bool acState = false;
bool fanState = false;
bool purifierState = false;

// --- Calibration Constants ---
const float ADC_REF_VOLTAGE = 3.3f;
const int ADC_MAX = 4095;
const float VOLTAGE_DIVIDER_RATIO = 100.0f; // Adjust to your voltage divider
const float CURRENT_SENSITIVITY = 0.185f;   // V/A for ACS712-5A module
const float CURRENT_ZERO_VOLTAGE = 1.65f;   // Midpoint (no current)
const float VOLTAGE_LOW_THRESHOLD = 190.0f;
const float VOLTAGE_HIGH_THRESHOLD = 250.0f;
const float CURRENT_HIGH_THRESHOLD = 10.0f;
const unsigned long ALERT_MIN_INTERVAL_MS = 60000UL;

unsigned long lastVoltageAlert = 0;
unsigned long lastCurrentAlert = 0;
unsigned long lastAirAlert = 0;
unsigned long lastTempAlert = 0;
int lcdPage = 0;

BlynkTimer timer;

// --- Function Prototypes ---
void readSensors();
void updateLCD();

// --- Blynk Handlers ---
BLYNK_WRITE(VPIN_AC) {
  acState = param.asInt();
  digitalWrite(RELAY_AC, acState ? LOW : HIGH);
}

BLYNK_WRITE(VPIN_FAN) {
  fanState = param.asInt();
  digitalWrite(RELAY_FAN, fanState ? LOW : HIGH);
}

BLYNK_WRITE(VPIN_PURIFIER) {
  purifierState = param.asInt();
  digitalWrite(RELAY_PURIFIER, purifierState ? LOW : HIGH);
}

BLYNK_WRITE(VPIN_TEMP_SET) { setTemp = param.asFloat(); }
BLYNK_WRITE(VPIN_HUM_SET) { setHum = param.asFloat(); }

// --- Setup ---
void setup() {
  Serial.begin(115200);
  dht.begin();
  lcd.init();
  lcd.backlight();

  // ADC configuration
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);

  // Relay setup
  pinMode(RELAY_AC, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_PURIFIER, OUTPUT);
  digitalWrite(RELAY_AC, HIGH);
  digitalWrite(RELAY_FAN, HIGH);
  digitalWrite(RELAY_PURIFIER, HIGH);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Air System");
  lcd.setCursor(0, 1);
  lcd.print("Connecting...");
  delay(2000);

  timer.setInterval(2000L, readSensors);
  timer.setInterval(3000L, updateLCD);
}

// --- Main Loop ---
void loop() {
  Blynk.run();
  timer.run();
}

// --- Sensor Reading Function ---
void readSensors() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  airQualityRaw = analogRead(MQ135_PIN);
  gasRaw = analogRead(MQ2_PIN);

  // --- Voltage Measurement ---
  int rawVolt = analogRead(VOLTAGE_PIN);
  float voltsAtADC = (rawVolt * ADC_REF_VOLTAGE) / ADC_MAX;
  measuredVoltage = voltsAtADC * VOLTAGE_DIVIDER_RATIO;

  // --- ACS712 Current Measurement ---
  const int numSamples = 10;
  float currentSum = 0;
  for (int i = 0; i < numSamples; i++) {
    int rawCurr = analogRead(CURRENT_PIN);
    float currSensorVoltage = (rawCurr * ADC_REF_VOLTAGE) / ADC_MAX;
    float currentValue = (currSensorVoltage - CURRENT_ZERO_VOLTAGE) / CURRENT_SENSITIVITY;
    currentSum += currentValue;
    delay(5);
  }
  measuredCurrent = currentSum / numSamples;
  if (measuredCurrent < 0.01f) measuredCurrent = 0.0f;

  // --- Blynk Update ---
  Blynk.virtualWrite(VPIN_TEMP, temperature);
  Blynk.virtualWrite(VPIN_HUM, humidity);
  Blynk.virtualWrite(VPIN_AIR, airQualityRaw);
  Blynk.virtualWrite(VPIN_GAS, gasRaw);
  Blynk.virtualWrite(VPIN_VOLTAGE, measuredVoltage);
  Blynk.virtualWrite(VPIN_CURRENT, measuredCurrent);

  // --- Automation Logic ---
  bool autoAC = temperature > setTemp;
  bool autoFan = humidity > setHum;
  bool autoPurifier = (airQualityRaw > 2500 || gasRaw > 2300);

  acState = acState || autoAC;
  fanState = fanState || autoFan;
  purifierState = purifierState || autoPurifier;

  digitalWrite(RELAY_AC, acState ? LOW : HIGH);
  digitalWrite(RELAY_FAN, fanState ? LOW : HIGH);
  digitalWrite(RELAY_PURIFIER, purifierState ? LOW : HIGH);

  // --- Alerts ---
  unsigned long now = millis();
  if ((measuredVoltage < VOLTAGE_LOW_THRESHOLD || measuredVoltage > VOLTAGE_HIGH_THRESHOLD) && now - lastVoltageAlert > ALERT_MIN_INTERVAL_MS) {
    lastVoltageAlert = now;
    Blynk.logEvent("voltage_alert", "Voltage fluctuation: " + String(measuredVoltage, 1) + "V");
  }

  if (measuredCurrent > CURRENT_HIGH_THRESHOLD && now - lastCurrentAlert > ALERT_MIN_INTERVAL_MS) {
    lastCurrentAlert = now;
    Blynk.logEvent("current_alert", "High current: " + String(measuredCurrent, 2) + "A");
  }

  if ((airQualityRaw > 3000 || gasRaw > 2800) && now - lastAirAlert > ALERT_MIN_INTERVAL_MS) {
    lastAirAlert = now;
    Blynk.logEvent("air_quality_alert", "Bad Air Quality Detected!");
  }

  if (temperature > (setTemp + 5.0f) && now - lastTempAlert > ALERT_MIN_INTERVAL_MS) {
    lastTempAlert = now;
    Blynk.logEvent("temp_alert", "Temperature exceeded set limit!");
  }

  // --- Debugging Info ---
  Serial.printf("T:%.1f H:%.0f AQ:%d G:%d V:%.1fV I:%.2fA AC:%d FAN:%d PUR:%d\n",
                temperature, humidity, airQualityRaw, gasRaw,
                measuredVoltage, measuredCurrent,
                acState, fanState, purifierState);
}

// --- LCD Update Function ---
void updateLCD() {
  lcd.clear();
  if (lcdPage == 0) {
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temperature, 1);
    lcd.print("C H:");
    lcd.print(humidity, 0);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("AC:");
    lcd.print(acState ? "ON " : "OFF");
    lcd.print(" F:");
    lcd.print(fanState ? "ON" : "OFF");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.print(measuredVoltage, 1);
    lcd.print("V I:");
    lcd.print(measuredCurrent, 1);

    lcd.setCursor(0, 1);
    lcd.print("Air:");
    if (airQualityRaw > 3000 || gasRaw > 2800)
      lcd.print("BAD ");
    else if (airQualityRaw > 2000)
      lcd.print("OK  ");
    else
      lcd.print("GOOD");
    lcd.print(" P:");
    lcd.print(purifierState ? "ON" : "OFF");
  }
  lcdPage = (lcdPage + 1) % 2;
}
