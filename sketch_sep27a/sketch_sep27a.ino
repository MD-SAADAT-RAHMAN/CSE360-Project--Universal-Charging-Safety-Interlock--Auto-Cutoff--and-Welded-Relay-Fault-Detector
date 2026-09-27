#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <INA226_WE.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ================= PIN DEFINITIONS =================
#define MQ2_ANALOG_PIN A0
#define DS18B20_PIN 2
#define RED_LED 3
#define BTN_7DAY_WARN 4
#define YELLOW_LED 5
#define GREEN_LED 6
#define RELAY_PIN 7
#define BUZZER_PIN 8
#define BTN_CRITICAL_DEMO 9
#define BTN_RESET 10

// ================= THRESHOLDS & TIMERS =================
#define I2C_LCD_ADDR 0x27
#define I2C_INA_ADDR 0x44 // Custom INA226 address
#define CRITICAL_TEMP 45.0 // Degrees Celsius
#define CRITICAL_GAS 350 // Gas baseline is usually ~100-150
#define FULL_CHARGE_MA 50 // Drops below 50mA when fully charged/disconnected

#define FAULT_CONFIRM_MS 2000 // Fault must persist for 2 solid seconds
#define CHARGE_CONFIRM_MS 10000 // Current must stay < 50mA for 10 straight seconds

// ================= SYSTEM STATES =================
enum SystemState { NORMAL, WARNING, CRITICAL, CHARGED };
SystemState currentState = NORMAL;

// ================= OBJECTS & VARIABLES =================
LiquidCrystal_I2C lcd(I2C_LCD_ADDR, 16, 2);
INA226_WE ina226(I2C_INA_ADDR);
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

// Signal Processing (Noise Filters)
float smoothedGas = 0;
float smoothedTemp = 0;
const float EMA_ALPHA = 0.2; // 20% new reading, 80% old history

// Timers
unsigned long lastReadTime = 0;
unsigned long faultStartTime = 0;
unsigned long fullChargeStartTime = 0;
unsigned long alarmToggleTime = 0;
bool alarmState = false;

void setup() {
Serial.begin(9600);
Wire.begin();

// 1. Hardware Pin Setup
pinMode(RELAY_PIN, OUTPUT);
pinMode(BUZZER_PIN, OUTPUT);
pinMode(RED_LED, OUTPUT);
pinMode(YELLOW_LED, OUTPUT);
pinMode(GREEN_LED, OUTPUT);

pinMode(BTN_7DAY_WARN, INPUT_PULLUP);
pinMode(BTN_CRITICAL_DEMO, INPUT_PULLUP);
pinMode(BTN_RESET, INPUT_PULLUP);

// 2. Boot Sequence
lcd.init();
lcd.backlight();
lcd.setCursor(0, 0);
lcd.print("BMS Booting... ");

// 3. Initialize Sensors
if (!ina226.init()) {
lcd.setCursor(0, 1);
lcd.print("INA226 MISSING! ");
while(1); // Freeze if INA226 is disconnected
}
ina226.setAverage(INA226_AVERAGE_16);
tempSensor.begin();

delay(2000);
resetSystem(); // This starts the system in the safe NORMAL state
}

void loop() {
unsigned long currentMillis = millis();

// --- EXCLUSIVE STATE: CRITICAL FAULT ---
if (currentState == CRITICAL) {
if (currentMillis - alarmToggleTime >= 200) {
alarmToggleTime = currentMillis;
alarmState = !alarmState;
digitalWrite(BUZZER_PIN, alarmState);
}
checkResetButton();
return; // Block all sensor reading while in critical lockdown
}

// --- EXCLUSIVE STATE: FULLY CHARGED ---
if (currentState == CHARGED) {
checkResetButton();
return; // Block all sensor reading when charging is finished
}

// --- NON-BLOCKING SENSOR READ (Every 300ms) ---
if (currentMillis - lastReadTime >= 300) {
lastReadTime = currentMillis;

// 1. Gather Raw Data
float busVoltage = ina226.getBusVoltage_V();
float current_mA = ina226.getCurrent_mA();
int rawGas = analogRead(MQ2_ANALOG_PIN);
tempSensor.requestTemperatures();
float rawTemp = tempSensor.getTempCByIndex(0);

// 2. EMA Filter (Smooths out random electrical noise)
if (smoothedGas == 0) smoothedGas = rawGas;
if (smoothedTemp == 0) smoothedTemp = rawTemp;
smoothedGas = (EMA_ALPHA * rawGas) + ((1.0 - EMA_ALPHA) * smoothedGas);
smoothedTemp = (EMA_ALPHA * rawTemp) + ((1.0 - EMA_ALPHA) * smoothedTemp);

// 3. Update LCD strictly for 16-character width
lcd.setCursor(0, 0);
lcd.print("V:"); lcd.print(busVoltage, 1);
lcd.print(" I:"); lcd.print((int)current_mA); lcd.print("mA ");

lcd.setCursor(0, 1);
lcd.print("T:"); lcd.print(smoothedTemp, 1);
lcd.print(" G:"); lcd.print((int)smoothedGas);

// Status Indicator
if (currentState == WARNING) {
lcd.print(" WN ");
} else {
lcd.print(" OK ");
}

// 4. FAULT DETECTION (Sensor Fusion Logic)
bool tempSpike = (smoothedTemp >= CRITICAL_TEMP);
bool ventingDetected = (smoothedGas >= CRITICAL_GAS && smoothedTemp >= 35.0);
bool demoFault = (digitalRead(BTN_CRITICAL_DEMO) == LOW);

if (tempSpike || ventingDetected || demoFault) {
if (faultStartTime == 0) {
faultStartTime = currentMillis; // Start the 2-second stopwatch
} else if (currentMillis - faultStartTime >= FAULT_CONFIRM_MS) {
executeCutoff();
}
} else {
faultStartTime = 0; // Reset timer if the fault disappears quickly
}

// 5. FULL CHARGE DETECTION (Tail Current Logic)
if (busVoltage > 4.0 && current_mA < FULL_CHARGE_MA) {
if (fullChargeStartTime == 0) {
fullChargeStartTime = currentMillis; // Start 10-second stopwatch
} else if (currentMillis - fullChargeStartTime >= CHARGE_CONFIRM_MS) {
executeFullCharge();
}
} else {
fullChargeStartTime = 0; // Reset timer if current jumps back up
}
}

// --- DEMO LOGIC: 7-DAY WARNING ---
if (digitalRead(BTN_7DAY_WARN) == LOW && currentState == NORMAL) {
currentState = WARNING;
digitalWrite(GREEN_LED, LOW);
digitalWrite(YELLOW_LED, HIGH);

// Quick warning triple-beep
for(int i=0; i<3; i++) {
digitalWrite(BUZZER_PIN, HIGH); delay(100);
digitalWrite(BUZZER_PIN, LOW); delay(100);
}
}
}

// ================= SYSTEM FUNCTIONS =================

void executeCutoff() {
currentState = CRITICAL;

// ACTIVE-LOW RELAY LOGIC: HIGH = OFF (Power Severed)
digitalWrite(RELAY_PIN, HIGH);

digitalWrite(GREEN_LED, LOW);
digitalWrite(YELLOW_LED, LOW);
digitalWrite(RED_LED, HIGH);

lcd.clear();
lcd.setCursor(0, 0); lcd.print("THERMAL RUNAWAY!");
lcd.setCursor(0, 1); lcd.print("POWER SEVERED!!!");
}

void executeFullCharge() {
currentState = CHARGED;

// ACTIVE-LOW RELAY LOGIC: HIGH = OFF (Prevents Trickle Overcharging)
digitalWrite(RELAY_PIN, HIGH);

digitalWrite(GREEN_LED, HIGH);
digitalWrite(YELLOW_LED, LOW);
digitalWrite(RED_LED, LOW);

lcd.clear();
lcd.setCursor(0, 0); lcd.print("BATTERY 100% ");
lcd.setCursor(0, 1); lcd.print("CHARGING HALTED ");

// Happy confirmation beep
digitalWrite(BUZZER_PIN, HIGH); delay(500); digitalWrite(BUZZER_PIN, LOW);
}

void resetSystem() {
currentState = NORMAL;
faultStartTime = 0;
fullChargeStartTime = 0;

// ACTIVE-LOW RELAY LOGIC: LOW = ON (Power Restored)
digitalWrite(RELAY_PIN, LOW);

digitalWrite(GREEN_LED, HIGH);
digitalWrite(YELLOW_LED, LOW);
digitalWrite(RED_LED, LOW);
digitalWrite(BUZZER_PIN, LOW);

lcd.clear();
lcd.setCursor(0, 0); lcd.print("SYSTEM RESETTING");
delay(1000);
lcd.clear();
}

void checkResetButton() {
if (digitalRead(BTN_RESET) == LOW) {
resetSystem();
delay(500); // Debounce to prevent double-clicking
}
}