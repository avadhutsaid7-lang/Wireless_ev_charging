#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>
#include <SPI.h>

// ================= PIN CONFIG =================
#define ENTRY_SERVO 18
#define EXIT_SERVO 19
#define RELAY_PIN 26
#define BUZZER_PIN 27

#define TRIG_PIN 5
#define ECHO_PIN 4

#define ENTRY_IR 32
#define EXIT_IR 33

#define SS_PIN 21
#define RST_PIN 22

#define VOLTAGE_PIN 34
#define CURRENT_PIN 35

// ================= OBJECTS =================
Servo entryServo;
Servo exitServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 mfrc522(SS_PIN, RST_PIN);

// ================= SYSTEM VARIABLES =================
bool vehicleInSlot = false;
bool chargingActive = false;
bool chargingDone = false;

float voltage = 0;
float current = 0;
float power = 0;
float energy_kWh = 0;
float cost = 0;

unsigned long chargingStartTime = 0;
unsigned long lastSampleTime = 0;

// Demo RFID UID (replace later with real one)
byte allowedUID[4] = {0xA3, 0xB7, 0x1C, 0xD9};

// ================= FUNCTION DECLARATIONS =================
float readVoltage();
float readCurrent();
void resetSystem();

// =========================================================

void setup() {

  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENTRY_IR, INPUT);
  pinMode(EXIT_IR, INPUT);

  digitalWrite(RELAY_PIN, LOW);

  entryServo.attach(ENTRY_SERVO);
  exitServo.attach(EXIT_SERVO);

  entryServo.write(0);
  exitServo.write(0);

  SPI.begin();
  mfrc522.PCD_Init();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("EV Station Ready");
}

// =========================================================

void loop() {

  // ================= ENTRY DETECTION =================
  if (!vehicleInSlot && !chargingActive) {

    if (digitalRead(ENTRY_IR) == LOW) {

      entryServo.write(90);
      delay(2000);
      entryServo.write(0);

      lcd.clear();
      lcd.print("Place Vehicle");
      delay(1000);
    }

    // Ultrasonic confirmation
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;

    if (distance < 20) {
      vehicleInSlot = true;
      lcd.clear();
      lcd.print("Scan RFID");
    }
  }

  // ================= RFID AUTH =================
  if (vehicleInSlot && !chargingActive) {

    if (mfrc522.PICC_IsNewCardPresent() &&
        mfrc522.PICC_ReadCardSerial()) {

      if (memcmp(mfrc522.uid.uidByte, allowedUID, 4) == 0) {

        digitalWrite(RELAY_PIN, HIGH);
        chargingActive = true;
        chargingStartTime = millis();
        lastSampleTime = millis();

        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);

        lcd.clear();
        lcd.print("Charging...");
      }
    }
  }

  // ================= CHARGING =================
  if (chargingActive) {

    voltage = readVoltage();
    current = readCurrent();
    power = voltage * current;

    unsigned long now = millis();
    float dt = (now - lastSampleTime) / 1000.0;
    lastSampleTime = now;

    energy_kWh += (power * dt) / 3600000.0;
    cost = energy_kWh * 1.0;

    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.print(voltage, 1);
    lcd.print(" I:");
    lcd.print(current, 1);

    lcd.setCursor(0, 1);
    lcd.print("E:");
    lcd.print(energy_kWh, 4);
    lcd.print("kWh ");

    // Completion logic
    if ((voltage >= 14.2 && current < 0.2) ||
        energy_kWh >= 0.01 ||
        (millis() - chargingStartTime) > 300000) {

      digitalWrite(RELAY_PIN, LOW);
      chargingActive = false;
      chargingDone = true;

      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);
    }

    delay(1000);
  }

  // ================= EXIT =================
  if (chargingDone) {

    lcd.clear();
    lcd.print("Cost Rs:");
    lcd.print(cost, 3);
    delay(2000);

    if (digitalRead(EXIT_IR) == LOW) {

      exitServo.write(90);
      delay(3000);
      exitServo.write(0);

      resetSystem();
    }
  }
}

// =========================================================
// ================= SENSOR FUNCTIONS ======================
// =========================================================

float readVoltage() {

  int raw = analogRead(VOLTAGE_PIN);
  float adcVoltage = (raw / 4095.0) * 3.3;

  float batteryVoltage = adcVoltage * 7.4;   // calibrated approx

  return batteryVoltage;
}

// ---------------------------------------------------------

float readCurrent() {

  float total = 0;

  for (int i = 0; i < 100; i++) {
    total += analogRead(CURRENT_PIN);
  }

  float avg = total / 100.0;
  float adcVoltage = (avg / 4095.0) * 3.3;

  float zeroOffset = 2.48;
  float currentValue = (adcVoltage - zeroOffset) / 0.066;

  if (currentValue < 0.05) currentValue = 0;

  return currentValue;
}

// ---------------------------------------------------------

void resetSystem() {

  vehicleInSlot = false;
  chargingDone = false;
  energy_kWh = 0;
  cost = 0;

  lcd.clear();
  lcd.print("EV Station Ready");
}