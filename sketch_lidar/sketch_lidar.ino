#include "BluetoothSerial.h"
#include <AccelStepper.h>

BluetoothSerial SerialBT;

// Pins
const int STEP_PIN = 27;
const int DIR_PIN  = 14;
const int EN_PIN   = 12;

const int analogInPin = A0;
const int analogOutPin = 9;

int   sensorValue  = 0;
int   outputValue  = 0;
float distance_cm  = 0.0;

const float VOLTAGE_REF  = 5.0;
const int   NUM_SAMPLES  = 5;

// Intervalle entre deux mesures complètes (angle + distance envoyés)
const unsigned long SENSOR_INTERVAL = 50;
unsigned long lastSensorTime = 0;

const int STEPS_PER_REV = 200;

// Vitesse moteur (pas/s)
const float SPEED_MIN  =  50.0f;   // Limite basse
const float SPEED_MAX  = 200.0f;   // Limite haute
const float SPEED_STEP =  50.0f;   // Incrément par commande P/M
float motorSpeed       =  100.0f;   // Vitesse courante (modifiable via Bluetooth)

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool running = false;   // Démarre uniquement sur commande "START" de Python

void initMotor() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(motorSpeed);
}

void runMotor() {
  stepper.runSpeed();
}

float computeAngle() {
  long steps = stepper.currentPosition() % STEPS_PER_REV;
  if (steps < 0) steps += STEPS_PER_REV;
  return (float)steps * (2.0 * PI / STEPS_PER_REV);
}

// Moyenne ADC en rafale : le moteur bouge peu pendant les ~1 ms d'échantillonnage
void sampleSharp() {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(analogInPin);
    delayMicroseconds(150);
  }
  sensorValue = (int)(sum / NUM_SAMPLES);
}

float computeDistance(int rawValue) {
  float voltage = rawValue * (VOLTAGE_REF / 1023.0);
  float dist    = 80.0;

  if (voltage > 0.6) {
    dist = (27.86 / (voltage - 0.42)) * 4;
    dist = constrain(dist, 10.0, 80.0);
  }
  return dist;
}

void updateLED(float dist) {
  outputValue = map((int)dist, 10, 80, 255, 0);
  outputValue = constrain(outputValue, 0, 255);
  analogWrite(analogOutPin, outputValue);
}

void sendData(float angle_deg, float dist_mm) {
  String msg = String(angle_deg, 2) + ";" + String(dist_mm, 1);
  SerialBT.println(msg);
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Sharp");
  Serial.println("Sharp GP2Y0A21YK0F Distance Sensor");
  Serial.println("Range: 10-80 cm");
  Serial.println("-----------------------------------");

  initMotor();
}

void loop() {
  // ── Lecture des commandes depuis Python ──────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "START") {
      running = true;
      stepper.setCurrentPosition(0);   // Position physique courante = angle 0°
      stepper.enableOutputs();
      stepper.setSpeed(motorSpeed);
      Serial.println("[OK] Démarrage");
    } else if (cmd == "STOP") {
      running = false;
      stepper.stop();
      stepper.disableOutputs();
      Serial.println("[OK] Arrêt");
    }
  }

  // ── Commandes Bluetooth (application mobile) ─────────────────────────────
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if ((c == 'P') && (motorSpeed != SPEED_MAX)) {
      motorSpeed = constrain(motorSpeed + SPEED_STEP, SPEED_MIN, SPEED_MAX);
      stepper.setSpeed(motorSpeed);
      SerialBT.println("[SPEED] " + String((int)motorSpeed) + " pas/s");
    } else if ((c == 'M') && (motorSpeed != SPEED_MIN)) {
        motorSpeed = constrain(motorSpeed - SPEED_STEP, SPEED_MIN, SPEED_MAX);
        stepper.setSpeed(motorSpeed);
        SerialBT.println("[SPEED] " + String((int)motorSpeed) + " pas/s");
    }
  }

  if (!running) return;

  // ── Rotation et acquisition ───────────────────────────────────────────────
  runMotor();

  unsigned long now = millis();
  if (now - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = now;

    sampleSharp();
    float angle_rad = computeAngle();
    float angle_deg = (angle_rad * 180.0 / PI);
    while (angle_deg < 0.0f) {
      angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
      angle_deg -= 360.0f;
    }
    distance_cm = computeDistance(sensorValue);
    float dist_mm = distance_cm * 10.0f;
    updateLED(distance_cm);
    sendData(angle_deg, dist_mm);
  }
}

