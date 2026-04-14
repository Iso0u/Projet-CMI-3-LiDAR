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

const int   STEPS_PER_REV = 200;
// Rapport de transmission courroie : poulie moteur (29 mm) / pièce rotative (74 mm)
// Pour 1 tour moteur, la pièce rotative tourne de 29/74 de tour.
const float GEAR_RATIO    = 29.0f / 74.0f;

// Vitesse moteur (pas/s)
const float SPEED_MIN  =  50.0f;
const float SPEED_MAX  = 200.0f;
const float SPEED_STEP =  50.0f;
float motorSpeed       = 100.0f;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// volatile : partagée entre loop() (Core 1) et stepperTask (Core 0)
volatile bool running = false;

// ── Tâche FreeRTOS dédiée au moteur (Core 0) ─────────────────────────────────
// Isolée du stack Bluetooth Classic (Core 1) : la connexion BT ne peut plus
// bloquer runSpeed() et faire caler le moteur.
void stepperTask(void* pvParameters) {
  for (;;) {
    if (running) stepper.runSpeed();
    vTaskDelay(1);   // 1 ms — bien en-deçà de l'intervalle à 200 pas/s (5 ms)
  }
}

void initMotor() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(motorSpeed);
}

float computeAngle() {
  // Pas du moteur → angle de la pièce rotative (après réduction par courroie)
  float motor_angle = (float)stepper.currentPosition() * (2.0f * PI / STEPS_PER_REV);
  float piece_angle = fmod(motor_angle * GEAR_RATIO, 2.0f * PI);
  if (piece_angle < 0) piece_angle += 2.0f * PI;
  return piece_angle;
}

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
  // SerialBT.println supprimé : l'appli mobile n'a pas besoin des données
  // capteur, et cet envoi 20x/s saturait le buffer BT et bloquait loop().
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);   // évite 1 s de blocage sur octet parasite USB
  SerialBT.begin("ESP32_Sharp");
  Serial.println("Sharp GP2Y0A21YK0F Distance Sensor");
  Serial.println("Range: 10-80 cm");
  Serial.println("-----------------------------------");

  initMotor();

  // Lancement de la tâche moteur sur Core 0 (BT tourne sur Core 1)
  xTaskCreatePinnedToCore(
    stepperTask,    // fonction
    "StepperTask",  // nom
    2048,           // stack (mots)
    NULL,           // params
    3,              // priorité (> loop Arduino = 1)
    NULL,           // handle
    0               // Core 0
  );
}

void loop() {
  // ── Commandes Python (USB Serial) ────────────────────────────────────────
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

  // ── Commandes Bluetooth (appli mobile) ───────────────────────────────────
  // "if" et non "while" : 1 commande max par itération pour ne pas bloquer.
  bool speedChanged = false;
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == 'P') {
      motorSpeed = constrain(motorSpeed + SPEED_STEP, SPEED_MIN, SPEED_MAX);
      stepper.setSpeed(motorSpeed);
      speedChanged = true;
    } else if (c == 'M') {
      motorSpeed = constrain(motorSpeed - SPEED_STEP, SPEED_MIN, SPEED_MAX);
      stepper.setSpeed(motorSpeed);
      speedChanged = true;
    }
  }

  if (!running) return;

  // ── Acquisition capteur ──────────────────────────────────────────────────
  // Le moteur tourne via stepperTask (Core 0), indépendamment de ce bloc.
  unsigned long now = millis();
  if (now - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = now;

    sampleSharp();
    float angle_rad = computeAngle();
    float angle_deg = angle_rad * 180.0 / PI;
    while (angle_deg < 0.0f)    angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    distance_cm = computeDistance(sensorValue);
    sendData(angle_deg, distance_cm * 10.0f);
    updateLED(distance_cm);
  }

  // ── Confirmation vitesse différée ────────────────────────────────────────
  // Envoyée après l'acquisition, uniquement si le buffer BT a de la place.
  if (speedChanged && SerialBT.availableForWrite() > 20) {
    SerialBT.println("[SPEED] " + String((int)motorSpeed) + " pas/s");
  }
}

