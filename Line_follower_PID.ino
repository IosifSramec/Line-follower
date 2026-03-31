#include <Arduino.h>

// ==========================================
// CONFIGURARE PINI (Conform conexiunilor tale)
// ==========================================
const uint8_t SENSOR_PINS[5] = {A0, A1, A2, A3, A4}; 

// Motor STÂNGA (Conectat la ENA, IN1, IN2)
const uint8_t LEFT_PWM  = 9;   
const uint8_t LEFT_IN1  = 2;   
const uint8_t LEFT_IN2  = 4;   

// Motor DREAPTA (Conectat la ENB, IN3, IN4)
const uint8_t RIGHT_PWM = 10;  
const uint8_t RIGHT_IN1 = 7;   
const uint8_t RIGHT_IN2 = 8;   

// =========================
// CONFIG SENZORI
// =========================
const bool LINE_IS_DARK = true;     
const int SENSOR_COUNT = 5;

// Greutati: linia la stanga => eroare negativa, la dreapta => pozitiva
const int16_t WEIGHTS[SENSOR_COUNT] = {-2000, -1000, 0, 1000, 2000};
const uint16_t LOST_LINE_SUM_THRESHOLD = 180;

// =========================
// CONFIG CONTROL (PID)
// =========================
float Kp = -1.5f;    // Poti creste usor daca e prea "moale"
float Ki = 0.0005f;  
float Kd = 0.7f;    // Poti creste daca tremura

const int MAX_BASE_PWM = 180; // Viteza maxima pe linie dreapta
const int MIN_BASE_PWM = 90;  // Viteza minima in curbe stranse
const int SEARCH_PWM = 100;   // Viteza de cautare cand pierde linia
const long INTEGRAL_LIMIT = 10000;

// =========================
// VARIABILE GLOBALE
// =========================
uint16_t sensorMin[SENSOR_COUNT];
uint16_t sensorMax[SENSOR_COUNT];
long integralTerm = 0;
float dFiltered = 0.0f;
int lastError = 0;
int lastSeenDirection = 1; 

// =========================
// FUNCTII AJUTATOARE
// =========================
int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void initCalibrationArrays() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
}

void calibrateSensors(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      int raw = analogRead(SENSOR_PINS[i]);
      if (raw < sensorMin[i]) sensorMin[i] = raw;
      if (raw > sensorMax[i]) sensorMax[i] = raw;
    }
  }
}

uint16_t readNormalizedSensor(uint8_t index) {
  int raw = analogRead(SENSOR_PINS[index]);
  int minV = sensorMin[index];
  int maxV = sensorMax[index];
  if (maxV <= minV + 5) return 0;
  long value = (long)(raw - minV) * 1000L / (maxV - minV);
  value = clampInt((int)value, 0, 1000);
  if (LINE_IS_DARK) value = 1000 - value;
  return (uint16_t)value;
}

bool readLineError(int &errorOut) {
  long weightedSum = 0;
  long sum = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    uint16_t v = readNormalizedSensor(i);
    weightedSum += (long)v * WEIGHTS[i];
    sum += v;
  }
  if (sum < LOST_LINE_SUM_THRESHOLD) {
    errorOut = lastSeenDirection * 2500;
    return false;
  }
  errorOut = (int)(weightedSum / sum);
  if (errorOut > 100) lastSeenDirection = 1;
  else if (errorOut < -100) lastSeenDirection = -1;
  return true;
}

int computeBaseSpeed(int error) {
  int absError = abs(error);
  long base = MAX_BASE_PWM - (long)(MAX_BASE_PWM - MIN_BASE_PWM) * absError / 2000L;
  return clampInt((int)base, MIN_BASE_PWM, MAX_BASE_PWM);
}

// Adaptat special pentru directia IN1-IN4 a robotului tau
void setMotor(int pwmPin, int in1, int in2, int speedValue, bool isRight) {
  speedValue = clampInt(speedValue, -255, 255);
  
  // Logică inversată pentru motorul drept conform hardware-ului tau
  if (isRight) {
      if (speedValue >= 0) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
        analogWrite(pwmPin, speedValue);
      } else {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
        analogWrite(pwmPin, -speedValue);
      }
  } else { // Motor stang
      if (speedValue >= 0) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
        analogWrite(pwmPin, speedValue);
      } else {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
        analogWrite(pwmPin, -speedValue);
      }
  }
}

void driveMotors(int leftSpeed, int rightSpeed) {
  setMotor(LEFT_PWM,  LEFT_IN1,  LEFT_IN2,  leftSpeed,  false);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, rightSpeed, true);
}

// =========================
// SETUP & LOOP
// =========================
void setup() {
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  for (int i = 0; i < SENSOR_COUNT; i++) pinMode(SENSOR_PINS[i], INPUT);

  Serial.begin(115200);
  initCalibrationArrays();
  
  // CALIBRARE: Misca robotul stanga-dreapta peste linie in aceste 3 secunde!
  calibrateSensors(3000); 
}

void loop() {
  int error = 0;
  bool onLine = readLineError(error);

  if (!onLine) {
    integralTerm = 0;
    // Cauta linia rotindu-se pe loc
    if (lastSeenDirection >= 0) driveMotors(SEARCH_PWM, -SEARCH_PWM);
    else driveMotors(-SEARCH_PWM, SEARCH_PWM);
    return;
  }

  if (abs(error) < 800) integralTerm += error;
  else integralTerm = (integralTerm * 80L) / 100L;

  integralTerm = constrain(integralTerm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  int derivative = error - lastError;
  dFiltered = 0.6f * dFiltered + 0.4f * derivative;

  float correction = Kp * error + Ki * integralTerm + Kd * dFiltered;
  int baseSpeed = computeBaseSpeed(error);

  driveMotors(baseSpeed + (int)correction, baseSpeed - (int)correction);
  lastError = error;
  
  delay(1); // Mică pauză pentru stabilitate
}