#include <Arduino.h>
#include <QTRSensors.h>

const uint8_t LEFT_PWM  = 9;
const uint8_t LEFT_IN1  = 2;
const uint8_t LEFT_IN2  = 4;
const uint8_t RIGHT_PWM = 10;
const uint8_t RIGHT_IN1 = 7;
const uint8_t RIGHT_IN2 = 8;

const int VITEZA_BAZA = 80;
const int VITEZA_BAZA_DREPT=255;
const float Kp = 2;
const float Ki = 0.05;
const float Kd = 50;


int16_t eroareaAnterioara = 0;
int sumaErori = 0;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

inline void mutaMotoare(int vStanga, int vDreapta) {
  vStanga  = constrain(vStanga,  -100 , 255);
  vDreapta = constrain(vDreapta, -100, 255);

  if (vStanga >= 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
  } else {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  }
  analogWrite(LEFT_PWM, abs(vStanga));

  if (vDreapta >= 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
  }
  analogWrite(RIGHT_PWM, abs(vDreapta));
}

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrare... Misca robotul pe linie!");
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibrare terminata.");
}

void loop() {
  uint16_t pozitie = qtr.readLineBlack(sensorValues);
  int eroareaCurenta = (int)pozitie - 3500;
  
  int derivata = eroareaCurenta - eroareaAnterioara;

  sumaErori = (sumaErori * 0.8) + eroareaCurenta;
  sumaErori = constrain(sumaErori, -10000, 10000);
  int corectie = (eroareaCurenta * Kp) + (derivata * Kd) + (sumaErori * Ki);
  eroareaAnterioara = eroareaCurenta;
  if(abs(derivata<=50))
    mutaMotoare(VITEZA_BAZA_DREPT+corectie*0.25, VITEZA_BAZA_DREPT-corectie*0.25 );
  else
  mutaMotoare(VITEZA_BAZA + corectie, VITEZA_BAZA - corectie);
}