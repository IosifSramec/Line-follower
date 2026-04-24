#include <Arduino.h>
#include <QTRSensors.h>

const uint8_t LEFT_PWM  = 9;
const uint8_t LEFT_IN1  = 2;
const uint8_t LEFT_IN2  = 4;
const uint8_t RIGHT_PWM = 10;
const uint8_t RIGHT_IN1 = 7;
const uint8_t RIGHT_IN2 = 8;

// PID x1000 — fara float
const int32_t Kp1000 = 1200;   // Kp = 2.0
const int32_t Ki1000 = 1;      // Ki = 0.001
const int32_t Kd1000 = 50000;  // Kd = 55.0

// Viteza adaptiva x10000 — fara float
const int32_t Kp_v10000 = 80;   // Kp_v = 0.0001
const int32_t Kd_v10000 = 150;  // Kd_v = 0.005

// Atenuare x1000 — fara float
const int32_t K_at1000 = 600;  // K_atenuare = 0.18

const int VITEZA_MAX       = 255;
const int VITEZA_MIN_CURBA = 60;

int16_t eroareaAnterioara = 0;
int32_t sumaErori         = 0;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

inline void mutaMotoare(int vStanga, int vDreapta) {
  if      (vStanga >  255) vStanga =  255;
  else if (vStanga < -150) vStanga = -150;
  else if(vStanga>-100 && vStanga<50) vStanga=0;
  if      (vDreapta >  255) vDreapta =  255;
  else if (vDreapta < -150) vDreapta = -150;
  else if(vDreapta>-100 && vDreapta<50) vDreapta=0;

  if (vStanga >= 0) {
    PORTD &= ~(1 << PD2);
    PORTD |=  (1 << PD4);
  } else {
    PORTD |=  (1 << PD2);
    PORTD &= ~(1 << PD4);
    vStanga = -vStanga;
  }
  analogWrite(LEFT_PWM, vStanga);

  if (vDreapta >= 0) {
    PORTD |=  (1 << PD7);
    PORTB &= ~(1 << PB0);
  } else {
    PORTD &= ~(1 << PD7);
    PORTB |=  (1 << PB0);
    vDreapta = -vDreapta;
  }
  analogWrite(RIGHT_PWM, vDreapta);
}

void setup() {
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Prescaler ADC 64 — citire ~2x mai rapida
  ADCSRA = (ADCSRA & ~0x07) | 0x06;

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint16_t pozitie = qtr.readLineBlack(sensorValues);
  int16_t eroareaCurenta = (int16_t)pozitie - 3500;
  int16_t derivata = eroareaCurenta - eroareaAnterioara;
  int16_t errAbs = abs(eroareaCurenta);
  int16_t derAbs = abs(derivata);

  // Suma erori — 0.8 = 4/5 fara float
  sumaErori = (sumaErori * 4) / 5 + eroareaCurenta;
  sumaErori = constrain(sumaErori, -10000, 10000);

  // Corectie PID — totul x1000, impartim la final
  int32_t corectie = (  (int32_t)eroareaCurenta * Kp1000
                      + (int32_t)derivata       * Kd1000
                      + (int32_t)sumaErori      * Ki1000 ) / 1000;

  // Viteza dinamica — Kp_v si Kd_v x10000
  int32_t reducereVit = (  (int32_t)errAbs * Kp_v10000
                          + (int32_t)derAbs * Kd_v10000 ) / 10000;
  int viteza = (int)(VITEZA_MAX - reducereVit);
  if (viteza < VITEZA_MIN_CURBA) viteza = VITEZA_MIN_CURBA;

  // Atenuare — 1000 / (1000 + K_at1000 * derAbs)
  // echivalent cu 1.0 / (1.0 + K_atenuare * derAbs) dar fara float
  int32_t corectieFinala = (corectie * 1000L) / (1000L + K_at1000 * derAbs);

  mutaMotoare(viteza + (int)corectieFinala, viteza - (int)corectieFinala);
  eroareaAnterioara = eroareaCurenta;
}
