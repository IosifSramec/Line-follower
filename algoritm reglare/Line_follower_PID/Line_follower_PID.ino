#include <Arduino.h>
// Definirea pinilor pe porturi (pentru referință)
// LEFT_PWM: Pin 9  -> Port B, Bit 1
// RIGHT_PWM: Pin 10 -> Port B, Bit 2
// LEFT_IN1: Pin 2  -> Port D, Bit 2
// LEFT_IN2: Pin 4  -> Port D, Bit 4
// RIGHT_IN1: Pin 7 -> Port D, Bit 7
// RIGHT_IN2: Pin 8 -> Port B, Bit 0
// SENZORI: A0-A4   -> Port C, Biții 0-4

const uint8_t LEFT_PWM  = 9;   
const uint8_t LEFT_IN1  = 2;   
const uint8_t LEFT_IN2  = 4;   

const uint8_t RIGHT_PWM = 10;  
const uint8_t RIGHT_IN1 = 7;   
const uint8_t RIGHT_IN2 = 8;   
int SENSOR_PINS[5] = {A0, A1, A2, A3, A4};

const uint8_t CLP = 5; 
const uint8_t NEAR=6;

int16_t ultimaEroareCunoscuta = 0;
int16_t eroareaAnterioara = 0;
const int VITEZA_BAZA = 120; 
 // valoare mică
int sumaErori = 0;

const float Kp = 0.6; 
const float Ki = 0.03;
const float Kd = 30;

inline int calculeazaEroareMemorie() {
  uint8_t stare = (~PINC) & 0b00011111;
  int eroareCurenta = 0;
  
  if (stare==0) {
    if (ultimaEroareCunoscuta < 0) return -250;
    if (ultimaEroareCunoscuta > 0) return 250;
    return 0;
  }

  switch (stare) {
    case 0b00100: eroareCurenta = 0;    break;
    case 0b01100: eroareCurenta = -50;  break;
    case 0b00110: eroareCurenta = 50;   break;
    case 0b01000: eroareCurenta = -100; break;
    case 0b00010: eroareCurenta = 100;  break;
    case 0b11000: eroareCurenta = -150; break;
    case 0b00011: eroareCurenta = 150;  break;
    case 0b10000: eroareCurenta = -200; break;
    case 0b00001: eroareCurenta = 200;  break;
    default:      eroareCurenta = ultimaEroareCunoscuta; break;
  }

  ultimaEroareCunoscuta = eroareCurenta;
  return eroareCurenta;
}

inline void mutaMotoare(int vStanga, int vDreapta) {
 
  vStanga = constrain(vStanga, -255, 255);
  vDreapta = constrain(vDreapta, -255, 255);

  // Motor STÂNGA - Direcție și PWM
  if (vStanga >= 0) {
    PORTD &= ~(1 << 2); // Pin 2 LOW
    PORTD |=  (1 << 4); // Pin 4 HIGH
    //analogWrite(LEFT_PWM, vStanga);
  } else {
    PORTD |=  (1 << 2); // Pin 2 HIGH
    PORTD &= ~(1 << 4); // Pin 4 LOW
    //analogWrite(LEFT_PWM, abs(vStanga));
  }
  analogWrite(LEFT_PWM, abs(vStanga));
  // Motor DREAPTA - Direcție și PWM
  if (vDreapta >= 0) {
    PORTD |=  (1 << 7); // Pin 7 HIGH
    PORTB &= ~(1 << 0); // Pin 8 LOW
  } else {
    PORTD &= ~(1 << 7); // Pin 7 LOW
    PORTB |=  (1 << 0); // Pin 8 HIGH
  }
  analogWrite(RIGHT_PWM, abs(vDreapta));
}

void setup() {
  //Serial.begin(9600); // Pornim comunicarea serială
  //Serial.println("--- ROBOT LINE FOLLOWER READY ---");

  DDRD |= (1 << 2) | (1 << 4) | (1 << 7); // Pinii 2, 4, 7
  DDRB |= (1 << 0) | (1 << 1) | (1 << 2); // Pinii 8, 9, 10
  DDRC |= 0b00000;

  pinMode(CLP, INPUT_PULLUP);
  pinMode(NEAR, INPUT_PULLUP);
  
  Serial.println("Astept apasare buton CLP...");
  while(digitalRead(CLP) == LOW) {
    delay(10);
  }
  
  //Serial.println("START!");
  delay(500);
}

void loop() {
  int eroare = calculeazaEroareMemorie();
  int derivata = eroare - eroareaAnterioara;
  sumaErori =(sumaErori*0.8)+eroare;
  sumaErori = constrain(sumaErori, -1000, 1000);

  int corectie = (eroare * Kp) + (derivata * Kd) + (sumaErori * Ki);

  eroareaAnterioara = eroare;
  mutaMotoare(VITEZA_BAZA +corectie, VITEZA_BAZA - corectie);
}