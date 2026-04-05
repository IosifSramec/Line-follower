const uint8_t LEFT_PWM  = 9;   
const uint8_t LEFT_IN1  = 2;   
const uint8_t LEFT_IN2  = 4;   

const uint8_t RIGHT_PWM = 10;  
const uint8_t RIGHT_IN1 = 7;   
const uint8_t RIGHT_IN2 = 8;   
int SENSOR_PINS[5] = {A0, A1, A2, A3, A4};

const uint8_t CLP = 5; 
const uint8_t NEAR=6;

int16_t ultimaEroareKunoscuta = 0;
int16_t eroareaAnterioara = 0;
const int VITEZA_BAZA = 80; 
 // valoare mică
int sumaErori = 0;

const float Kp = 0.5; //0.3
const float Ki = 0.01;
const float Kd = 20;

int calculeazaEroareMemorie() {
  int stare = 0;
  // Citim senzorii (Presupunem HIGH pe negru)
  if (digitalRead(SENSOR_PINS[0]) == LOW) stare |= 0b10000;
  if (digitalRead(SENSOR_PINS[1]) == LOW) stare |= 0b01000;
  if (digitalRead(SENSOR_PINS[2]) == LOW) stare |= 0b00100;
  if (digitalRead(SENSOR_PINS[3]) == LOW) stare |= 0b00010;
  if (digitalRead(SENSOR_PINS[4]) == LOW) stare |= 0b00001;
  int eroareCurenta = 0;
  bool linieDetectata = (stare != 0b00000);

  if (!linieDetectata) {
    if (ultimaEroareKunoscuta < 0) return -250;
    if (ultimaEroareKunoscuta > 0) return 250;
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
    default:      eroareCurenta = ultimaEroareKunoscuta; break;
  }

  ultimaEroareKunoscuta = eroareCurenta;
  return eroareCurenta;
}

void mutaMotoare(int vStanga, int vDreapta) {
  // Debug viteze calculate (opțional, ocupă mult spațiu în Serial)
  // Serial.print(" L:"); Serial.print(vStanga); Serial.print(" R:"); Serial.println(vDreapta);

  vStanga = constrain(vStanga, -255, 255);
  vDreapta = constrain(vDreapta, -255, 255);

  // Motor STÂNGA - Direcție și PWM
  if (vStanga >= 0) {
    digitalWrite(LEFT_IN1, LOW); 
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_PWM, vStanga);
  } else {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, abs(vStanga));
  }

  // Motor DREAPTA - Direcție și PWM
  if (vDreapta >= 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, vDreapta);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_PWM, abs(vDreapta));
  }
}

void setup() {
  Serial.begin(9600); // Pornim comunicarea serială
  Serial.println("--- ROBOT LINE FOLLOWER READY ---");

  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  for(int i=0; i<5; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(CLP, INPUT_PULLUP);
  pinMode(NEAR, INPUT_PULLUP);
  
  Serial.println("Astept apasare buton CLP...");
  while(digitalRead(CLP) == LOW) {
    // Putem vedea starea senzorilor chiar înainte de start pentru calibrare
    // Serial.println(digitalRead(SENSOR_PINS[2])); 
    delay(10);
  }
  
  Serial.println("START!");
  delay(500);
}

void loop() {
  int eroare = calculeazaEroareMemorie();
  int derivata = eroare - eroareaAnterioara;
  sumaErori =(sumaErori*0.8)+eroare;
  sumaErori = constrain(sumaErori, -1000, 1000); // anti-windup
  int corectie = (eroare * Kp) + (derivata * Kd) + (sumaErori * Ki);
  eroareaAnterioara = eroare;
  // DEBUG SERIAL: Afișăm eroarea curentă
  //Serial.print("Eroare: "); Serial.print(eroare);
  //Serial.print(" | Corectie: "); Serial.println(corectie);
  //Serial.print(" | NEAR: "); Serial.println(corectie);
  mutaMotoare(VITEZA_BAZA -corectie, VITEZA_BAZA + corectie);
  //delay(10);
  // Mic delay ca să nu umplem buffer-ul serial prea repede (doar pentru debug)
  // delay(50); 
}
