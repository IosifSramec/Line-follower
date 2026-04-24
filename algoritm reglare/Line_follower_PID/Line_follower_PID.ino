#include <Arduino.h>
#include <QTRSensors.h>

// --- Pini Motoare ---
const uint8_t LEFT_PWM = 9, LEFT_IN1 = 2, LEFT_IN2 = 4;
const uint8_t RIGHT_PWM = 10, RIGHT_IN1 = 7, RIGHT_IN2 = 8;

// --- Parametri PID ---
const int32_t Kp1000 = 95;   // Reactie la eroare
const int32_t Kd1000 = 1200; // Anticipare (amortizare)
const int32_t Ki1000 = 1;

// --- Parametri Viteză Variabilă ---
const int VITEZA_MAXIMA      = 220; // Viteza pe linie dreapta (cand derivata e mica)
const int VITEZA_MINIMA      = 130; // Limita inferioara ca sa nu se opreasca de tot
const int K_VITEZA_DERIVATA  = 20; 
const int K_VITEZA_EROARE = 2; // Constanta de franare (mai mare = franeaza mai tare la derivata mare)
const int VITEZA_CAUTARE     = 180; 

// --- Variabile Control ---
int32_t eroareaAnterioara = 0;
int32_t ultimaEroare      = 0;
int32_t sumaErori         = 0;
bool eraPierduta          = false;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

bool liniepierduta() {
  for (uint8_t i = 0; i < SensorCount; i++) if (sensorValues[i] > 400) return false;
  return true;
}

void mutaMotoare(int vStanga, int vDreapta) {
  vStanga = constrain(vStanga, -255, 255);
  vDreapta = constrain(vDreapta, -255, 255);
  digitalWrite(LEFT_IN1, vStanga >= 0 ? LOW : HIGH);
  digitalWrite(LEFT_IN2, vStanga >= 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM, abs(vStanga));
  digitalWrite(RIGHT_IN1, vDreapta >= 0 ? HIGH : LOW);
  digitalWrite(RIGHT_IN2, vDreapta >= 0 ? LOW : HIGH);
  analogWrite(RIGHT_PWM, abs(vDreapta));
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint16_t pozitie = qtr.readLineBlack(sensorValues);
  
  // 1. Calculăm eroarea brută și cea ajustată (boost pe margini)
  int32_t eroareaRaw = (int32_t)pozitie - 3500;
  int32_t eroareaAjustata = eroareaRaw;
  
  if (abs(eroareaRaw) > 1800) {
    eroareaAjustata = (eroareaRaw > 0) ? (eroareaRaw + (eroareaRaw - 1800) * 2) 
                                      : (eroareaRaw + (eroareaRaw + 1800) * 2);
  }

  // 2. Logică Căutare (dacă linia este pierdută)
  if (liniepierduta()) {
    eraPierduta = true;
    if (ultimaEroare < 0) mutaMotoare(-VITEZA_CAUTARE, VITEZA_CAUTARE); // Rotire stânga
    else                  mutaMotoare(VITEZA_CAUTARE, -VITEZA_CAUTARE); // Rotire dreapta
    return;
  }

  // Resetare stare după regăsirea liniei
  if (eraPierduta) {
    sumaErori = 0;
    eroareaAnterioara = eroareaAjustata;
    eraPierduta = false;
  }

  // 3. Calcul PID
  int32_t derivata = eroareaAjustata - eroareaAnterioara;
  sumaErori = constrain(sumaErori + eroareaAjustata, -10000, 10000);
  ultimaEroare = eroareaAjustata;

  int32_t corectie = (eroareaAjustata * Kp1000 + derivata * Kd1000 + sumaErori * Ki1000) / 1000;

  // 4. VITEZĂ VARIABILĂ (Frânare adaptivă)
  // Calculăm cât scădem din viteza maximă bazat pe agresivitatea virajului
  int32_t absEroare = abs(eroareaRaw);
  int32_t absDerivata = abs(derivata);
  
  // Factorul de scădere: (Derivata * K + Eroare * K) / 10
  int32_t scadereViteza = ((absDerivata * K_VITEZA_DERIVATA) + (absEroare * K_VITEZA_EROARE)) / 10;
  
  int vitezacurenta = VITEZA_MAXIMA - (int)scadereViteza;
  
  // Ne asigurăm că robotul nu scade sub viteza minimă de tracțiune
  if (vitezacurenta < VITEZA_MINIMA) vitezacurenta = VITEZA_MINIMA;

  // 5. Aplicare comenzi motoare
  mutaMotoare(vitezacurenta + corectie, vitezacurenta - corectie);

  // Salvare eroare pentru iterația următoare
  eroareaAnterioara = eroareaAjustata;
}