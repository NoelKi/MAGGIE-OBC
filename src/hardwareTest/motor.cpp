#include <Arduino.h>
#include <Encoder.h>

// 1 = Kalibrier-Modus (Encoder von Hand vermessen), 0 = normale Regelung
#define CALIBRATE 1

// Pins für den DRV8871 (H-Brücke, nur 2 Eingänge, kein ENA)
const int IN1_PIN = 18;
const int IN2_PIN = 19;

// Quadratur-Encoder A/B (Teensy nutzt HW-Decoder -> sehr präzise)
const int ENC_A_PIN = 0;
const int ENC_B_PIN = 1;
Encoder motorEnc(ENC_A_PIN, ENC_B_PIN);
long lastEncPos = 0;

// PWM-Konfiguration (Teensy 4.x)
const int PWM_FREQ = 20000;  // 20 kHz -> ausserhalb des Hoerbereichs
const int PWM_RES  = 8;      // 8 Bit -> Werte 0..255

// --- Positionsregelung ---
// WICHTIG: An deinen Encoder anpassen! Counts fuer EINE volle Umdrehung
// (Getriebe eingerechnet). Kalibrieren: Motor genau 1x von Hand drehen und
// den "Encoder = ..."-Wert im Monitor ablesen.
const long COUNTS_PER_REV = 4550;                // gemessen: halbe Umdrehung ~2300
const long HALF_TURN      = COUNTS_PER_REV / 2;  // 180 Grad = ~2300 Counts

const float Kp        = 0.8f;  // Regler-Verstaerkung
const int   MIN_PWM   = 10;    // Losbrech-PWM (Motor muss sich noch bewegen)
const int   MAX_PWM   = 20;   // Begrenzung nach oben
const long  POS_TOL   = 5;     // Zielfenster in Counts

// speed: -255..255  (Vorzeichen = Richtung, Betrag = Drehzahl)
// Beim DRV8871 wird die PWM direkt auf IN1/IN2 gelegt.
void setMotor(int speed) {
  int pwm = constrain(abs(speed), 0, 255);

  if (speed >= 0) {
    analogWrite(IN1_PIN, pwm);   // vorwaerts
    analogWrite(IN2_PIN, 0);
  } else {
    analogWrite(IN1_PIN, 0);
    analogWrite(IN2_PIN, pwm);   // rueckwaerts
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  analogWriteResolution(PWM_RES);
  analogWriteFrequency(IN1_PIN, PWM_FREQ);
  analogWriteFrequency(IN2_PIN, PWM_FREQ);

  setMotor(0);
  motorEnc.write(0);  // Nullpunkt setzen
  Serial.println("DRV8871 PWM-Test + Encoder gestartet");
}

// Encoder-Position ausgeben, sobald sie sich geaendert hat
void reportEncoder() {
  long pos = motorEnc.read();  // Zaehlt in Quadratur-Schritten (4x Auflösung)
  if (pos != lastEncPos) {
    Serial.printf("Encoder = %ld\n", pos);
    lastEncPos = pos;
  }
}

// Warten und dabei den Encoder weiter auslesen
void delayWithEncoder(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    reportEncoder();
  }
}

// Faehrt per P-Regler auf die Ziel-Position und stoppt dort.
void moveTo(long target) {
  while (true) {
    long pos   = motorEnc.read();
    long error = target - pos;

    if (labs(error) <= POS_TOL) break;  // Ziel erreicht

    int pwm = (int)(Kp * labs(error));
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    setMotor(error > 0 ? pwm : -pwm);

    reportEncoder();
  }
  setMotor(0);  // am Ziel anhalten
  Serial.printf("Ziel erreicht: %ld\n", target);
}

// Kalibrierung: Motor aus, Position live ausgeben.
// Welle exakt 1x von Hand drehen -> abgelesener Wert = COUNTS_PER_REV.
// Taste (irgendein Zeichen) im Monitor senden -> Nullpunkt neu setzen.
void calibrateLoop() {
  setMotor(0);  // Motor stromlos, damit man frei drehen kann

  if (Serial.available()) {
    while (Serial.available()) Serial.read();  // Puffer leeren
    motorEnc.write(0);
    lastEncPos = 0;
    Serial.println(">> Nullpunkt gesetzt. Jetzt genau 1 Umdrehung drehen.");
  }

  long pos = motorEnc.read();
  if (pos != lastEncPos) {
    Serial.printf("Position = %ld\n", pos);
    lastEncPos = pos;
  }
}

void loop() {
#if CALIBRATE
  calibrateLoop();
#else
  static long target = 0;

  // Halbe Umdrehung vorwaerts
  target += HALF_TURN;
  moveTo(target);
  delayWithEncoder(500);

  // Halbe Umdrehung zurueck
  target -= HALF_TURN;
  moveTo(target);
  delayWithEncoder(500);
#endif
}
