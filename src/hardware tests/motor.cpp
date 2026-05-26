#include <Arduino.h>

// Pins für den DRV8871
const int AIN1_PIN = 22; 
const int AIN2_PIN = 15; 

// PWM Einstellungen
const int PWM_RESOLUTION = 8; 
const int MAX_SPEED = 255;    // Maximale PWM-Stufe (100% Geschwindigkeit)
const float PWM_FREQUENCY = 20000.0; // 20 kHz gegen Motor-Fiepen

// Vorwärtsdeklaration der Motorfunktionen
void driveMotor(int speed);
void coastMotor();

void setup() {
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(AIN1_PIN, PWM_FREQUENCY);
  analogWriteFrequency(AIN2_PIN, PWM_FREQUENCY);

  coastMotor(); // Sicher starten
}

void loop() {
  // ==========================================
  // I. VORWÄRTS-PHASE
  // ==========================================
  
  // 1. Sanft hochrampen bis Vollgas vorwärts
  for (int speed = 0; speed <= MAX_SPEED; speed += 5) {
    driveMotor(speed);
    delay(20); // 20ms * 51 Schritte = ca. 1 Sekunde Rampe
  }
  
  delay(10000); // 1 Sekunde lang Höchstgeschwindigkeit halten

  // 2. Sanft wieder runterrampen bis zum Stillstand
  for (int speed = MAX_SPEED; speed >= 0; speed -= 5) {
    driveMotor(speed);
    delay(20); // ca. 1 Sekunde Rampe nach unten
  }

  coastMotor(); // Kurz komplett stromlos schalten
  delay(5000);   // 0,5 Sekunden Pause im Scheitelpunkt

  // ==========================================
  // II. RÜCKWÄRTS-PHASE
  // ==========================================
  
  // 3. Sanft hochrampen bis Vollgas rückwärts (negative Werte)
  for (int speed = 0; speed >= -MAX_SPEED; speed -= 5) {
    driveMotor(speed);
    delay(20); 
  }

  delay(1000); // 1 Sekunde lang Höchstgeschwindigkeit rückwärts halten

  // 4. Sanft wieder runterrampen bis zum Stillstand
  for (int speed = -MAX_SPEED; speed <= 0; speed += 5) {
    driveMotor(speed);
    delay(20); 
  }

  coastMotor(); // Am Ende wieder stromlos schalten
  delay(1000);  // 1 Sekunde Pause, bevor der gesamte Loop von vorne startet
}

/**
 * Steuert den Motor basierend auf der Richtung und Geschwindigkeit.
 * @param speed Werte von -255 (Vollgas rückwärts) bis 255 (Vollgas vorwärts).
 */
void driveMotor(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (speed > 0) {
    // Vorwärts: PWM auf AIN1, AIN2 bleibt flach auf Masse
    analogWrite(AIN1_PIN, speed);
    digitalWrite(AIN2_PIN, LOW);
  } 
  else if (speed < 0) {
    // Rückwärts: AIN1 bleibt flach auf Masse, PWM auf AIN2
    digitalWrite(AIN1_PIN, LOW);
    analogWrite(AIN2_PIN, abs(speed)); // Duty Cycle muss positiv sein
  } 
  else {
    coastMotor();
  }
}

/**
 * Schaltet die Brücke stromlos (Auslauf). 
 * Verhindert im Gegensatz zu Hard-Brake (HIGH/HIGH) extreme Spannungsspitzen.
 */
void coastMotor() {
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
}