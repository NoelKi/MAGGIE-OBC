#include <Arduino.h>

// Pins für den L298N (Logik-Eingänge)
const int IN1_PIN = 22;
const int IN2_PIN = 15;

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Motor dauerhaft AN, eine Richtung (Vollgas):
  // IN1 = HIGH, IN2 = LOW  -> Motor dreht vorwärts.
  // (ENA-Jumper muss gesteckt sein!)
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  Serial.println("Motor sollte jetzt laufen (IN1=HIGH, IN2=LOW)");
}

void loop() {
  // Nichts zu tun - der Motor bleibt durchgehend an.
  Serial.println("Motor an...");
  delay(1000);
}
