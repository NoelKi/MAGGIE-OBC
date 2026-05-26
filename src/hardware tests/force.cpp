#include <Arduino.h>
#include "HX711.h"

// Pin-Definitionen für deinen Schaltplan
const int HX711_DOUT_PIN = 3;
const int HX711_SCK_PIN = 27;

// Objekt erstellen
HX711 scale;

// Dein ermittelter Kalibrierfaktor (Wert muss im Labor angepasst werden)
// Ein positiver Wert bedeutet meistens, dass Druck/Zug in die richtige Richtung zählt.
float calibration_factor = 26.0; 

void setup() {
  Serial.begin(115200);
  // Kurz warten, bis der Serielle Monitor bereit ist (wichtig beim Teensy)
  while (!Serial && millis() < 4000); 
  
  Serial.println("HX711 - Initialisierung...");

  // Sensor mit den Pins 3 (Data) und 27 (Clock) starten
  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);

  // WICHTIG: Beim Systemstart sollte die Wägezelle komplett unbelastet sein!
  Serial.println("Nullabgleich (Tara) läuft... Bitte Zelle nicht belasten.");
  scale.tare(); 
  Serial.println("Tara fertig.");

  // Kalibrierfaktor anwenden
  scale.set_scale(calibration_factor);       
  
  Serial.println("Sensor ist bereit für Messungen!");
  Serial.println("Tippe 't' in den Monitor für ein erneutes Trieren.");
}

void loop() {
  // scale.is_ready() prüft, ob der HX711 ein neues Sample fertig konvertiert hat
  if (scale.is_ready()) {
    // get_units(1) liest genau einen Wert aus und rechnet den Kalibrierfaktor ein
    float weight = scale.get_units(1);
    
    Serial.print("Messwert: ");
    Serial.print(weight, 2);
    Serial.println(" g"); // oder kg, je nachdem wie du kalibriert hast
  } else {
    // Falls der Chip noch nicht so weit ist (Schnittstelle läuft auf 10Hz oder 80Hz)
    delay(100); 
  }

  // Manuelles Nullen über die serielle Konsole im Labor
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 't' || ch == 'T') {
      Serial.println("Setze Gewicht auf Null...");
      scale.tare();
      Serial.println("Neu genullt.");
    }
  }
}