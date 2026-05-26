#include <Arduino.h>
#include "BMI088.h"

// Deine fest gelöteten Pins
const int CS_ACCEL = 37;
const int CS_GYRO = 36;

// Erstelle das IMU-Objekt. SPI greift automatisch auf Pin 11, 12, 13 zu.
Bmi088 IMU(SPI, CS_ACCEL, CS_GYRO);

void setup() {
  Serial.begin(115200);
  
  // 1. WICHTIG: Erst warten, bis die Spannungsregler auf dem PCB zu 100% stabil sind!
  delay(2000); 

  // 2. SPI-Pins konfigurieren und Bus starten
  SPI.begin();
  
  Serial.println("Initialisiere BMI088 (SPI0)...");

  // 3. Dem BMI088 aktiv sagen, dass er im SPI-Modus arbeiten soll.
  // Dazu ziehen wir beide Chip-Select-Pins kurz manuell HIGH und wieder LOW.
  // Das triggert das Protokoll-Interface des Chips, falls der PS-Pin zickt.
  pinMode(CS_ACCEL, OUTPUT);
  pinMode(CS_GYRO, OUTPUT);
  digitalWrite(CS_ACCEL, HIGH);
  digitalWrite(CS_GYRO, HIGH);
  delay(10);
  
  // 4. Test-Transaktion mit 1 MHz und MODE 3 (Standard für viele Bosch-Sensoren)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  delay(10);

  // 5. Jetzt den Bibliotheks-Start aufrufen
  int status = IMU.begin();
  
  SPI.endTransaction();

  if (status < 0) {
    Serial.print("Immer noch Verbindungsfehler! Code: ");
    Serial.println(status);
    Serial.println("-> Software-Timing-Fehler ausgeschlossen. Bitte Hardware prüfen.");
    while(1) {
      // Loop-Stopp bei Fehler
    }
  }
  
  Serial.println("BMI088 erfolgreich erkannt!");
}

void loop() {
  // Sensordaten abrufen
  IMU.readSensor();

  // Ausgabe für den Serial Plotter (Beschleunigung in m/s² und Rotation in rad/s)
  //Format: AccelX, AccelY, AccelZ
  Serial.print("accellllllll");
  Serial.print(IMU.getAccelX_mss());
  Serial.print(",");
  Serial.print(IMU.getAccelY_mss());
  Serial.print(",");
  Serial.print(IMU.getAccelZ_mss());
  
  // Wenn du das Gyroskop sehen willst, kommentiere die oberen Zeilen aus und die unteren ein:
  Serial.print("      gyroooooooooo");
  Serial.print(IMU.getGyroX_rads());
  Serial.print(",");
  Serial.print(IMU.getGyroY_rads());
  Serial.print(",");
  Serial.print(IMU.getGyroZ_rads());
  

  Serial.println(); // Zeilenumbruch für den Plotter

  delay(2000); // 50 Hz Abtastrate – perfekt für eine flüssige Kurve im Serial Plotter
}
