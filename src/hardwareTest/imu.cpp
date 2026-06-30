#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "BMI088.h"

// Deine fest gelöteten Pins
const int CS_ACCEL = 37;
const int CS_GYRO = 36;

// Erstelle das IMU-Objekt. SPI greift automatisch auf Pin 11, 12, 13 zu.
Bmi088 IMU(SPI, CS_ACCEL, CS_GYRO);

// SD-Karten-Logging
char g_logFileName[16] = {0};
File g_logFile;
bool g_logging = false;
unsigned long g_lastFlushMs = 0;
const unsigned long FLUSH_INTERVAL_MS = 1000; // einmal pro Sekunde auf SD durchschreiben

// Sucht den nächsten freien Dateinamen imu_001.csv, imu_002.csv, ...
static bool buildNextLogFileName(char *out, size_t outLen) {
  for (int i = 1; i < 1000; ++i) {
    snprintf(out, outLen, "imu_%03d.csv", i);
    if (!SD.exists(out)) return true;
  }
  return false;
}

static void startLogging() {
  if (g_logging) {
    Serial.println("Logging laeuft bereits.");
    return;
  }
  if (!buildNextLogFileName(g_logFileName, sizeof(g_logFileName))) {
    Serial.println("Keine freien Dateinamen mehr.");
    return;
  }
  g_logFile = SD.open(g_logFileName, FILE_WRITE);
  if (!g_logFile) {
    Serial.print("Konnte Datei nicht oeffnen: ");
    Serial.println(g_logFileName);
    return;
  }
  g_logFile.println("timestamp_us,ax_mss,ay_mss,az_mss,gx_rads,gy_rads,gz_rads");
  g_logFile.flush();
  g_logging = true;
  g_lastFlushMs = millis();
  Serial.print("Logging gestartet -> ");
  Serial.println(g_logFileName);
}

static void stopLogging() {
  if (!g_logging) {
    Serial.println("Logging laeuft nicht.");
    return;
  }
  g_logFile.flush();
  g_logFile.close();
  g_logging = false;
  Serial.print("Logging gestoppt. Datei: ");
  Serial.println(g_logFileName);
}

static void listFiles() {
  File root = SD.open("/");
  if (!root) {
    Serial.println("Kann Root nicht oeffnen.");
    return;
  }
  Serial.println("--- Dateien auf SD ---");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
    } else {
      Serial.print("  ");
      Serial.print(entry.size());
      Serial.println(" bytes");
    }
    entry.close();
  }
  root.close();
  Serial.println("----------------------");
}

static void readFile(const char *name) {
  if (!SD.exists(name)) {
    Serial.print("Datei nicht gefunden: ");
    Serial.println(name);
    return;
  }
  // Während des Lesens kurz pausieren, damit nicht parallel geschrieben wird
  bool wasLogging = g_logging;
  if (wasLogging) {
    g_logFile.flush();
  }
  File f = SD.open(name, FILE_READ);
  if (!f) {
    Serial.println("Open zum Lesen fehlgeschlagen.");
    return;
  }
  Serial.print("---BEGIN ");
  Serial.print(name);
  Serial.println(" ---");
  while (f.available()) {
    // blockweise lesen ist deutlich schneller als byteweise
    uint8_t buf[64];
    int n = f.read(buf, sizeof(buf));
    Serial.write(buf, n);
  }
  f.close();
  Serial.print("---END ");
  Serial.print(name);
  Serial.println(" ---");
  (void)wasLogging;
}

static void deleteFile(const char *name) {
  if (g_logging && strcmp(name, g_logFileName) == 0) {
    Serial.println("Aktive Log-Datei kann nicht geloescht werden. Erst STOP.");
    return;
  }
  if (!SD.exists(name)) {
    Serial.print("Datei nicht gefunden: ");
    Serial.println(name);
    return;
  }
  if (SD.remove(name)) {
    Serial.print("Geloescht: ");
    Serial.println(name);
  } else {
    Serial.println("Loeschen fehlgeschlagen.");
  }
}

static void printHelp() {
  Serial.println("Befehle:");
  Serial.println("  START         - neues Log starten");
  Serial.println("  STOP          - aktuelles Log schliessen");
  Serial.println("  LIST          - Dateien auflisten");
  Serial.println("  READ <name>   - Datei ueber Serial ausgeben");
  Serial.println("  DEL  <name>   - Datei loeschen");
  Serial.println("  HELP          - diese Hilfe");
}

static void handleSerialCommand() {
  static char buf[48];
  static size_t idx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n' || idx >= sizeof(buf) - 1) {
      buf[idx] = 0;
      idx = 0;
      if (strlen(buf) == 0) return;
      if (strcmp(buf, "START") == 0) startLogging();
      else if (strcmp(buf, "STOP") == 0) stopLogging();
      else if (strcmp(buf, "LIST") == 0) listFiles();
      else if (strcmp(buf, "HELP") == 0) printHelp();
      else if (strncmp(buf, "READ ", 5) == 0) readFile(buf + 5);
      else if (strncmp(buf, "DEL ", 4) == 0) deleteFile(buf + 4);
      else {
        Serial.print("Unbekannter Befehl: ");
        Serial.println(buf);
        printHelp();
      }
      return;
    }
    buf[idx++] = c;
  }
}

void setup() {
  Serial.begin(115200);

  // 1. WICHTIG: Erst warten, bis die Spannungsregler auf dem PCB zu 100% stabil sind!
  delay(200);

  // 2. SPI-Pins konfigurieren und Bus starten
  SPI.begin();

  Serial.println("Initialisiere BMI088 (SPI0)...");

  // 3. Dem BMI088 aktiv sagen, dass er im SPI-Modus arbeiten soll.
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
    Serial.println("-> Software-Timing-Fehler ausgeschlossen. Bitte Hardware pruefen.");
    while(1) {
      // Loop-Stopp bei Fehler
    }
  }

  Serial.println("BMI088 erfolgreich erkannt!");

  // 6. SD-Karte initialisieren (eingebauter Slot des Teensy 4.1)
  Serial.print("Initialisiere SD-Karte (BUILTIN_SDCARD)...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(" FEHLGESCHLAGEN");
    Serial.println("-> Keine Karte erkannt oder Karte nicht FAT32-formatiert.");
  } else {
    Serial.println(" OK");
  }

  Serial.println("Logging ist AUS. Mit 'START' im Serial-Monitor beginnen.");
  printHelp();
}

void loop() {
  // Serial-Kommandos zuerst abarbeiten (LIST, READ, STOP, ...)
  handleSerialCommand();

  // Sensordaten abrufen
  IMU.readSensor();

  float ax = IMU.getAccelX_mss();
  float ay = IMU.getAccelY_mss();
  float az = IMU.getAccelZ_mss();
  float gx = IMU.getGyroX_rads();
  float gy = IMU.getGyroY_rads();
  float gz = IMU.getGyroZ_rads();

  // Auf SD schreiben, falls aktiv
  if (g_logging) {
    g_logFile.printf("%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n",
                     micros(), ax, ay, az, gx, gy, gz);
    // Periodisch flushen, damit bei Stromausfall wenig verloren geht
    if (millis() - g_lastFlushMs >= FLUSH_INTERVAL_MS) {
      g_logFile.flush();
      g_lastFlushMs = millis();
    }
  }

  // Serial-Ausgabe (wie vorher, zum Mitlesen)
  // Serial.print("accel ");
  // Serial.print(ax); Serial.print(",");
  // Serial.print(ay); Serial.print(",");
  // Serial.print(az);
  // Serial.print("   gyro ");
  // Serial.print(gx); Serial.print(",");
  // Serial.print(gy); Serial.print(",");
  // Serial.print(gz);
  // Serial.println();

  delay(20); // 50 Hz Abtastrate
}
