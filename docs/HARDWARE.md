# Hardware Konfiguration für MAGGIE

## Sensor-Konfiguration

### IMU (Inertial Measurement Unit)
```
Sensor: MPU9250 / ICM-20948 (9-DOF)
Protokoll: I2C
I2C Adresse: 0x68 (Default) oder 0x69 (mit AD0 high)
Pins (Teensy 4.1):
  - SDA: Pin 18 (I2C1_SDA)
  - SCL: Pin 19 (I2C1_SCL)
Maximale Frequenz: 400 kHz
Beschleunigungsmessbereich: ±16g (konfigurierbar)
Drehgeschwindigkeit: ±2000°/s (konfigurierbar)
Sample-Rate: bis 8 kHz
Stromverbrauch: ~3.5 mA
```

### Barometer & Thermometer
```
Sensor: BMP390 / BMP280 (Druck + Temperatur)
Protokoll: I2C
I2C Adresse: 0x77 (Default) oder 0x76 (mit SDO low)
Pins: SDA/SCL same as IMU
Druckbereich: 300-1100 hPa
Temperaturbereich: -40°C to +85°C
Höhenauflösung: ±1 m
Sample-Rate: bis 200 Hz
Stromverbrauch: ~0.5 mA
```

### GPS (optional)
```
Sensor: NEO-6M / NEO-M9N
Protokoll: UART (seriell)
Baudrate: 9600 bps (konfigurierbar)
Pins (Teensy 4.1):
  - RX: Pin 0 (UART1_RX)
  - TX: Pin 1 (UART1_TX)
Positionsgenauigkeit: ±2.5 m
Zeitgenauigkeit: ±100 ns
Zeit bis First-Fix: ~30 Sekunden (kalt), ~5 Sekunden (warm)
Stromverbrauch: ~30-50 mA
Hinweis: Benötigt freien Blick zum Himmel
```

### SD-Karte (Datenspeicher)
```
Interfaz: SPI (Serial Peripheral Interface)
Pins (Teensy 4.1):
  - MOSI: Pin 11
  - MISO: Pin 12
  - SCK:  Pin 13
  - CS:   Pin 10 (konfigurierbar)
Maximum-Frequenz: 25 MHz
Kapazität: min. 4 GB (FAT32 formatiert)
Schreibgeschwindigkeit: 10-20 MB/s
Stromverbrauch: ~10-40 mA während Schreiben
```

## CAN-Bus Konfiguration

```
Interface: CAN/CAN FD
Baudrate: 500 kbps (Standard für Avionik)
Pins (Teensy 4.1):
  - CAN_RX: Pin 23
  - CAN_TX: Pin 22
Protokoll: Extended Frame (29-bit IDs)
Beispiel Nachrichten:
  - 0x100: Telemetrie Primary
  - 0x101: Telemetrie Secondary
  - 0x200: Status/Heartbeat
  - 0x300: Command
```

## Power Supply

```
Spannung: 5V (via USB) oder externe Batterie
Batterietyp (empfohlen): 4S LiPo (14.8V nominal)
Max. Stromaufnahme: ~500 mA (mit allen Sensoren + SD-Schreiben)
Backup-Batterie: optional für RTC

Stromverteilung:
  - Teensy 4.1: 5V, ~100 mA
  - IMU: 3.3V, ~4 mA
  - Barometer: 3.3V, ~1 mA
  - GPS: 5V, ~40 mA
  - SD-Karte: 3.3V, ~25 mA (durchschnittlich)
```

## Pin-Belegung (Teensy 4.1)

```
Analog Inputs (ADC):
  A0 (Pin 14): Battery Voltage Monitor
  A1 (Pin 15): Temperature Sensor (optional)

Digital I/O:
  Pin 10: SD Card Chip Select (SPI)
  Pin 13: Status LED (optional)
  Pin 22: CAN TX
  Pin 23: CAN RX

I2C:
  Pin 18: I2C1_SDA (Sensoren)
  Pin 19: I2C1_SCL (Sensoren)

UART:
  Pin 0/1: UART1 (GPS/Telemetrie)
  Pin 8/9: UART4 (alternative)

SPI:
  Pin 11: MOSI
  Pin 12: MISO
  Pin 13: SCK

Status LED: Pin 13 (optional)
  - Blinken: Normal Operation
  - Steady: Error
  - Off: System Shutdown
```

## Sensor-Ausrichtung

```
Koordinatensystem (Raketenflug):
  X-Achse: Seitlich (rechts positiv)
  Y-Achse: Rückwärts (nach hinten positiv)
  Z-Achse: Aufwärts (oben positiv)

IMU Montage:
  - Befestigen Sie den Sensor mit der Z-Achse nach oben
  - Markieren Sie die +X und +Y Richtung
  - Vermeiden Sie Vibrationen durch dampfende Unterlagen
  
Barometer:
  - Außenseite der Rakete (exponiert zur Atmosphäre)
  - Kleine Öffnung für Druckausgleich
```

## Kalibrierung

### IMU Kalibrierung
```
Accelerometer:
  - Platzieren Sie Sensor auf flacher, ebener Fläche
  - Erfassen Sie 1000 Messwerte
  - Berechnen Sie Durchschnitt = Offset

Gyroscope:
  - Sensor still lagern
  - Erfassen Sie 1000 Messwerte
  - Berechnen Sie Durchschnitt = Bias
  - Wird automatisch beim Start durchgeführt

Magnetometer:
  - Rotieren Sie den Sensor in alle Richtungen
  - Erfassen Sie Min/Max Werte pro Achse
  - Berechnen Sie Mittelpunkt und Skalierung
```

### Barometer Kalibrierung
```
Referenzdruckmessung:
  - Messen Sie lokalen Luftdruck mit Referenzgerät
  - Setzen Sie sea_level_pressure im Code
  - Dies verbessert Höhengenauigkeit

Temperatur-Offsets:
  - Optional: Vergleichen Sie mit Referenzthermometer
  - Passen Sie Kalibrierungskoeffizienten an
```

## Netzwerk-Konfiguration

### CAN-Bus Kommunikation
```
Master-Slave Topologie:
  - MAGGIE: Secondary Node (0x02)
  - Ground Station: Master (0x01)
  - Avionik: Primary (0x00)

Message-IDs:
  0x100-0x1FF: Telemetrie
  0x200-0x2FF: Status
  0x300-0x3FF: Kommandos
  0x400-0x4FF: Errors

Beispiel Telemetrie-Nachricht (0x102):
  Byte 0-1: Beschleunigung X (float16)
  Byte 2-3: Beschleunigung Y (float16)
  Byte 4-5: Beschleunigung Z (float16)
  Byte 6-7: Reserviert
```

## Umweltspezifikationen

```
Betriebstemperatur:
  - Elektronik: -20°C bis +70°C
  - Sensoren: -40°C bis +85°C

Luftdruck:
  - Testreihe 1 (Bodentest): 700-1100 hPa
  - Testreihe 2 (Flugtest): 0.1-700 hPa (bis 120 km Höhe)

Vibrationen:
  - Max. Beschleunigung während Start: ~5g (50 m/s²)
  - Häufigkeit: bis 100 Hz
  
Magnetisches Feld:
  - Erdmagnetfeld: ~25-65 µT
  - Interferenzen: Mindern durch Abstand zu Hochstrom-Leitungen

Feuchtigkeit:
  - Nicht-kapselte Elektronik: max. 80% rel. Luftfeuchtigkeit
  - Lagern Sie in trockener Umgebung
```

## Test- und Qualitätssicherung

### Pre-Flight Checklist
```
□ Alle Sensoren I2C-Scan bestanden
□ SD-Karte erkannt und formatiert
□ IMU kalibriert
□ Barometer kalibriert (sea_level_pressure gesetzt)
□ GPS Lock (falls verwendet)
□ CAN-Bus Verbindung hergestellt
□ Stromversorgung stabil (5V ±10%)
□ Batterie-Spannung ausreichend
□ Speicher verfügbar (min. 100 MB)
□ Telemetrie-Test erfolgreich
□ Experiment-Kontroller online
□ Safety-Monitor aktiv
```

### Funktionsprüfung
```
1. IMU Test:
   - Bewegen Sie Sensor in alle Richtungen
   - Prüfen Sie auf Änderungen in Acc/Gyro/Mag

2. Barometer Test:
   - Prüfen Sie auf positive Höhenänderung (höher = weniger Druck)
   - Temperaturmesswerte im erwarteten Bereich

3. Speicher-Test:
   - Schreiben Sie Testdatei auf SD-Karte
   - Prüfen Sie CSV-Format

4. Telemetrie-Test:
   - Starten Sie Data-Collection
   - Beobachten Sie kontinuierliche Ausgabe auf Serial Monitor
```

---

**Hinweis**: Diese Konfiguration basiert auf Standard-Raketenfly-Parametern. 
Passen Sie die Sensor-Bereiche und Schwellenwerte basierend auf Ihrem spezifischen Experiment an.
