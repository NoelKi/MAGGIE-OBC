# MAGGIE - On-Board Computer Software
## REXUS Rocket Experiment System

### Übersicht

MAGGIE ist ein vollständiges Embedded-System für Experimente im REXUS-Raketen-Programm. Das System verwaltet Datenerfassung, Telemetrie-Übertragung, Sensorkalibrierung und Experiment-Kontrolle.

### Projektstruktur

```
src/
├── main.cpp                      # Haupt-Eintrittspunkt
├── system.cpp                    # Zentrale System-Verwaltung
├── control/
│   ├── mission_control.cpp       # Missions-Koordination (Startup, Flug, Recovery)
│   ├── experiment_controller.cpp # Experiment-Steuerung
│   └── safety_monitor.cpp        # Sicherheits- und Watchdog-Überwachung
├── drivers/
│   ├── imu_driver.cpp           # IMU (9-DOF Sensor)
│   ├── pressure_temperature_driver.cpp  # Barometer + Thermometer
│   ├── gps_driver.cpp           # GPS Empfänger (optional)
│   └── storage_driver.cpp       # SD-Karten Speicher
└── services/
    ├── data_collection_service.cpp  # Sensor-Datenerfassung
    ├── telemetry_service.cpp        # Daten-Übertragung
    └── logging_service.cpp          # Daten-Logging auf SD-Karte

include/
├── system.h
├── control/
├── drivers/
└── services/

hal/
├── hal_can.h / .cpp             # CAN-Bus Hardware Layer
└── hal_gpio.h / .cpp            # GPIO Hardware Layer
```

### Hauptkomponenten

#### 1. **Mission Control** (`mission_control.h/cpp`)
Koordiniert den gesamten Missionsablauf:
- **STARTUP**: Initialisierungsphase
- **PREFLIGHT_CHECK**: Systemüberprüfungen
- **STANDBY**: Bereitschaft
- **ASCENT**: Aufstiegsphase
- **MICROGRAVITY**: Schwerelosigkeitsphase
- **DESCENT**: Abstiegsphase
- **RECOVERY**: Bergungsphase
- **TESTING**: Bergungsphase

#### 2. **Sensortreiber**

**IMU Driver** (9-DOF):
- Accelerometer (3 Achsen)
- Gyroscope (3 Achsen)
- Magnetometer (3 Achsen)
- Unterstützte Hardware: MPU9250, ICM-20948

**Barometer/Thermometer**:
- Luftdruck (hPa)
- Temperatur (°C)
- Höhenberechnung
- Unterstützte Hardware: BMP390, BMP280, MS5607

**GPS** (Optional):
- Position, Höhe
- Geschwindigkeit
- Satellitenanzahl

#### 3. **Datenerfassungs-Service**
- Koordiniert alle Sensoren
- Konfigurierbare Sample-Rate (bis 1000 Hz)
- Automatische Kalibrierung
- Status-Monitoring

#### 4. **Telemetrie-Service**
- Mehrere Übertragungskanäle:
  - Serial (Debug via USB)
  - CAN Bus (für Avionik)
  - LoRa (optionale Notfall-Telemetrie)
- CSV-Formatierung
- Binäre Übertragung

#### 5. **Logging-Service**
- Permanente Speicherung auf SD-Karte
- CSV-Format für einfache Datenanalyse
- Event-Logging
- Speicherplatz-Management

#### 6. **Safety Monitor**
- Sensor-Werteüberwachung
- Temperatur/Beschleunigung/Höhe-Limits
- Stromversorgung-Check
- Speicher-Check
- Notfall-Abschaltung

#### 7. **Experiment Controller**
- Experiment-Zustandsverwaltung
- Gerätsteuerung (Ventile, Pumpen, Solenoid)
- Zeit-Management
- Trigger-Logik

### Hardware-Anforderungen

**Mikrocontroller:**
- Teensy 4.1 (ARM Cortex-M7, 600 MHz)

**Sensoren:**
- IMU: MPU9250 oder ICM-20948 (I2C)
- Barometer: BMP390 oder BMP280 (I2C)
- GPS: NEO-6M oder NEO-M9N (UART) - optional
- SD-Karte: über SPI

**Schnittstellen:**
- I2C Bus (Sensoren)
- SPI Bus (SD-Karte)
- UART (GPS, Telemetrie)
- CAN Bus (Avionik)
- USB (Debug)

### Verwendung

#### Kompilierung und Upload:
```bash
# Mit PlatformIO
platformio run -t upload

# oder in VS Code mit PlatformIO Erweiterung
```

#### Monitoring:
```bash
# Serial Monitor
platformio device monitor --baud 115200
```

### Software-Architektur

Das System folgt einer geschichteten Architektur:

```
┌─────────────────────────────────┐
│   System Control (system.h)     │
│   - main() Einstiegspunkt       │
└────────────────┬────────────────┘
                 │
┌────────────────┴────────────────┐
│   Mission Control               │
│   - Missionsablauf-Verwaltung   │
│   - Zustandsübergänge           │
└────────────────┬────────────────┘
                 │
    ┌────────────┼────────────────┬──────────────┐
    │            │                │              │
┌───┴────────┐ ┌─┴──────────┐ ┌──┴──────────┐ ┌┴────────────┐
│Experiment  │ │Data        │ │Telemetry   │ │Safety       │
│Controller  │ │Collection  │ │Service     │ │Monitor      │
│            │ │Service     │ │            │ │            │
└────────────┘ └──┬─────────┘ └────────────┘ └────────────┘
                  │
      ┌───────────┴────────────┬──────────────┐
      │                        │              │
   ┌──┴──────┐    ┌──────────┴──┐    ┌──────┴──────┐
   │IMU      │    │Barometer    │    │GPS          │
   │Driver   │    │Driver       │    │Driver       │
   └─────────┘    └─────────────┘    └─────────────┘
   
      ┌─────────────────────────────┐
      │  Storage/Logging Service    │
      │  - SD-Karten-Verwaltung     │
      │  - CSV-Export               │
      └─────────────────────────────┘
```

### Datenformat (CSV)

Telemetrie-Daten werden im folgenden Format gespeichert:

```csv
timestamp,sequence,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,pressure,temperature,altitude,sensor_status
1234567890,0,0.123,-0.456,9.810,0.001,0.002,-0.003,10.5,20.3,-5.2,1013.25,22.5,0.0,3
```

### Konfiguration

Wichtige Konfigurationsparameter (in Header-Dateien):

- **IMU Sample-Rate**: 100 Hz (konfigurierbar in DataCollectionService)
- **Telemetrie-Interval**: 100 ms (10 Hz)
- **Max. Experiment-Dauer**: 300 Sekunden (5 Minuten)
- **Watchdog-Timeout**: 5 Sekunden
- **Beschleunigung-Limit**: 50 m/s² (~5g)
- **Temperatur-Bereich**: -40°C bis +85°C
- **Höhen-Limit**: 100 km

### Erweiterung und Anpassung

Das System ist modular aufgebaut für leichte Anpassung:

1. **Neue Sensoren hinzufügen**: Neuen Treiber in `src/drivers/` erstellen, von DataCollectionService aufrufen
2. **Experiment-Logik**: Änderungen in `ExperimentController::update()`
3. **Telemetrie-Kanäle**: Neue Kanäle in `TelemetryService` hinzufügen
4. **Sicherheits-Limits**: Anpassung in `SafetyMonitor`

### Debugging und Troubleshooting

**Serial Output aktivieren:**
- Terminal-Baudrate: 115200
- Alle wichtigen Übergänge werden auf Serial geloggt

**Häufige Probleme:**

| Problem | Lösung |
|---------|--------|
| Sensor nicht erkannt | I2C Adresse prüfen, Wire.begin() aufrufen |
| SD-Karte nicht gefunden | SPI Pins prüfen, CS Pin konfigurieren |
| Daten nicht übertragen | Baudrate prüfen, Kabel überprüfen |
| Watchdog-Timeout | Loop zu lange, optimieren |

### Performance-Charakteristiken

- **Speicherverbrauch**: ~50 KB RAM (bei Teensy 4.1: 512 KB verfügbar)
- **Durchsatz Telemetrie**: ~1200 Bytes/Sekunde (bei 10 Hz)
- **SD-Karten Schreibgeschwindigkeit**: ~10-20 KB/s
- **Zykluszeit Loop**: ~10 ms bei 100 Hz Sensor-Rate

### Lizenz und Attribution

Entwickelt für das REXUS-Programm (Rocket Experiment for University Students)
Europäische Raumfahrtagentur (ESA) in Zusammenarbeit mit deutschen Universitäten

### Support und Kontakt

Für Fragen und Issues: [Ihr Kontakt hier]

---

**Letzte Aktualisierung**: März 2026
**Version**: 1.0
# MAGGIE-OBC
