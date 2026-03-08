# MAGGIE — On-Board Computer Software
## REXUS Rocket Experiment System

### Übersicht

MAGGIE ist ein vollständiges Embedded-System für Experimente im REXUS-Raketen-Programm. Das System verwaltet Datenerfassung, Telemetrie-Übertragung, Sensorkalibrierung und Experiment-Kontrolle über drei spezialisierte Betriebsmodi: **Flight**, **Preflight-Testing** und **Hardware-Testing**.

---

### Projektstruktur

```
MAGGIE_OBC/
├── platformio.ini                     # PlatformIO Konfiguration (Teensy 4.1)
├── EXPERIMENT_CONTROLLER_ARCHITECTURE.md  # Architektur-Dokumentation
├── src/
│   ├── main.cpp                       # Einstiegspunkt
│   ├── system.cpp                     # System-Initialisierung
│   ├── control/
│   │   ├── mission_control.cpp        # Top-Level Missions-Koordination
│   │   ├── flight_experiment_controller.cpp  # Flug-Experiment (Mikrogravitation)
│   │   ├── ground_test_controller.cpp        # Hardware-Test am Boden
│   │   ├── preflight_test_controller.cpp     # Preflight-Validierung
│   │   └── safety_monitor.cpp         # Sicherheits- & Watchdog-Überwachung
│   ├── drivers/
│   │   ├── imu_driver.cpp             # IMU (9-DOF: Acc, Gyro, Mag)
│   │   ├── pressure_temperature_driver.cpp  # Barometer + Thermometer
│   │   ├── pressure_sensor_driver.cpp # Drucksensor
│   │   ├── can_bus_driver.cpp         # CAN-Bus (STM32 Roboterarm)
│   │   ├── pwm_motor_driver.cpp       # PWM Motor-Steuerung
│   │   ├── weight_sensor_driver.cpp   # Gewichtssensor
│   │   └── storage_driver.cpp         # SD-Karten Speicher
│   ├── services/
│   │   ├── data_collection_service.cpp    # Sensor-Datenerfassung
│   │   ├── experiment_data_service.cpp    # Experiment-Datenverwaltung
│   │   ├── telemetry_service.cpp          # UDP-Telemetrie an Ground Station
│   │   └── logging_service.cpp            # SD-Karten Logging
│   └── hal/
│       ├── hal.h / hal.cpp            # Zentrale HAL — importiere statt Einzelmodule
│       ├── hal_config.h               # Pin-Definitionen & Konfiguration
│       ├── hal_gpio.h / .cpp          # GPIO (digitale Ein-/Ausgänge, LEDs)
│       ├── hal_i2c.h / .cpp           # I2C (IMU, Barometer)
│       ├── hal_spi.h / .cpp           # SPI (Drucksensor, SD-Karte)
│       ├── hal_adc.h / .cpp           # ADC (Gewichtssensoren)
│       ├── hal_pwm.h / .cpp           # PWM (Motor)
│       ├── hal_uart.h / .cpp          # UART (Telemetrie, RXSM)
│       └── hal_can.h / .cpp           # CAN-Bus (STM32 Roboterarm)
├── include/
│   ├── system.h
│   ├── control/
│   │   ├── mission_control.h
│   │   ├── flight_experiment_controller.h
│   │   ├── ground_test_controller.h
│   │   ├── preflight_test_controller.h
│   │   └── safety_monitor.h
│   ├── drivers/
│   │   ├── imu_driver.h
│   │   ├── pressure_temperature_driver.h
│   │   ├── pressure_sensor_driver.h
│   │   ├── can_bus_driver.h
│   │   ├── pwm_motor_driver.h
│   │   ├── weight_sensor_driver.h
│   │   └── storage_driver.h
│   └── services/
│       ├── data_collection_service.h
│       ├── experiment_data_service.h
│       ├── telemetry_service.h
│       └── logging_service.h
└── docs/
    └── README.md                      # Diese Datei
```

---

### Hauptkomponenten

#### 1. **Mission Control** (`mission_control.h/cpp`)
Top-Level Koordination des Missionsablaufs. Verwaltet ausschließlich den Lifecycle — keine Flugphasen-Details.

| Zustand | Beschreibung |
|---------|-------------|
| `STARTUP` | Boot-Phase: HAL + Services initialisieren |
| `PREFLIGHT_CHECK` | Automatische Systemchecks |
| `STANDBY` | Bereit — Modus-Auswahl via Serial (F/T/H) |
| `EXPERIMENT` | Flug-Modus: FlightExperimentController aktiv |
| `TESTING` | Preflight-Modus: PreflightTestController aktiv |
| `HARDWARE_TESTING` | HW-Test-Modus: GroundTestController aktiv |
| `SHUTDOWN` | System herunterfahren |

#### 2. **Drei spezialisierte Experiment-Controller**

> Detaillierte Beschreibung: `EXPERIMENT_CONTROLLER_ARCHITECTURE.md`

**Flight Experiment Controller** (`flight_experiment_controller.h/cpp`)
Für den echten Raketenflug. Erkennt Flugphasen anhand von Sensordaten und steuert das Experiment automatisch.

| FlightPhase | Aktion |
|-------------|--------|
| `GROUND` | Warte auf Lift-Off |
| `ASCENT` | Experiment pausiert (Vibration) |
| `MICROGRAVITY` | 🚀 Experiment aktiv |
| `DESCENT` | Experiment pausiert |
| `RECOVERY` | Daten sichern, Abschluss |

**Ground Test Controller** (`ground_test_controller.h/cpp`)
Für Hardware-Diagnose am Boden via Serial-Befehle:
```
TEST_SENSOR <id>        — IMU / Barometer / Temperatur testen
TEST_DEVICE <id>        — Ventil / Motor testen
ACTIVATE <id> <power>   — Gerät manuell aktivieren (0–255)
CALIBRATE <id>          — Sensor kalibrieren
CONTINUOUS <id>         — Sensor kontinuierlich auslesen
STOP                    — Auslesen stoppen
```

**Preflight Test Controller** (`preflight_test_controller.h/cpp`)
Validiert die komplette Experiment-Sequenz vor dem Start:
```
1. Sensor Validation       — Alle Sensoren prüfen
2. Experiment Sequences    — Abläufe testen
3. Emergency Stop          — Notfall-Abschaltung validieren
4. Timing Validation       — Timing-Genauigkeit prüfen
```
Compressed-Time-Modus: `runFullTest(true)` — 1 s Test ≙ 1 min Flugzeit.

#### 3. **HAL — Hardware Abstraction Layer** (`src/hal/`)
Zentrale Hardware-Schnittstelle. Immer `hal.h` importieren statt Einzelmodule:

```cpp
#include "hal/hal.h"
HAL::initializeAll();  // Alle Submodule auf einmal initialisieren
```

| Modul | Funktion |
|-------|---------|
| `hal_config.h` | Pin-Definitionen (muss zuerst inkludiert werden) |
| `hal_gpio` | Digitale Ein-/Ausgänge, LEDs |
| `hal_i2c` | I2C-Bus (IMU, Barometer) |
| `hal_spi` | SPI-Bus (Drucksensor, SD-Karte) |
| `hal_adc` | Analog/Digital-Wandlung (Gewichtssensoren) |
| `hal_pwm` | PWM-Motorsteuerung |
| `hal_uart` | Serielle Kommunikation (Telemetrie, RXSM) |
| `hal_can` | CAN-Bus (STM32 Roboterarm) |

#### 4. **Sensortreiber** (`src/drivers/`)

| Treiber | Hardware | Schnittstelle |
|---------|----------|--------------|
| `imu_driver` | MPU9250 / ICM-20948 | I2C |
| `pressure_temperature_driver` | BMP390 / BMP280 / MS5607 | I2C |
| `pressure_sensor_driver` | Externer Drucksensor | SPI |
| `weight_sensor_driver` | Gewichtssensor | ADC |
| `pwm_motor_driver` | Bürstenloser Motor | PWM |
| `can_bus_driver` | STM32 Roboterarm | CAN |
| `storage_driver` | SD-Karte | SPI |

#### 5. **Services** (`src/services/`)

| Service | Beschreibung |
|---------|-------------|
| `data_collection_service` | Koordiniert alle Sensoren, konfigurierbare Sample-Rate |
| `experiment_data_service` | Experiment-Datenverwaltung und -pufferung |
| `telemetry_service` | UDP-Telemetrie an MAGGIE Ground Station (Port 9000) |
| `logging_service` | Permanente Speicherung auf SD-Karte |

#### 6. **Safety Monitor** (`safety_monitor.h/cpp`)
- Sensor-Werteüberwachung (Temperatur, Beschleunigung, Höhe)
- Stromversorgung- und Speicher-Check
- Automatischer Notfall-Stopp bei Grenzwertverletzung

---

### Software-Architektur

```
┌──────────────────────────────────────────┐
│              MissionControl              │
│  STARTUP → PREFLIGHT → STANDBY → ...    │
└────────────────────┬─────────────────────┘
                     │  Modus-Auswahl (F/T/H)
        ┌────────────┼────────────────┐
        ↓            ↓                ↓
┌─────────────┐ ┌──────────────┐ ┌───────────────────┐
│  Preflight  │ │   Ground     │ │  Flight           │
│    Test     │ │    Test      │ │  Experiment       │
│ Controller  │ │ Controller   │ │  Controller       │
└─────────────┘ └──────────────┘ └─────────┬─────────┘
                                           │ Flugphasen
        ┌──────────────────────────────────┴──────────┐
        │          Safety Monitor  |  Telemetry        │
        └─────────────────────────────────────────────┘
                              │
           ┌──────────────────┼───────────────────┐
           ↓                  ↓                   ↓
    ┌─────────────┐   ┌──────────────┐   ┌──────────────┐
    │ IMU Driver  │   │  Pressure /  │   │  Weight /    │
    │  (I2C)      │   │  Baro Driver │   │  Motor / CAN │
    └─────────────┘   └──────────────┘   └──────────────┘
                              │
                    ┌─────────────────┐
                    │   HAL Layer     │
                    │  GPIO I2C SPI   │
                    │  ADC PWM UART   │
                    │  CAN            │
                    └─────────────────┘
```

---

### Hardware

**Mikrocontroller:** Teensy 4.1 (ARM Cortex-M7, 600 MHz, 512 KB RAM)

**Schnittstellen:**
- I2C: IMU, Barometer
- SPI: Drucksensor, SD-Karte
- ADC: Gewichtssensoren
- PWM: Motor
- UART: Telemetrie (UDP via RXSM), Serial Debug
- CAN: STM32 Roboterarm

---

### Verwendung

#### Build & Upload
```bash
# Mit PlatformIO CLI
platformio run -t upload

# oder in VS Code mit PlatformIO-Erweiterung (Ctrl+Alt+U)
```

#### Serial Monitor
```bash
platformio device monitor --baud 115200
```

#### Betriebsmodus wählen (aus STANDBY via Serial)
```
F  —  Flight Mode          (FlightExperimentController)
T  —  Testing Mode         (PreflightTestController)
H  —  Hardware Test Mode   (GroundTestController)
S  —  Shutdown
```

---

### Debugging

**Serial Output:** Baudrate 115200 — alle Zustandsübergänge werden geloggt.

**Häufige Probleme:**

| Problem | Lösung |
|---------|--------|
| Sensor nicht erkannt | I2C-Adresse prüfen, `Wire.begin()` aufrufen |
| SD-Karte nicht gefunden | SPI-Pins und CS-Pin prüfen |
| CAN-Bus keine Antwort | Terminierung prüfen, Baudrate |
| Watchdog-Timeout | Loop-Zeit reduzieren (Ziel: < 10 ms) |

---

### Lizenz

Entwickelt für das REXUS-Programm (Rocket Experiment for University Students) —
ESA / ZARM / DLR

---

**Letzte Aktualisierung**: März 2026  
**Version**: 2.0
