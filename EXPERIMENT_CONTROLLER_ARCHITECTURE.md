# MAGGIE Experiment Controller Architecture

## Überblick

Das Projekt nutzt jetzt **3 spezialisierte Experiment-Controller** statt eines generischen Controllers. Dies ermöglicht:

- ✅ Klare Separation of Concerns
- ✅ Optimierte Logik für jeden Use-Case
- ✅ Einfacheres Debugging und Testing
- ✅ Reduktion von Code-Komplexität

---

## 1. Ground Test Controller

**Dateistandort:**
- Header: `/include/control/ground_test_controller.h`
- Implementation: `/src/control/ground_test_controller.cpp`

**Zweck:** Testen einzelner Hardware-Komponenten am Boden

**Hauptfunktionen:**
```cpp
testSensor(uint8_t sensor_id)          // Test Sensor (IMU, Baro, Temp)
testDevice(uint8_t device_id)          // Test Device (Ventile, Motor)
activateDevice(device_id, power)       // Manuelle Steuerung
deactivateDevice(device_id)            // Gerät deaktivieren
calibrateSensor(uint8_t sensor_id)     // Sensor kalibrieren
startContinuousRead(sensor_id)         // Kontinuierliches Auslesen
handleSerialCommand(command)           // Debug-Konsole Interface
```

**Verfügbare Serial-Befehle:**
```
TEST_SENSOR 0           - Test IMU
TEST_SENSOR 1           - Test Barometer
TEST_SENSOR 2           - Test Temperature
TEST_DEVICE 0           - Test Ventil 1 (2 Sekunden aktivieren)
TEST_DEVICE 1           - Test Ventil 2
TEST_DEVICE 2           - Test Motor (Speed-Ramp)
ACTIVATE <id> <power>   - Gerät mit PWM aktivieren (0-255)
DEACTIVATE <id>         - Gerät deaktivieren
CALIBRATE <id>          - Sensor kalibrieren
CONTINUOUS <id>         - Sensor kontinuierlich auslesen
STOP                    - Kontinuierliches Auslesen beenden
```

**Use-Cases:**
- 🔧 Hardware-Diagnose
- 📊 Sensor-Überprüfung
- ⚙️ Geräte-Test (einzeln)
- 🔌 Kalibrierung

**States:**
- `IDLE` - Bereit
- `SENSOR_TEST` - Sensor wird getestet
- `DEVICE_ACTIVATION` - Gerät wird aktiviert
- `CALIBRATION` - Kalibrierung läuft
- `CONTINUOUS_READ` - Kontinuierliches Auslesen

---

## 2. Preflight Test Controller

**Dateistandort:**
- Header: `/include/control/preflight_test_controller.h`
- Implementation: `/src/control/preflight_test_controller.cpp`

**Zweck:** Validierung der kompletten Experiment-Sequenz VOR dem Start

**Hauptfunktionen:**
```cpp
runFullTest(bool compressed_time)      // Kompletter Preflight-Test
testSensorValidation()                 // Nur Sensoren testen
testExperimentSequence(seq_id)         // Sequence validieren
testEmergencyStop()                    // Emergency-Stop testen
testTimingValidation()                 // Timing-Genauigkeit prüfen
simulateFlightPhase(phase_id, ms)      // Flugphase simulieren
printTestReport()                      // Test-Bericht ausgeben
```

**Test-Ablauf:**
1. **Sensor Validation** - Prüfe alle Sensoren auf Funktion
2. **Experiment Sequences** - Teste alle Experiment-Sequenzen
3. **Emergency Stop** - Validiere Notfall-Abschaltung
4. **Timing Validation** - Überprüfe Timing-Genauigkeit

**Besonderheit: Compressed Time**
```cpp
// Schneller Test: 1 Sekunde im Test = 1 Minute echte Flugzeit
runFullTest(true);   // ~30 Sekunden Test statt 30 Minuten Flugzeit
```

**Ausgabe-Beispiel:**
```
===================================
PREFLIGHT TEST REPORT
===================================
Total Tests: 6
✓ Passed: 6
✗ Failed: 0
Duration: 5432 ms
===================================
```

**Use-Cases:**
- ✅ Vor-Start-Validierung
- 🧪 Experiment-Sequence-Test
- ⏱️ Timing-Überprüfung
- 🚨 Safety-System-Validierung
- 📝 Checklisten-Erfüllung

**States:**
- `IDLE` - Bereit
- `RUNNING` - Test läuft
- `SENSOR_VALIDATION` - Sensoren werden validiert
- `SEQUENCE_TEST` - Sequenzen werden getestet
- `EMERGENCY_STOP_TEST` - Emergency-Stop wird getestet
- `TIMING_VALIDATION` - Timing wird validiert
- `COMPLETE` - Test erfolgreich abgeschlossen
- `FAILED` - Test fehlgeschlagen

---

## 3. Flight Experiment Controller

**Dateistandort:**
- Header: `/include/control/flight_experiment_controller.h`
- Implementation: `/src/control/flight_experiment_controller.cpp`

**Zweck:** Echtes Experiment während des Raketenflugs

**Hauptfunktionen:**
```cpp
armExperiment()                        // Gerät für Start armen
startExperiment()                      // Experiment starten
pauseExperiment()                      // Experiment pausieren
resumeExperiment()                     // Experiment fortsetzen
emergencyStop(reason)                  // Notfall-Stopp
onFlightPhaseChange(phase)             // Flugphase-Wechsel
getExperimentDuration()                // Dauer seit Start
hasError() / getErrorCode()            // Fehler-Status
```

**Integrierte Flugphase-Logik:**

| Phase | Aktion |
|-------|--------|
| **GROUND** | Warte auf Lift-Off |
| **ASCENT** | Pausiere Experiment (zu viel Vibration) |
| **MICROGRAVITY** | 🚀 STARTE EXPERIMENT |
| **DESCENT** | Pausiere Experiment |
| **RECOVERY** | Beende Experiment, sichere Daten |

**Automatische Phase-Transition:**
```
Das FlightExperimentController wird von MissionControl
über onFlightPhaseChange() benachrichtigt:

MissionControl erkennt Lift-Off (Accelerometer-Schwelle)
  ↓
Aktiviert ASCENT Phase
  ↓
FlightExperimentController pausiert Experiment
  ↓
MissionControl erkennt Apogee (Höchster Punkt)
  ↓
Aktiviert MICROGRAVITY Phase
  ↓
FlightExperimentController STARTET Experiment
  ↓
Experiment läuft für ~15-20 Sekunden Schwerelosigkeit
  ↓
MissionControl erkennt Abstieg (Altitude fällt)
  ↓
Aktiviert DESCENT Phase
  ↓
FlightExperimentController pausiert Experiment
```

**Error-Handling:**
```cpp
// Kritische Fehler führen automatisch zu Emergency-Stop
checkSafetyConditions()  // Prüfe:
  - Temperaturgrenzen
  - Druckgrenzen
  - Stromaufnahme
  - Sensorfehler
```

**Use-Cases:**
- 🚀 Echtes Flug-Experiment
- 📊 Echtzeit-Datenerfassung
- 🛡️ Fehlerbehandlung im Flug
- 🔒 Safety-Integrationen

**States:**
- `IDLE` - Vor Start
- `ARMED` - Bereit für Start
- `PRE_LAUNCH` - 10 Sekunden vor Lift-Off
- `RUNNING` - Experiment läuft ✓
- `PAUSED` - Pausiert
- `EMERGENCY_STOP` - Notfall-Stopp
- `FINISHED` - Abgeschlossen
- `ERROR` - Fehler aufgetreten

---

## Integration in MissionControl

### Aktuelle Architektur:

```
┌─────────────────────────────────────────┐
│         MissionControl                  │
│    (verwaltet Flugphasen und Timing)   │
└──────────────────┬──────────────────────┘
                   │
        ┌──────────┼──────────┐
        ↓          ↓          ↓
   ┌─────────┐ ┌──────────┐ ┌──────────────────┐
   │ Safety  │ │ Telemetry│ │ FlightExperiment │
   │ Monitor │ │ Service  │ │ Controller       │
   └─────────┘ └──────────┘ └──────────────────┘
                                      ↓
                              ┌──────────────┐
                              │ Devices      │
                              │ (Ventile,    │
                              │  Motor, etc.)│
                              └──────────────┘
```

### Beispiel: Flug-Ablauf

```cpp
// In MissionControl::run()

switch (current_state) {
    case MissionState::STANDBY:
        // Warte auf Lift-Off Signal
        if (isLiftOffSignal()) {
            experiment.onFlightPhaseChange(FlightPhase::ASCENT);
            onStateChange(MissionState::ASCENT);
        }
        break;
        
    case MissionState::ASCENT:
        // MICROGRAVITY wird automatisch erkannt (Accelerometer-Sensor)
        if (detectedApogee()) {
            experiment.onFlightPhaseChange(FlightPhase::MICROGRAVITY);
            onStateChange(MissionState::MICROGRAVITY);
        }
        experiment.update();  // Experiment pausiert während Ascent
        break;
        
    case MissionState::MICROGRAVITY:
        // 🚀 Experiment läuft automatisch!
        experiment.update();  // Experiment AKTIV
        if (detectedDescent()) {
            experiment.onFlightPhaseChange(FlightPhase::DESCENT);
            onStateChange(MissionState::DESCENT);
        }
        break;
}
```

---

## Workflow für Entwicklung

### 1. **Entwicklung** (mit Ground Test Controller)
```cpp
// main.cpp - Entwicklungs-Modus
GroundTestController ground_test;
ground_test.init();

void loop() {
    // Serial-Befehle:
    // "TEST_DEVICE 0" - Test Ventil 1
    // "ACTIVATE 0 128" - Ventil mit 50% Power
    // "CONTINUOUS 0" - Sensor auslesen
    ground_test.handleSerialCommand(Serial.readStringUntil('\n'));
    ground_test.update();
}
```

### 2. **Pre-Flight** (mit Preflight Test Controller)
```cpp
// main.cpp - Pre-Flight-Modus
PreflightTestController preflight;
preflight.init();

void loop() {
    preflight.runFullTest(true);  // Schneller Test
    preflight.update();
    
    if (preflight.getStatus() == PreflightTestState::COMPLETE) {
        Serial.println("✓ READY FOR FLIGHT");
        // Switch to Flight Mode
    }
}
```

### 3. **Flug** (mit Flight Experiment Controller)
```cpp
// main.cpp - Flight-Modus
MissionControl mission;
mission.init();

void loop() {
    mission.run();  // MissionControl ruft experiment.update() auf
}
```

---

## Checkliste: Implementierung

- [ ] Ground Test Controller - Hardware-Befehle implementieren
- [ ] Preflight Test Controller - Sequenz-Tests implementieren
- [ ] Flight Experiment Controller - Flugphase-Logik implementieren
- [ ] MissionControl - Integration mit neuen Controllern
- [ ] main.cpp - Mode-Auswahl (Entwicklung/Preflight/Flight)
- [ ] Sensor-Validierung - Implementieren in allen 3 Controllern
- [ ] Error-Handling - Exception-Handling und Logging
- [ ] Testing - Unit-Tests für alle 3 Controller

---

## FAQ

**F: Warum 3 Controller statt 1?**
A: Jeder Controller hat einen anderen Zweck und Optimierungen. So ist der Code sauberer und leichter zu warten.

**F: Kann ich zwischen den Modi wechseln?**
A: Ja! Mode-Auswahl in `main.cpp` über Compile-Time-Defines oder Serial-Kommando.

**F: Was passiert bei Fehler im Flug?**
A: Flight Experiment Controller erkennt den Fehler, pausiert Experiment und benachrichtigt MissionControl.

**F: Wie wird die Experiment-Logik implementiert?**
A: In `FlightExperimentController::executeExperimentSequence()` - dort werden Ventile/Motor/Sensoren gesteuert.

