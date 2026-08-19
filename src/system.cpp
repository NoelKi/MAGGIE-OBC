#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

// ===========================================================================
// Sensor Read Intervals (in milliseconds)
// ===========================================================================
#define WEIGHT_READ_INTERVAL_MS 500     ///< Read weight every 500ms
#define FORCE_READ_INTERVAL_MS 100      ///< Read force every 100ms
#define TELEMETRY_INTERVAL_MS 50        ///< Send IMU downlink every 50ms (20 Hz)
#define SYS_TELEMETRY_INTERVAL_MS 1000  ///< Send SYS/STATE downlink every 1s (1 Hz)
#define WEIGHT_CALIBRATION_FACTOR 1.0f  ///< Calibration factor for HX711
#define DOWNLINK_BAUDRATE 38400         ///< Downlink UART speed (Serial8, pins 34/35)

System::System() {
    // Konstruktor
}

System::~System() {
    delete weight_sensor_;
    delete force_sensor_2_;
    delete imu_;
    delete downlink_;
    delete motor_;
    delete uplink_;
    delete rexus_;
    for (auto* camera : cameras_) {
        delete camera;
    }
    delete camera_bus_;  // nach den Kameras, sie halten einen Zeiger darauf
}

bool System::init() {
    startup_time = millis();

    printWelcomeBanner();

    // -----------------------------------------------------------------------
    // Gewichtssensor (HX711, ein Sensor an Pin 2/3)
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Gewichtssensor (HX711)...");
    weight_sensor_ = new WeightSensorDriver(
        PIN_HX711_DOUT, PIN_HX711_SCK,
        PIN_HX711_DOUT, PIN_HX711_SCK   // zweiter Kanal zeigt auf denselben Chip – wird nicht verwendet
    );

    weight_sensor_ready_ = weight_sensor_->init(ScaleType::HX711);
    if (weight_sensor_ready_) {
        // Kalibrierung und Tare
        weight_sensor_->setCalibrationFactor(1, WEIGHT_CALIBRATION_FACTOR);
        weight_sensor_->tareScale1();
        Serial.println("INFO  [System]: Gewichtssensor bereit.");
    } else {
        Serial.println("WARN  [System]: Gewichtssensor nicht initialisiert - weiter ohne Gewichtsmessung.");
    }

    // -----------------------------------------------------------------------
    // Kraftsensor 2 (Custom Analog Sensor)
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Kraftsensor 2 (Analog)...");
    force_sensor_2_ = new ForceSensorHAL(
        FORCE_SENSOR_2_X_PIN,
        FORCE_SENSOR_2_Y_PIN,
        FORCE_SENSOR_2_Z_PIN
    );

    force_sensor_ready_ = force_sensor_2_->init();
    if (force_sensor_ready_) {
        Serial.println("INFO  [System]: Kraftsensor 2 bereit.");
    } else {
        Serial.println("WARN  [System]: Kraftsensor 2 nicht initialisiert - weiter ohne Kraftmessung.");
    }

    // -----------------------------------------------------------------------
    // IMU (BMI088 ueber SPI) - fuer Telemetrie-Downlink
    // Fehler hier sind NICHT fatal: das System laeuft ohne Telemetrie weiter.
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere IMU (BMI088)...");
    imu_ = new IMUHAL(PIN_CS_ACCEL, PIN_CS_GYRO, 0 /* SPI0 */);
    imu_ready_ = imu_->init();
    if (imu_ready_) {
        Serial.println("INFO  [System]: IMU bereit.");
    } else {
        Serial.println("WARN  [System]: IMU konnte nicht initialisiert werden - Telemetrie ohne IMU.");
    }

    // -----------------------------------------------------------------------
    // Telemetrie-Downlink (Serial8, Pins 34 RX / 35 TX)
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Telemetrie-Downlink (Serial8, Pin 34/35)...");
    downlink_ = new TelemetryDownlink(Serial8);
    downlink_ready_ = downlink_->init(DOWNLINK_BAUDRATE);
    if (downlink_ready_) {
        Serial.println("INFO  [System]: Downlink bereit (38400 Baud).");
    } else {
        Serial.println("WARN  [System]: Downlink konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Motor 1 (DRV8871 + Quadratur-Encoder, Closed-Loop Positionsregelung)
    // Fehler hier sind NICHT fatal: das System laeuft ohne Motor weiter.
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Motor 1 (DRV8871 + Encoder)...");
    motor_ = new MotorHAL(PIN_M1_A, PIN_M1_B, 1);
    motor_ready_ = motor_->init();
    if (motor_ready_) {
        motor_->initEncoder(PIN_M1_ENC_A, PIN_M1_ENC_B);
        Serial.printf("INFO  [System]: Motor 1 bereit (Treiber Pin %u/%u, Encoder Pin %u/%u, %s).\n",
                      PIN_M1_A, PIN_M1_B, PIN_M1_ENC_A, PIN_M1_ENC_B,
                      motor_->usesSoftPwm() ? "Software-PWM" : "Hardware-PWM");
    } else {
        Serial.println("WARN  [System]: Motor 1 konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Uplink-Telecommand-Empfang (teilt sich Serial8 mit dem Downlink, Pin 34 RX)
    // -----------------------------------------------------------------------
    uplink_ = new UplinkReceiver(Serial8);

    // -----------------------------------------------------------------------
    // REXUS-Signale (L0 / SOE / SODS) + Missions-Zustandsmaschine
    // Am Bodenaufbau liegen die Leitungen offen -> Pulldowns in REXUSHAL::init()
    // halten sie definiert LOW, sonst würde Rauschen die Sequenz auslösen.
    // -----------------------------------------------------------------------
    Serial.printf("INFO  [System]: Initialisiere REXUS-Signale (L0 %u / SOE %u / SODS %u)...\n",
                  PIN_L0_T, PIN_SOE_I, PIN_SODS_I);
    rexus_ = new REXUSHAL(PIN_L0_T, PIN_SOE_I, PIN_SODS_I);
    rexus_ready_ = rexus_->init();

    state_machine_.init();
    last_state_ = state_machine_.getState();
    Serial.printf("INFO  [System]: Zustandsmaschine gestartet in %s "
                  "(Telecommand TEST_ENTER schaltet in den Bodentest).\n",
                  StateMachine::toString(last_state_));

    // -----------------------------------------------------------------------
    // Kameras (Anzahl/Intervall siehe camera_config.hpp)
    // Fehler hier sind NICHT fatal: einzelne Kameras können fehlen/ungeklärt sein.
    // -----------------------------------------------------------------------
    Serial.printf("INFO  [System]: Initialisiere Kamera-Bus (Mux-Select Pin %u/%u)...\n",
                  PIN_CAM_MUX_A, PIN_CAM_MUX_B);
    camera_bus_ = new CameraBus(CAMERA_BUS_UART, PIN_CAM_MUX_A, PIN_CAM_MUX_B);
    if (!camera_bus_->begin(RunCam::BAUDRATE)) {
        Serial.println("WARN  [System]: Kamera-Bus nicht verfügbar - alle Kameras übersprungen.");
    }

    Serial.printf("INFO  [System]: Initialisiere %u Kamera(s)...\n", (unsigned)CAMERA_COUNT);
    for (size_t i = 0; i < CAMERA_COUNT; ++i) {
        cameras_[i] = new CameraHAL(CAMERA_CONFIGS[i], camera_bus_);
        if (cameras_[i]->init()) {
            // Bus ist offen. Die Kameras brauchen nach dem Einschalten noch
            // einige Sekunden - der Handshake läuft in handleCameras().
            Serial.printf("INFO  [System]: Kamera %u auf Mux-Kanal %u, warte auf Boot...\n",
                          cameras_[i]->getCameraID(), cameras_[i]->getMuxChannel());
        } else {
            Serial.printf("WARN  [System]: Kamera %u deaktiviert oder kein Bus - übersprungen.\n",
                          cameras_[i]->getCameraID());
        }
    }

    // -----------------------------------------------------------------------
    // Zusammenfassung. Der OBC startet auch dann, wenn einzelne Subsysteme
    // fehlen - system_healthy meldet den Zustand nur an die Bodenstation.
    // -----------------------------------------------------------------------
    system_healthy = weight_sensor_ready_ && force_sensor_ready_ &&
                     imu_ready_ && downlink_ready_;

    if (system_healthy) {
        Serial.println("INFO  [System]: System erfolgreich initialisiert.\n");
    } else {
        Serial.printf("WARN  [System]: System DEGRADIERT gestartet - Gewicht:%s Kraft:%s IMU:%s Downlink:%s\n\n",
                      weight_sensor_ready_ ? "OK" : "FEHLT",
                      force_sensor_ready_  ? "OK" : "FEHLT",
                      imu_ready_           ? "OK" : "FEHLT",
                      downlink_ready_      ? "OK" : "FEHLT");
    }
    return true;
}

void System::run() {
    const uint32_t now = millis();

    // Eingehende Telecommands verarbeiten (jeden Loop, geringe Latenz)
    handleUplink(now);

    // Missionszustand fortschreiben (REXUS-Signale + Telecommands)
    handleStateMachine(now);

    // Closed-Loop-Positionsregelung des Motors einen Schritt weiterführen
    if (motor_) motor_->update();

    // Gewicht periodisch auslesen und ausgeben
    if (now - last_weight_read_ms_ >= WEIGHT_READ_INTERVAL_MS) {
        last_weight_read_ms_ = now;
        handleWeightReading();
    }

    // Kraftsensor periodisch auslesen und ausgeben
    if (now - last_force_read_ms_ >= FORCE_READ_INTERVAL_MS) {
        last_force_read_ms_ = now;
        handleForceReading();
    }

    // Telemetrie-Downlink periodisch senden (IMU + Motor)
    if (now - last_telemetry_ms_ >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_ms_ = now;
        handleTelemetry();
        handleMotorTelemetry();
    }

    // Zustand + Subsystem-Gesundheit langsamer hinterherschicken (1 Hz)
    if (now - last_sys_telemetry_ms_ >= SYS_TELEMETRY_INTERVAL_MS) {
        last_sys_telemetry_ms_ = now;
        handleSystemTelemetry(now);
    }

    // Kameras: jede Kamera entscheidet anhand ihres CameraTriggerMode selbst,
    // ob gerade ein Kommando fällig ist (siehe camera_config.hpp).
    handleCameras(now);
}

void System::handleCameras(uint32_t now_ms) {
    for (size_t i = 0; i < CAMERA_COUNT; ++i) {
        CameraHAL* camera = cameras_[i];
        if (!camera) continue;

        camera->update(now_ms);

        // Ergebnis des Handshakes genau einmal melden.
        if (camera_reported_[i] || camera->getState() != CameraState::READY) continue;
        camera_reported_[i] = true;

        if (camera->isDetected()) {
            Serial.printf("INFO  [System]: Kamera %u (Kanal %u) erkannt - RunCam-Protokoll v%u, Features 0x%04X.\n",
                          camera->getCameraID(), camera->getMuxChannel(),
                          camera->getProtocolVersion(), camera->getFeatures());
        } else {
            // TX zur Kamera kann trotzdem funktionieren, deshalb kein Abbruch -
            // Kommandos werden ab jetzt ohne Bestätigung gesendet.
            Serial.printf("WARN  [System]: Kamera %u (Kanal %u) antwortet nicht (RX-Leitung/Mux prüfen) - sende blind weiter.\n",
                          camera->getCameraID(), camera->getMuxChannel());
        }
    }
}

void System::handleTelemetry() {
    if (!downlink_ready_ || !imu_ready_ || !imu_ || !downlink_) return;

    IMUReading reading;
    if (!imu_->read(reading)) return;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;
    if (reading.valid)  status1 |= DL_STATUS1_IMU_VALID;

    downlink_->sendImu(reading, status1, 0);
}

void System::handleMotorTelemetry() {
    if (!downlink_ready_ || !motor_ready_ || !motor_ || !downlink_) return;

    const int32_t position = static_cast<int32_t>(motor_->getPosition());
    const int16_t speed    = motor_->getSpeed();

    uint8_t state = 0;
    if (motor_->isOn())     state |= DL_MOTOR_STATE_ON;
    if (motor_->isMoving()) state |= DL_MOTOR_STATE_MOVING;
    else                    state |= DL_MOTOR_STATE_AT_TARGET;

    // HDRM-Stellung aus der Encoder-Position ableiten, damit die Bodenstation
    // sie nicht selbst aus den Counts rekonstruieren muss.
    if (motor_->isHdrmOpen())   state |= DL_MOTOR_STATE_HDRM_OPEN;
    if (motor_->isHdrmClosed()) state |= DL_MOTOR_STATE_HDRM_CLOSED;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    downlink_->sendMotor(position, speed, state, status1, 0);
}

uint8_t System::subsystemBits() const {
    uint8_t bits = 0;
    if (imu_ready_)           bits |= DL_SUBSYS_IMU;
    if (motor_ready_)         bits |= DL_SUBSYS_MOTOR;
    if (downlink_ready_)      bits |= DL_SUBSYS_DOWNLINK;
    if (weight_sensor_ready_) bits |= DL_SUBSYS_WEIGHT;
    if (force_sensor_ready_)  bits |= DL_SUBSYS_FORCE;
    return bits;
}

void System::handleSystemTelemetry(uint32_t now_ms) {
    if (!downlink_ready_ || !downlink_) return;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    const uint8_t rexus_bits = (rexus_ready_ && rexus_) ? rexus_->rawBits() : 0;

    downlink_->sendSystem(static_cast<uint8_t>(state_machine_.getState()),
                          subsystemBits(),
                          now_ms - startup_time,
                          status1, 0, rexus_bits);

    // Uplink-Empfangsstatistik hinterherschicken: zeigt am Boden, ob ein
    // Telecommand die UART ueberhaupt erreicht.
    if (uplink_) {
        downlink_->sendUplinkStats(uplink_->rxBytes(), uplink_->framesOk(),
                                   uplink_->framesBad(), uplink_->lastOpcode());
        downlink_->sendUplinkRaw(uplink_->burstBytes(), uplink_->burstLen());
    }
}

// ---------------------------------------------------------------------------
// Zustandsmaschine
// ---------------------------------------------------------------------------

void System::handleStateMachine(uint32_t now_ms) {
    StateMachineInputs in;

    if (rexus_ready_ && rexus_) {
        rexus_->update(now_ms);          // sampeln + entprellen, dann erst lesen
        rexus_->getAllSignals(in.l0, in.soe, in.sods);
    }
    in.operator_abort = abort_requested_;

    // TODO: force_load_n aus der Wegezelle des Greifers speisen, sobald sie
    // verbaut ist - Kraftsensor 2 misst an anderer Stelle.

    state_machine_.update(now_ms, in);

    const MissionState current = state_machine_.getState();
    if (current != last_state_) {
        onStateChanged(last_state_, current);
        last_state_ = current;
    }
}

void System::onStateChanged(MissionState previous, MissionState current) {
    (void)previous;

    // Aktoren stillsetzen, sobald der Freigabezustand (TEST) verlassen wird -
    // insbesondere wenn SODS den Bodentest abbricht oder ein ABORT kommt.
    if (!state_machine_.actuatorsUnlocked() && motor_ && motor_ready_) {
        motor_->off();
        Serial.printf("INFO  [System]: Aktoren gestoppt (Zustand %s).\n",
                      StateMachine::toString(current));
    }
}

// ---------------------------------------------------------------------------
// Telecommands
// ---------------------------------------------------------------------------

void System::handleUplink(uint32_t now_ms) {
    if (!uplink_) return;

    UplinkCommand cmd;
    while (uplink_->poll(cmd)) {
        const UplinkOpcode op = static_cast<UplinkOpcode>(cmd.opcode);

        // System-Kommandos sind immer erlaubt.
        switch (op) {
            case UplinkOpcode::TEST_ENTER:
                state_machine_.enterTest(now_ms);
                continue;
            case UplinkOpcode::TEST_EXIT:
                state_machine_.exitTest(now_ms);
                continue;
            case UplinkOpcode::ABORT:
                Serial.println("WARN  [System]: ABORT per Telecommand empfangen.");
                abort_requested_ = true;
                continue;

            case UplinkOpcode::MOTOR_OFF:
                // Ausschalten ist ein SICHERHEITSkommando und deshalb immer
                // erlaubt - anders als alle uebrigen Aktor-Kommandos. Lag es
                // hinter der TEST-Sperre, liesse sich ein laufender Motor
                // ausserhalb von TEST nur noch per ABORT stoppen, und ABORT ist
                // ein Endzustand, aus dem nur ein Reset herausfuehrt.
                if (motor_ready_ && motor_) {
                    Serial.println("INFO  [System]: TC MOTOR_OFF (zustandsunabhaengig)");
                    motor_->off();
                }
                continue;

            default:
                break;
        }

        // Alles Weitere greift auf Aktoren zu und ist nur im TEST-Zustand frei.
        if (!state_machine_.actuatorsUnlocked()) {
            Serial.printf("WARN  [System]: Telecommand 0x%02X abgewiesen - "
                          "Zustand %s, Aktoren gesperrt (TEST_ENTER senden).\n",
                          cmd.opcode, StateMachine::toString(state_machine_.getState()));
            continue;
        }

        if (!handleMotorCommand(cmd)) {
            Serial.printf("WARN  [System]: Unbekanntes Telecommand 0x%02X ignoriert.\n",
                          cmd.opcode);
        }
    }
}

bool System::handleMotorCommand(const UplinkCommand& cmd) {
    if (!motor_ready_ || !motor_) {
        Serial.println("WARN  [System]: Motor-Telecommand ohne initialisierten Motor - ignoriert.");
        return true;   // bekanntes Kommando, nur keine Hardware
    }

    switch (static_cast<UplinkOpcode>(cmd.opcode)) {
        case UplinkOpcode::MOTOR_ON:
            Serial.println("INFO  [System]: TC MOTOR_ON");
            motor_->on();
            return true;
        case UplinkOpcode::MOTOR_OFF:
            Serial.println("INFO  [System]: TC MOTOR_OFF");
            motor_->off();
            return true;
        case UplinkOpcode::MOTOR_HALF_TURN:
            Serial.println("INFO  [System]: TC MOTOR_HALF_TURN (relativ +180 Grad)");
            motor_->halfTurn();
            return true;
        case UplinkOpcode::HDRM_OPEN:
            Serial.println("INFO  [System]: TC HDRM_OPEN (absolut +180 Grad)");
            motor_->hdrmOpen();
            return true;
        case UplinkOpcode::HDRM_CLOSE:
            Serial.println("INFO  [System]: TC HDRM_CLOSE (absolut Nullposition)");
            motor_->hdrmClose();
            return true;
        case UplinkOpcode::MOTOR_ZERO:
            Serial.println("INFO  [System]: TC MOTOR_ZERO - aktuelle Position = HDRM geschlossen");
            motor_->zeroPosition();
            return true;
        default:
            return false;
    }
}

void System::handleWeightReading() {
    if (!weight_sensor_ready_ || !weight_sensor_) return;

    ScaleReading r;
    if (weight_sensor_->readScale1(r)) {
        Serial.printf("[WEIGHT] Gewicht: %8.2f g  |  raw (tare-bereinigt): %ld\n",
                      r.weight, static_cast<long>(r.raw_value));
    } else {
        Serial.println("[WEIGHT] Lesefehler – Sensor nicht bereit!");
    }
    Serial.println("---");
}

void System::handleForceReading() {
    if (!force_sensor_ready_ || !force_sensor_2_) return;

    ForceSensorReading reading;
    if (force_sensor_2_->read(reading)) {
        Serial.printf("[FORCE]  X: %8.2f  |  Y: %8.2f  |  Z: %8.2f  |  raw(X/Y/Z): %ld/%ld/%ld\n",
                      reading.force_x, reading.force_y, reading.force_z,
                      static_cast<long>(reading.raw_x),
                      static_cast<long>(reading.raw_y),
                      static_cast<long>(reading.raw_z));
    } else {
        Serial.println("[FORCE] Lesefehler – Sensor nicht bereit!");
    }
    Serial.println("---");
}

void System::printWelcomeBanner() {
    Serial.println("\n");
    Serial.println("╔═════════════════════════════════════════════════╗");
    Serial.println("║             MAGGIE On-Board Computer            ║");
    Serial.println("║     REXUS Program - Rocket Experiment System    ║");
    Serial.println("║                     v 1.0                       ║");
    Serial.println("╚═════════════════════════════════════════════════╝");
    Serial.println("");
    Serial.println("Built by MAGGIE Team on Teensy 4.1");
    Serial.println("Copyright 2026 - All Rights Reserved");
    Serial.println("");
}