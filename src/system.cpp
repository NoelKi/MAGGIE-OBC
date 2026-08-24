#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

// ===========================================================================
// Intervalle (in Millisekunden)
// ===========================================================================
#define TELEMETRY_INTERVAL_MS 50        ///< Send IMU/Motor downlink every 50ms (20 Hz)
#define SYS_TELEMETRY_INTERVAL_MS 1000  ///< Send SYS/STATE downlink every 1s (1 Hz)
#define DOWNLINK_BAUDRATE 38400         ///< Downlink UART speed (Serial4, pins 16/17)

System::System() {
    // Konstruktor
}

System::~System() {
    delete imu_;
    delete downlink_;
    delete motor_;
    delete force1_;
    delete force2_;
    delete uplink_;
    delete rexus_;
}

bool System::init() {
    startup_time = millis();

    printWelcomeBanner();

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
    // Telemetrie-Downlink (Serial4, Pin 16 RX = updownlink+ / 17 TX = updownlink-)
    // -----------------------------------------------------------------------
    Serial.printf("INFO  [System]: Initialisiere Telemetrie-Downlink (Serial4, Pin %u/%u)...\n",
                  PIN_UPDOWNLINK_PLUS, PIN_UPDOWNLINK_MINUS);
    downlink_ = new TelemetryDownlink(Serial4);
    downlink_ready_ = downlink_->init(DOWNLINK_BAUDRATE);
    if (downlink_ready_) {
        Serial.println("INFO  [System]: Downlink bereit (38400 Baud).");
    } else {
        Serial.println("WARN  [System]: Downlink konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Motor 1 (DRV8871, ungeregelt + Quadratur-Encoder als Sensor)
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
    // Kraftsensoren (je eine HX711-Gruppe an gemeinsamem Takt)
    //   Sensor 1: 3 Zellen X/Y/Z
    //   Sensor 2: 4 Zellen A/B/C/D (Eigenbau, Verrechnung erst am Boden)
    //
    // Der Nullabgleich laeuft in init() mit - die Zellen muessen dabei
    // UNBELASTET sein. Schlaegt er fehl, laeuft der Sensor ohne Tara weiter
    // und meldet das per DL_FORCE_TARED an die Bodenstation.
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Kraftsensor 1 (3x HX711, X/Y/Z)...");
    force1_ = new ForceHAL(PIN_FORCE1_DOUT, 3, PIN_FORCE1_SCK, FORCE1_TELE_DIV);
    force1_ready_ = force1_->init();
    logForceSensor(force1_, "Kraftsensor 1", PIN_FORCE1_DOUT, 3, PIN_FORCE1_SCK);

    Serial.println("INFO  [System]: Initialisiere Kraftsensor 2 (4x HX711, A/B/C/D)...");
    force2_ = new ForceHAL(PIN_FORCE2_DOUT, 4, PIN_FORCE2_SCK, FORCE2_TELE_DIV);
    force2_ready_ = force2_->init();
    logForceSensor(force2_, "Kraftsensor 2", PIN_FORCE2_DOUT, 4, PIN_FORCE2_SCK);

    // -----------------------------------------------------------------------
    // Uplink-Telecommand-Empfang (teilt sich Serial4 mit dem Downlink, Pin 16 RX)
    // -----------------------------------------------------------------------
    uplink_ = new UplinkReceiver(Serial4);

    // -----------------------------------------------------------------------
    // REXUS-Signale (L0 / SOE / SODS). Sie loesen in diesem Ausbau KEINE
    // Zustandswechsel aus - die Rohpegel gehen nur mit dem SYS/STATE-Frame an
    // die Bodenstation, damit die Verkabelung am Aufbau geprueft werden kann.
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
    // Zusammenfassung. Der OBC startet auch dann, wenn einzelne Subsysteme
    // fehlen - system_healthy meldet den Zustand nur an die Bodenstation.
    // -----------------------------------------------------------------------
    system_healthy = imu_ready_ && downlink_ready_ && motor_ready_;

    if (system_healthy) {
        Serial.println("INFO  [System]: System erfolgreich initialisiert.\n");
    } else {
        Serial.printf("WARN  [System]: System DEGRADIERT gestartet - IMU:%s Downlink:%s Motor:%s\n\n",
                      imu_ready_      ? "OK" : "FEHLT",
                      downlink_ready_ ? "OK" : "FEHLT",
                      motor_ready_    ? "OK" : "FEHLT");
    }
    return true;
}

void System::run() {
    const uint32_t now = millis();

    // Eingehende Telecommands verarbeiten (jeden Loop, geringe Latenz)
    handleUplink(now);

    // Zustand fortschreiben (Telecommands)
    handleStateMachine(now);

    // Fahrt am Ziel abschalten, Bremsimpuls beenden
    if (motor_) motor_->update();

    // Kraftsensor jeden Loop pollen und im Takt der Wandlung senden (10 Hz)
    handleForce();

    // Laufzeitbegrenzung des Motors prüfen
    handleMotorTimeout(now);

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

    // ENCODER_OK unterscheidet am Boden "Motor dreht nicht" von "Encoder ist
    // gar nicht angehaengt" - beides sieht in den Counts gleich aus.
    uint8_t state = 0;
    if (motor_->isOn())        state |= DL_MOTOR_STATE_ON;
    if (motor_->hasEncoder())  state |= DL_MOTOR_STATE_ENCODER_OK;
    if (motor_->isTurning())   state |= DL_MOTOR_STATE_TURNING;
    if (motor_->turnFailed())  state |= DL_MOTOR_STATE_TURN_FAILED;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    downlink_->sendMotor(position, speed, state, status1, 0);
}

void System::logForceSensor(ForceHAL* hal, const char* label,
                            const uint8_t* pins, uint8_t count, uint8_t sck) {
    if (!hal) return;

    if (!hal->tared()) {
        Serial.printf("WARN  [System]: %s antwortet nicht - kein Nullabgleich. "
                      "Verkabelung der Datenpins und der Taktleitung %u pruefen.\n",
                      label, sck);
        return;
    }

    // Die Nullpunkte gehoeren ins Protokoll: Sie sind der Bezug jeder spaeteren
    // Messung, und ein voellig abweichender Wert zwischen zwei Starts ist der
    // erste Hinweis auf eine belastete Zelle beim Booten.
    Serial.printf("INFO  [System]: %s bereit (Takt %u), Nullpunkte:", label, sck);
    for (uint8_t i = 0; i < count; i++) {
        Serial.printf(" Pin%u=%ld", pins[i], static_cast<long>(hal->offset(i)));
    }
    Serial.println();
}

void System::handleForce() {
    handleForceSensor(force1_, force1_ready_, DownlinkForceMsg::TARGET1, force1_state_);
    handleForceSensor(force2_, force2_ready_, DownlinkForceMsg::TARGET2, force2_state_);
}

void System::handleForceSensor(ForceHAL* hal, bool ready, DownlinkForceMsg msg,
                               ForceChannelState& state) {
    if (!ready || !hal) return;

    // read() kehrt ohne neue Wandlung sofort zurueck - der Aufruf jeden Loop
    // kostet also nur ein digitalRead() je Kanal. Kommt ein Wert, geht er
    // direkt raus: Der Downlink laeuft damit im Takt des Sensors (10 Hz) statt
    // in einem festen Intervall, das gegen die Wandlung schwebt und Werte
    // doppelt oder gar nicht sendet.
    const bool fresh = hal->read(state.last);
    const uint32_t now = millis();

    if (!fresh) {
        // Kein neuer Messwert. Solange der Sensor normal wandelt, ist das der
        // Regelfall zwischen zwei Samples - nichts zu tun.
        if (!hal->stalled()) return;

        // Sensor haengt (oder hat noch nie geantwortet). Trotzdem senden, mit
        // gesetztem STALE-Flag: Bliebe der Downlink hier still, saehe das am
        // Boden aus wie "Sensor okay, Kraft konstant" - waehrend das
        // Subsystembit im SYS-Frame weiter OK meldet. Ein Frame mit STALE sagt
        // dagegen genau, was los ist.
        //
        // Gedrosselt auf FORCE_STALE_TX_INTERVAL_MS: Ohne diese Bremse ginge
        // bei stehendem Wandler in JEDEM Loop ein Frame raus - das waere der
        // sicherste Weg, den 38400-Baud-Downlink dichtzumachen.
        if (now - state.last_tx_ms < FORCE_STALE_TX_INTERVAL_MS) return;
    }

    if (!downlink_ready_ || !downlink_) return;
    state.last_tx_ms = now;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    downlink_->sendForce(msg, state.last, hal->stalled(), hal->tared(), status1);
}

void System::tareForceSensor(ForceHAL* hal, bool ready, const char* label) {
    if (!ready || !hal) {
        Serial.printf("WARN  [System]: FORCE_TARE fuer %s - Sensor nicht da, ignoriert.\n",
                      label);
        return;
    }

    Serial.printf("INFO  [System]: TC FORCE_TARE - %s nullen "
                  "(Zellen muessen unbelastet sein)\n", label);
    if (!hal->tare()) {
        Serial.printf("WARN  [System]: FORCE_TARE fuer %s fehlgeschlagen - "
                      "Wandler antwortet nicht, alter Nullpunkt bleibt.\n", label);
        return;
    }

    Serial.printf("INFO  [System]: %s Nullpunkte:", label);
    for (uint8_t i = 0; i < hal->channels(); i++) {
        Serial.printf(" [%u]=%ld", i, static_cast<long>(hal->offset(i)));
    }
    Serial.println();
}

uint8_t System::subsystemBits() const {
    uint8_t bits = 0;
    if (imu_ready_)      bits |= DL_SUBSYS_IMU;
    if (motor_ready_)    bits |= DL_SUBSYS_MOTOR;
    if (downlink_ready_) bits |= DL_SUBSYS_DOWNLINK;
    if (force1_ready_)   bits |= DL_SUBSYS_FORCE1;
    if (force2_ready_)   bits |= DL_SUBSYS_FORCE2;
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
    // REXUS weiterhin sampeln und entprellen - die Rohpegel gehen mit dem
    // SYS/STATE-Frame nach unten, auch wenn sie nichts mehr auslösen.
    if (rexus_ready_ && rexus_) {
        rexus_->update(now_ms);
    }

    StateMachineInputs in;
    in.operator_abort = abort_requested_;

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
    // insbesondere bei TEST_EXIT und bei einem ABORT.
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

            case UplinkOpcode::FORCE_TARE:
                // Nullen ist ein reiner SENSOR-Eingriff, kein Aktor - deshalb
                // ausserhalb der TEST-Sperre. Es bewegt nichts und laesst sich
                // durch ein erneutes Tarieren jederzeit korrigieren.
                //
                // ARG waehlt den Sensor: 0 = beide, 1 = Sensor 1, 2 = Sensor 2.
                // Beide dauern zusammen bis zu 2x TARE_TIMEOUT_MS - das ist
                // hier vertretbar, weil in dieser Zeit nichts faehrt.
                if (cmd.arg == 0 || cmd.arg == 1) {
                    tareForceSensor(force1_, force1_ready_, "Kraftsensor 1");
                }
                if (cmd.arg == 0 || cmd.arg == 2) {
                    tareForceSensor(force2_, force2_ready_, "Kraftsensor 2");
                }
                if (cmd.arg < 0 || cmd.arg > 2) {
                    Serial.printf("WARN  [System]: FORCE_TARE mit ungueltigem ARG %d "
                                  "(0=beide, 1=Sensor 1, 2=Sensor 2) - ignoriert.\n",
                                  cmd.arg);
                }
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
            Serial.printf("INFO  [System]: TC MOTOR_ON (speed=%d)\n", cmd.arg);
            motor_->on(cmd.arg);
            // Sichtbar machen, wenn die Anlaufgrenze gegriffen hat - sonst
            // wundert man sich am Boden ueber den abweichenden PWM-Istwert.
            if (cmd.arg != 0 && motor_->getSpeed() != cmd.arg) {
                Serial.printf("WARN  [System]: PWM %d unter der Anlaufgrenze "
                              "(%d) - auf %d angehoben.\n",
                              cmd.arg, MotorHAL::MIN_DRIVE_SPEED, motor_->getSpeed());
            }
            motor_on_since_ms_ = millis();
            return true;
        case UplinkOpcode::MOTOR_TURN:
            Serial.printf("INFO  [System]: TC MOTOR_TURN (%d Grad relativ)\n", cmd.arg);
            if (!motor_->hasEncoder()) {
                Serial.println("WARN  [System]: MOTOR_TURN ohne Encoder - ignoriert.");
                return true;
            }
            motor_->turnBy(cmd.arg);
            // Der Watchdog gilt auch hier: Bleibt der Encoder stehen (Mechanik
            // fest, Kanal ab), erreicht update() sein Ziel nie.
            motor_on_since_ms_ = millis();
            return true;
        case UplinkOpcode::MOTOR_GOTO:
            Serial.printf("INFO  [System]: TC MOTOR_GOTO (%d Grad absolut)\n", cmd.arg);
            if (!motor_->hasEncoder()) {
                Serial.println("WARN  [System]: MOTOR_GOTO ohne Encoder - ignoriert.");
                return true;
            }
            motor_->goTo(cmd.arg);
            motor_on_since_ms_ = millis();
            return true;
        case UplinkOpcode::MOTOR_OFF:
            Serial.println("INFO  [System]: TC MOTOR_OFF");
            motor_->off();
            return true;
        case UplinkOpcode::MOTOR_ZERO:
            Serial.println("INFO  [System]: TC MOTOR_ZERO - Encoder-Zaehler auf 0");
            motor_->zeroPosition();
            return true;
        default:
            return false;
    }
}

void System::handleMotorTimeout(uint32_t now_ms) {
    // Ohne Positionsregelung endet eine Motorfahrt nicht mehr von selbst - der
    // Motor dreht, bis MOTOR_OFF kommt. Wenn ausgerechnet die Uplink-Strecke
    // der Prueflig ist, ist das am Tisch ein Risiko. Der Watchdog ist die
    // Rueckfallebene; MOTOR_ON_TIMEOUT_MS = 0 schaltet ihn ab.
    if (MOTOR_ON_TIMEOUT_MS == 0) return;
    if (!motor_ready_ || !motor_ || !motor_->isOn()) return;

    if (now_ms - motor_on_since_ms_ >= MOTOR_ON_TIMEOUT_MS) {
        Serial.printf("WARN  [System]: Motor-Laufzeit ueber %lu ms - "
                      "automatisch gestoppt. Erneut MOTOR_ON senden.\n",
                      static_cast<unsigned long>(MOTOR_ON_TIMEOUT_MS));
        motor_->off();
    }
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
