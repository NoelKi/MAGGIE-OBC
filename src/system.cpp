#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

// ===========================================================================
// Intervalle (in Millisekunden)
// ===========================================================================
#define TELEMETRY_INTERVAL_MS 50        ///< Send IMU/Motor downlink every 50ms (20 Hz)
#define SYS_TELEMETRY_INTERVAL_MS 1000  ///< Send SYS/STATE downlink every 1s (1 Hz)
#define DOWNLINK_BAUDRATE 38400         ///< Downlink UART speed (Serial8, pins 34/35)

System::System() {
    // Konstruktor
}

System::~System() {
    delete imu_;
    delete downlink_;
    delete motor_;
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

    // Closed-Loop-Positionsregelung des Motors einen Schritt weiterführen
    if (motor_) motor_->update();

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

    uint8_t state = 0;
    if (motor_->isOn())     state |= DL_MOTOR_STATE_ON;
    if (motor_->isMoving()) state |= DL_MOTOR_STATE_MOVING;
    else                    state |= DL_MOTOR_STATE_AT_TARGET;

    // Stellung aus der Encoder-Position ableiten, damit die Bodenstation sie
    // nicht selbst aus den Counts rekonstruieren muss.
    if (motor_->isAtHalfTurn()) state |= DL_MOTOR_STATE_HDRM_OPEN;
    if (motor_->isAtZero())     state |= DL_MOTOR_STATE_HDRM_CLOSED;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    downlink_->sendMotor(position, speed, state, status1, 0);
}

uint8_t System::subsystemBits() const {
    uint8_t bits = 0;
    if (imu_ready_)      bits |= DL_SUBSYS_IMU;
    if (motor_ready_)    bits |= DL_SUBSYS_MOTOR;
    if (downlink_ready_) bits |= DL_SUBSYS_DOWNLINK;
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
        case UplinkOpcode::MOTOR_HALF_TURN:   // Altbestand, gleiche Fahrt wie 0x03
        case UplinkOpcode::HALF_TURN_FWD:
            Serial.println("INFO  [System]: TC HALF_TURN_FWD (relativ +180 Grad)");
            motor_->halfTurnForward();
            return true;
        case UplinkOpcode::HALF_TURN_REV:
            Serial.println("INFO  [System]: TC HALF_TURN_REV (relativ -180 Grad)");
            motor_->halfTurnReverse();
            return true;
        case UplinkOpcode::MOTOR_ZERO:
            Serial.println("INFO  [System]: TC MOTOR_ZERO - aktuelle Position = Nullpunkt");
            motor_->zeroPosition();
            return true;
        default:
            return false;
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
