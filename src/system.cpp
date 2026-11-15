#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

// ===========================================================================
// Intervalle (in Millisekunden)
// ===========================================================================
// Datenraten und Baudrate: include/data_rates.hpp

// Zusaetzliche Serial4-Puffer. Der Core puffert nur 40 Byte Senden und 64 Byte
// Empfang. Ein 50-ms-Takt schickt aber 80 Byte (IMU + 2x Motor), jede volle
// Sekunde nochmals 60 - mit 40 Byte blockierte write() dann den Loop fuer
// 10-20 ms. Bei ~55 % Auslastung der 38400 Baud faengt der grosse Puffer jeden
// Burst ab: write() kehrt sofort zurueck, die Frames bleiben unveraendert.
static uint8_t serial4_tx_extra[512];
static uint8_t serial4_rx_extra[256];

// Serial1-Puffer fuer den Roboterarm. Empfang: Eine SD-Schreiboperation haelt
// den Loop bis zu 250 ms an - bei 115200 Baud sind das bis zu ~2,9 KB, die die
// RCU in der Zeit schicken koennte. 1 KB reicht fuer die ~3 KB/s, die sie
// realistisch sendet (RCU-IMU mit 104 Hz plus Gelenke). Senden: STOP/INHIBIT
// "an alle" sind 8 Frames a 8 Byte auf einmal.
static uint8_t serial1_rx_extra[1024];
static uint8_t serial1_tx_extra[256];

// 1 Grad in Mikroradiant (pi / 180 * 1e6) - die RCU rechnet in urad.
static constexpr float URAD_PER_DEG = 17453.2925f;

System::System() {
    // Konstruktor
}

System::~System() {
    delete imu_;
    delete downlink_;
    delete motor_;
    delete motor2_;
    delete force1_;
    delete force2_;
    delete uplink_;
    delete rexus_;
    delete arm_;
}

bool System::init(bool after_watchdog_reset) {
    startup_time = millis();

    printWelcomeBanner();
    // Build-Stempel: Ein Teensy laesst sich nicht auslesen - nur so ist im
    // Log erkennbar, welche Firmware tatsaechlich geflasht ist.
    Serial.printf("INFO  [System]: Firmware-Build %s %s\n", __DATE__, __TIME__);

    // -----------------------------------------------------------------------
    // Nichtfluechtiger Speicher - zuerst, damit Boot-Nummer und letzter
    // Zustand feststehen, bevor irgendetwas anderes startet.
    // -----------------------------------------------------------------------
    persist_.begin();
    Serial.printf("INFO  [System]: Boot Nr. %lu%s.\n",
                  static_cast<unsigned long>(persist_.bootCount()),
                  after_watchdog_reset ? " (nach Watchdog-Reset)" : "");

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
    downlink_ready_ = downlink_->init(rates::DOWNLINK_BAUDRATE);
    Serial4.addMemoryForWrite(serial4_tx_extra, sizeof(serial4_tx_extra));
    Serial4.addMemoryForRead(serial4_rx_extra, sizeof(serial4_rx_extra));   // Uplink

    // Bordspeicher: jedes gesendete Frame geht zusaetzlich in den SD-Puffer.
    // Fehlt die Karte, laeuft alles andere unveraendert weiter.
    sd_.begin(persist_.bootCount(), after_watchdog_reset);
    downlink_->setTap([](void* ctx, const uint8_t* frame, uint8_t len) {
        static_cast<SdLogger*>(ctx)->push(frame, len);
    }, &sd_);
    if (downlink_ready_) {
        Serial.printf("INFO  [System]: Downlink bereit (%lu Baud).\n",
                      static_cast<unsigned long>(rates::DOWNLINK_BAUDRATE));
    } else {
        Serial.println("WARN  [System]: Downlink konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Motor 1 (DRV8871, ungeregelt + Quadratur-Encoder als Sensor)
    // Fehler hier sind NICHT fatal: das System laeuft ohne Motor weiter.
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Motor 1 (DRV8871 + Encoder, sanfte Anfahrt)...");
    MotorHAL::logDriveProfile();
    motor_ = new MotorHAL(PIN_M1_A, PIN_M1_B, 1, /*soft_approach=*/true);
    motor_ready_ = motor_->init();
    if (motor_ready_) {
        motor_->initEncoder(PIN_M1_ENC_A, PIN_M1_ENC_B);
        Serial.printf("INFO  [System]: Motor 1 bereit (Treiber Pin %u/%u, Encoder Pin %u/%u).\n",
                      PIN_M1_A, PIN_M1_B, PIN_M1_ENC_A, PIN_M1_ENC_B);
    } else {
        Serial.println("WARN  [System]: Motor 1 konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Motor 2 (baugleich zu Motor 1, eigene PWM-/Encoder-Pins)
    // Fehler hier sind ebenfalls NICHT fatal.
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Motor 2 (DRV8871 + Encoder, sanfte Anfahrt)...");
    motor2_ = new MotorHAL(PIN_M2_A, PIN_M2_B, 2, /*soft_approach=*/true);
    motor2_ready_ = motor2_->init();
    if (motor2_ready_) {
        motor2_->initEncoder(PIN_M2_ENC_A, PIN_M2_ENC_B);
        Serial.printf("INFO  [System]: Motor 2 bereit (Treiber Pin %u/%u, Encoder Pin %u/%u).\n",
                      PIN_M2_A, PIN_M2_B, PIN_M2_ENC_A, PIN_M2_ENC_B);
    } else {
        Serial.println("WARN  [System]: Motor 2 konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Kraftsensoren (je eine HX711-Gruppe an gemeinsamem Takt)
    //   Sensor 1: 3 Zellen X/Y/Z
    //   Sensor 2: 4 Zellen A/B/C/D (Eigenbau, Verrechnung erst am Boden)
    //
    // Der Nullabgleich startet in init() und laeuft NICHT blockierend im Loop
    // mit (siehe handleForceSensor) - die Zellen muessen dabei UNBELASTET sein.
    // Schlaegt er fehl, laeuft der Sensor ohne Tara weiter und meldet das per
    // DL_FORCE_TARED an die Bodenstation.
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Kraftsensor 1 (3x HX711, X/Y/Z)...");
    force1_ = new ForceHAL(PIN_FORCE1_DOUT, 3, PIN_FORCE1_SCK, FORCE1_TELE_DIV);
    force1_ready_ = force1_->init();
    Serial.println("INFO  [System]: Kraftsensor 1 - Nullabgleich laeuft im Hintergrund.");

    Serial.println("INFO  [System]: Initialisiere Kraftsensor 2 (4x HX711, A/B/C/D)...");
    force2_ = new ForceHAL(PIN_FORCE2_DOUT, 4, PIN_FORCE2_SCK, FORCE2_TELE_DIV);
    force2_ready_ = force2_->init();
    Serial.println("INFO  [System]: Kraftsensor 2 - Nullabgleich laeuft im Hintergrund.");

    // -----------------------------------------------------------------------
    // Uplink-Telecommand-Empfang (teilt sich Serial4 mit dem Downlink, Pin 16 RX)
    // -----------------------------------------------------------------------
    uplink_ = new UplinkReceiver(Serial4);

    // -----------------------------------------------------------------------
    // Roboterarm (RCU an Serial1, Pin 0 RX / 1 TX). init() oeffnet nur die
    // UART - ob die RCU antwortet, steht in SYS/ARM_LINK. Kein Gelenk wird
    // hier freigegeben: Nach dem Einschalten stehen die Module in INHIBITED
    // (ICD 3.2), freigeben geht nur per ARM_RELEASE in TEST.
    // -----------------------------------------------------------------------
    Serial.printf("INFO  [System]: Initialisiere Roboterarm-UART (Serial1, Pin %u/%u, %lu Baud)...\n",
                  PIN_ARM_RX, PIN_ARM_TX, static_cast<unsigned long>(rates::ARM_BAUDRATE));
    arm_ = new ArmLink(Serial1);
    arm_ready_ = arm_->init(rates::ARM_BAUDRATE);
    Serial1.addMemoryForRead(serial1_rx_extra, sizeof(serial1_rx_extra));
    Serial1.addMemoryForWrite(serial1_tx_extra, sizeof(serial1_tx_extra));

    // -----------------------------------------------------------------------
    // REXUS-Signale (L0 / SOE / SODS). Sie loesen in diesem Ausbau KEINE
    // Zustandswechsel aus - die Rohpegel gehen nur mit dem SYS/STATE-Frame an
    // die Bodenstation, damit die Verkabelung am Aufbau geprueft werden kann.
    // -----------------------------------------------------------------------
    Serial.printf("INFO  [System]: Initialisiere REXUS-Signale (L0 %u / SOE %u / SODS %u)...\n",
                  PIN_L0_T, PIN_SOE_I, PIN_SODS_I);
    rexus_ = new REXUSHAL(PIN_L0_T, PIN_SOE_I, PIN_SODS_I);
    rexus_ready_ = rexus_->init();

    // -----------------------------------------------------------------------
    // Zustand des letzten Laufs wiederherstellen (siehe stateAfterReboot).
    // Im Flug geht es mit der gesicherten Zeit weiter: letzter Checkpoint plus
    // die Zeit seit diesem Reset (millis() laeuft ab dem Reset). Die Dauer
    // eines Stromausfalls selbst ist nicht messbar - der Ablauf ist danach um
    // hoechstens ~1 s plus diese Dauer verspaetet.
    // -----------------------------------------------------------------------
    uint8_t stored = 0;
    const bool stored_valid = persist_.loadState(stored);
    const MissionState start = StateMachine::stateAfterReboot(stored, stored_valid);
    const bool timed = StateMachine::isTimed(start);

    flight_flags_      = persist_.loadFlightFlags();
    last_checkpoint_s_ = timed ? persist_.loadCheckpoint() : 0;

    const uint32_t now = millis();
    const uint32_t elapsed = timed ? last_checkpoint_s_ * 1000UL + now : 0;
    state_machine_.init(start, now, elapsed);
    last_state_ = start;
    persist_.saveState(static_cast<uint8_t>(start));   // z.B. TEST -> PRE_LAUNCH festhalten
    if (start == MissionState::DESCENT) planDescent(now);   // Motoren wieder blockieren
    logStateEvent(start, 0, now);   // Startzustand als erstes Ereignis auf die SD-Karte

    Serial.printf("INFO  [System]: Zustandsmaschine gestartet in %s (gespeichert: %s, "
                  "Flugzeit %lu ms, HDRM-Flags 0x%02X).\n",
                  StateMachine::toString(start),
                  stored_valid ? StateMachine::toString(static_cast<MissionState>(stored))
                               : "nichts",
                  static_cast<unsigned long>(elapsed), flight_flags_);

    // -----------------------------------------------------------------------
    // Zusammenfassung. Der OBC startet auch dann, wenn einzelne Subsysteme
    // fehlen - system_healthy meldet den Zustand nur an die Bodenstation.
    // -----------------------------------------------------------------------
    system_healthy = imu_ready_ && downlink_ready_ && motor_ready_ && motor2_ready_ &&
                     force1_ready_ && force2_ready_ && rexus_ready_;

    if (system_healthy) {
        Serial.println("INFO  [System]: System erfolgreich initialisiert.\n");
    } else {
        Serial.printf("WARN  [System]: System DEGRADIERT gestartet - IMU:%s Downlink:%s "
                      "Motor1:%s Motor2:%s Kraft1:%s Kraft2:%s REXUS:%s\n\n",
                      imu_ready_      ? "OK" : "FEHLT",
                      downlink_ready_ ? "OK" : "FEHLT",
                      motor_ready_    ? "OK" : "FEHLT",
                      motor2_ready_   ? "OK" : "FEHLT",
                      force1_ready_   ? "OK" : "FEHLT",
                      force2_ready_   ? "OK" : "FEHLT",
                      rexus_ready_    ? "OK" : "FEHLT");
    }
    return true;
}

void System::run() {
    const uint32_t now = millis();

    // Eingehende Telecommands verarbeiten (jeden Loop, geringe Latenz)
    handleUplink(now);

    // Telemetrie der Arm-RCU einlesen (jeden Loop, damit der UART-Puffer
    // auch nach einer langen SD-Schreiboperation nicht ueberlaeuft)
    if (arm_) arm_->poll();

    // Zustand fortschreiben (Telecommands, REXUS-Signale, Flugzeit) und die
    // Aktionen des Flugablaufs ausfuehren
    handleStateMachine(now);
    handleFlight(now);

    // Trajektorien-Test an einem Gelenk. Laeuft nur in TEST: Beim Verlassen
    // beendet stopArm() ihn; die Abfrage hier ist die zweite Sicherung.
    if (arm_traj_.active()) {
        if (!state_machine_.actuatorsUnlocked()) {
            arm_traj_.cancel("Zustand nicht mehr TEST");
            stopArm();
        } else if (arm_ && arm_ready_) {
            arm_traj_.update(*arm_, now);
        }
    }
    // Phasenwechsel und Abbruch sofort melden (sonst 1 Hz mit SYS).
    if (arm_traj_.takeChanged()) sendArmTrajStatus();

    // Fahrt am Ziel abschalten, Bremsimpuls beenden
    if (motor_)  motor_->update();
    if (motor2_) motor2_->update();

    // Kraftsensor jeden Loop pollen und im Takt der Wandlung senden (10 Hz)
    handleForce();

    // Laufzeitbegrenzung des Motors prüfen
    handleMotorTimeout(now);

    // Telemetrie periodisch (Raten: data_rates.hpp). Jedes Frame geht an SD
    // und - falls die UART mitkommt - an den Downlink.
    if (now - last_telemetry_ms_ >= rates::IMU_INTERVAL_MS) {
        last_telemetry_ms_ = now;
        handleTelemetry();
    }
    if (now - last_motor_telemetry_ms_ >= rates::MOTOR_INTERVAL_MS) {
        last_motor_telemetry_ms_ = now;
        // Reihenfolge jedes Mal tauschen: Der Downlink ist ueberbucht (siehe
        // data_rates.hpp), und das ZWEITE Frame eines Paars findet die
        // Warteschlange fast immer voll vor. Mit fester Reihenfolge kam Motor 2
        // so nur mit ~1 Frame alle 4 s an (Motor 1: ~4/s) - die GUI sah den
        // Zustand "laeuft" nicht und schickte neue PWM-Sollwerte nicht ab.
        motor2_telemetry_first_ = !motor2_telemetry_first_;
        if (motor2_telemetry_first_) {
            handleMotorTelemetry(motor2_, motor2_ready_, DownlinkMotorMsg::STATE2);
            handleMotorTelemetry(motor_,  motor_ready_,  DownlinkMotorMsg::STATE);
        } else {
            handleMotorTelemetry(motor_,  motor_ready_,  DownlinkMotorMsg::STATE);
            handleMotorTelemetry(motor2_, motor2_ready_, DownlinkMotorMsg::STATE2);
        }
    }

    // Zustand + Subsystem-Gesundheit langsamer hinterherschicken (1 Hz)
    if (now - last_sys_telemetry_ms_ >= rates::SYS_INTERVAL_MS) {
        last_sys_telemetry_ms_ = now;
        handleSystemTelemetry(now);
    }

    // Roboterarm (eigene, niedrige Raten - siehe data_rates.hpp)
    handleArmTelemetry(now);

    // Flugzeit sichern, dann Bordspeicher: beides nie bei laufendem Motor.
    updateCheckpoint(now);
    sd_.service(now, !motorsRunning());
}

bool System::motorsRunning() const {
    return (motor_ && motor_->isOn()) || (motor2_ && motor2_->isOn());
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

void System::handleMotorTelemetry(MotorHAL* motor, bool ready, DownlinkMotorMsg msg) {
    if (!downlink_ready_ || !ready || !motor || !downlink_) return;

    const int32_t position = static_cast<int32_t>(motor->getPosition());
    const int16_t speed    = motor->getSpeed();

    // ENCODER_OK unterscheidet am Boden "Motor dreht nicht" von "Encoder ist
    // gar nicht angehaengt" - beides sieht in den Counts gleich aus.
    uint8_t state = 0;
    if (motor->isOn())        state |= DL_MOTOR_STATE_ON;
    if (motor->hasEncoder())  state |= DL_MOTOR_STATE_ENCODER_OK;
    if (motor->isTurning())   state |= DL_MOTOR_STATE_TURNING;
    if (motor->turnFailed())  state |= DL_MOTOR_STATE_TURN_FAILED;

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    downlink_->sendMotor(msg, position, speed, state, status1, 0);
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
    handleForceSensor(force1_, force1_ready_, DownlinkForceMsg::TARGET1, force1_state_,
                      "Kraftsensor 1", PIN_FORCE1_DOUT, PIN_FORCE1_SCK);
    handleForceSensor(force2_, force2_ready_, DownlinkForceMsg::TARGET2, force2_state_,
                      "Kraftsensor 2", PIN_FORCE2_DOUT, PIN_FORCE2_SCK);
}

void System::handleForceSensor(ForceHAL* hal, bool ready, DownlinkForceMsg msg,
                               ForceChannelState& state, const char* label,
                               const uint8_t* pins, uint8_t sck) {
    if (!ready || !hal) return;

    // read() kehrt ohne neue Wandlung sofort zurueck - der Aufruf jeden Loop
    // kostet also nur ein digitalRead() je Kanal. Kommt ein Wert, geht er
    // direkt raus: Der Downlink laeuft damit im Takt des Sensors (10 Hz) statt
    // in einem festen Intervall, das gegen die Wandlung schwebt und Werte
    // doppelt oder gar nicht sendet.
    const bool fresh = hal->read(state.last);
    const uint32_t now = millis();

    // Ein im Hintergrund laufender Nullabgleich (Boot oder Telecommand) endet
    // innerhalb von read(); das Ergebnis hier genau einmal protokollieren.
    bool tare_ok = false;
    if (hal->takeTareResult(tare_ok)) {
        logForceSensor(hal, label, pins, hal->channels(), sck);
    }

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

    Serial.printf("INFO  [System]: TC FORCE_TARE - %s wird im Hintergrund genullt "
                  "(Zellen muessen unbelastet sein)\n", label);
    // Kehrt sofort zurueck - der Abgleich laeuft ueber die naechsten Loops, das
    // Ergebnis wird in handleForceSensor() protokolliert.
    if (!hal->startTare()) {
        Serial.printf("WARN  [System]: FORCE_TARE fuer %s nicht gestartet.\n", label);
    }
}

uint8_t System::subsystemBits() const {
    uint8_t bits = 0;
    if (imu_ready_)      bits |= DL_SUBSYS_IMU;
    if (motor_ready_)    bits |= DL_SUBSYS_MOTOR;
    if (downlink_ready_) bits |= DL_SUBSYS_DOWNLINK;
    if (force1_ready_)   bits |= DL_SUBSYS_FORCE1;
    if (force2_ready_)   bits |= DL_SUBSYS_FORCE2;
    if (motor2_ready_)   bits |= DL_SUBSYS_MOTOR2;
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

    // Arm-Link-Statistik: zeigt am Boden, ob die RCU ueberhaupt antwortet.
    if (arm_ && arm_ready_) downlink_->sendArmLink(*arm_, now_ms);
    sendArmTrajStatus();
}

void System::sendArmTrajStatus() {
    if (!downlink_ready_ || !downlink_ || !arm_ || !arm_ready_) return;
    downlink_->sendArmTraj(static_cast<uint8_t>(arm_traj_.phase()),
                           static_cast<uint8_t>(arm_traj_.result()),
                           arm_traj_.resultNode(), arm_traj_.step(),
                           arm_traj_.maxErrDeg(), arm_traj_.runs());
}

// ---------------------------------------------------------------------------
// Roboterarm
// ---------------------------------------------------------------------------

void System::handleArmTelemetry(uint32_t now_ms) {
    if (!downlink_ready_ || !downlink_ || !arm_ || !arm_ready_) return;

    // Nur Gelenke mit NEUEM Wert senden: Antwortet die RCU nicht, faellt die
    // Kurve am Boden ab, statt mit dem letzten Wert scheinbar weiterzulaufen.
    if (now_ms - last_arm_joint_ms_ >= rates::ARM_JOINT_INTERVAL_MS) {
        last_arm_joint_ms_ = now_ms;
        for (uint8_t node = 1; node <= ARM_MAX_JOINTS; node++) {
            ArmJoint* j = arm_->joint(node);
            if (!j->state_new) continue;
            j->state_new = false;
            downlink_->sendArmJoint(node, *j);
        }
    }

    if (now_ms - last_arm_imu_ms_ >= rates::ARM_IMU_INTERVAL_MS) {
        last_arm_imu_ms_ = now_ms;
        ArmRcuImu& imu = arm_->rcuImu();
        if (imu.fresh) {
            imu.fresh = false;
            downlink_->sendArmRcuImu(imu);
        }
    }

    if (now_ms - last_arm_power_ms_ >= rates::ARM_POWER_INTERVAL_MS) {
        last_arm_power_ms_ = now_ms;
        for (uint8_t node = 1; node <= ARM_MAX_JOINTS; node++) {
            ArmJoint* j = arm_->joint(node);
            if (j->voltage_new) {
                j->voltage_new = false;
                downlink_->sendArmVoltage(node, *j);
            }
            if (j->temp_new) {
                j->temp_new = false;
                downlink_->sendArmTemp(node, *j);
            }
        }
    }
}

void System::stopArm() {
    arm_traj_.cancel("Arm gestoppt (TEST verlassen / ABORT)");
    if (!arm_ || !arm_ready_) return;
    arm_->stopAll();
    arm_->inhibitAll();
}

/// Gelenk aus dem ARG eines Arm-Kommandos (High-Byte). 0 = alle.
static uint8_t armNode(const UplinkCommand& cmd) {
    return static_cast<uint8_t>(static_cast<uint16_t>(cmd.arg) >> 8);
}

/// Wert aus dem ARG eines Arm-Kommandos (Low-Byte).
static uint8_t armValue(const UplinkCommand& cmd) {
    return static_cast<uint8_t>(cmd.arg & 0xFF);
}

bool System::handleArmCommand(const UplinkCommand& cmd) {
    const UplinkOpcode op = static_cast<UplinkOpcode>(cmd.opcode);
    if (op != UplinkOpcode::ARM_RELEASE && op != UplinkOpcode::ARM_JOG &&
        op != UplinkOpcode::ARM_SET_VELOCITY && op != UplinkOpcode::ARM_TRAJ_TEST) {
        return false;
    }
    if (!arm_ || !arm_ready_) {
        Serial.println("WARN  [System]: Arm-Telecommand ignoriert - Arm-UART nicht bereit.");
        return true;
    }
    // Waehrend der Trajektorie keine weiteren Fahrbefehle - sie wuerden das
    // Ziel des Players ueberschreiben. Abbrechen geht mit ARM_STOP/ARM_INHIBIT.
    if (arm_traj_.active()) {
        Serial.printf("WARN  [System]: Arm-Telecommand 0x%02X ignoriert - Trajektorie laeuft "
                      "(abbrechen: ARM_STOP).\n", cmd.opcode);
        return true;
    }

    // Freigeben und Fahren nur fuer EIN bestimmtes Gelenk, nie "alle".
    const uint8_t node = armNode(cmd);
    if (node < 1 || node > ARM_MAX_JOINTS) {
        Serial.printf("WARN  [System]: Arm-Telecommand 0x%02X mit ungueltigem Gelenk %u "
                      "(1..%u) - ignoriert.\n", cmd.opcode, node, ARM_MAX_JOINTS);
        return true;
    }

    switch (op) {
        case UplinkOpcode::ARM_RELEASE:
            Serial.printf("INFO  [System]: TC ARM_RELEASE Gelenk %u\n", node);
            arm_->sendCommand(node, ArmMsg::RELEASE);
            return true;

        case UplinkOpcode::ARM_JOG: {
            const int8_t deg = static_cast<int8_t>(armValue(cmd));
            if (deg == 0 || deg > ARM_JOG_MAX_DEG || deg < -ARM_JOG_MAX_DEG) {
                Serial.printf("WARN  [System]: ARM_JOG Gelenk %u mit %d Grad ausserhalb "
                              "+-1..%d - ignoriert.\n", node, deg, ARM_JOG_MAX_DEG);
                return true;
            }
            const int32_t urad = static_cast<int32_t>(lroundf(deg * URAD_PER_DEG));
            // Das Modul nimmt einen Fahrbefehl nur direkt nach RELEASE an -
            // deshalb geht vor JEDEM Jog ein RELEASE an dasselbe Gelenk. Beide
            // Frames liegen direkt hintereinander im TX-Puffer, die RCU bekommt
            // sie also immer in dieser Reihenfolge. Passt RELEASE nicht mehr in
            // den Puffer, unterbleibt auch der Jog (sonst faehrt er ins Leere).
            // Lockert keine Sperre: ARM_JOG kommt hier nur in TEST an.
            Serial.printf("INFO  [System]: TC ARM_JOG Gelenk %u um %d Grad (%ld urad), "
                          "vorher RELEASE\n", node, deg, static_cast<long>(urad));
            if (!arm_->sendCommand(node, ArmMsg::RELEASE)) {
                Serial.printf("WARN  [System]: ARM_JOG Gelenk %u - RELEASE passte nicht in "
                              "den TX-Puffer, Jog nicht gesendet.\n", node);
                return true;
            }
            arm_->sendMoveRel(node, urad);
            return true;
        }

        case UplinkOpcode::ARM_SET_VELOCITY: {
            const uint8_t deg_s = armValue(cmd);
            if (deg_s == 0) {
                Serial.printf("WARN  [System]: ARM_SET_VELOCITY Gelenk %u mit 0 Grad/s - ignoriert.\n",
                              node);
                return true;
            }
            // sendSetVelocity rechnet in urad/s und urad/s^2 um.
            // Vorgabe Team: hoechstens 6 Grad/s und 2 Grad/s^2 (RCU-Standard) -
            // dieselben Grenzen wie der Trajektorientest.
            if (deg_s > ARM_JOG_MAX_VEL_DEG_S) {
                Serial.printf("WARN  [System]: ARM_SET_VELOCITY Gelenk %u mit %u Grad/s ueber %u - ignoriert.\n",
                              node, deg_s, ARM_JOG_MAX_VEL_DEG_S);
                return true;
            }
            const uint8_t accel = deg_s < ARM_JOG_MAX_ACCEL_DEG_S2 ? deg_s : ARM_JOG_MAX_ACCEL_DEG_S2;
            Serial.printf("INFO  [System]: TC ARM_SET_VELOCITY Gelenk %u auf %u Grad/s, %u Grad/s^2\n",
                          node, deg_s, accel);
            arm_->sendSetVelocity(node, deg_s, accel);
            return true;
        }

        case UplinkOpcode::ARM_TRAJ_TEST: {
            // High-Byte = Gelenk fuer Spalte 1, Low-Byte = Gelenk fuer Spalte 2
            // (0 = nur ein Gelenk). Node 0 = "alle" ist oben schon abgewiesen.
            // Hoechstens zwei Gelenke - Freigabe des Teams, CLAUDE.md Regel 2.
            const uint8_t node2 = armValue(cmd);
            if (node2 > ARM_MAX_JOINTS) {
                Serial.printf("WARN  [System]: ARM_TRAJ_TEST mit ungueltigem 2. Gelenk %u - ignoriert.\n",
                              node2);
                return true;
            }
            const uint8_t nodes[2] = { node, node2 };
            const uint8_t count = node2 ? 2 : 1;
            Serial.printf("INFO  [System]: TC ARM_TRAJ_TEST Gelenk %u%s", node, node2 ? " + " : "\n");
            if (node2) Serial.printf("%u\n", node2);
            arm_traj_.start(*arm_, nodes, count, millis());
            return true;
        }

        default:
            return false;
    }
}

// ---------------------------------------------------------------------------
// Zustandsmaschine
// ---------------------------------------------------------------------------

void System::handleStateMachine(uint32_t now_ms) {
    // Entprellen; die Rohpegel gehen ausserdem mit SYS/STATE nach unten.
    if (rexus_ready_ && rexus_) rexus_->update(now_ms);

    // 1. Wechsel aus Telecommands (handleUplink) ZUERST uebernehmen. Sonst
    //    saehe update() unten z.B. nach STATE_RESET ein noch anliegendes SODS,
    //    bevor onStateChanged() es als "erst nach Abfallen wieder gueltig"
    //    markiert hat - und schaltete im selben Loop zurueck nach ARMED.
    commitStateChange(now_ms);

    const uint8_t asserted = currentSignalBits();

    // Ein Signal, das bei der Rueckkehr nach PRE_LAUNCH noch anlag (z.B. nach
    // der Landung), zaehlt erst wieder, nachdem es abgefallen ist - sonst
    // startete STATE_RESET sofort den naechsten Flug. Abgefallene Bits werden
    // geloescht; gespeichert nur bei stehenden Motoren (Flash, siehe oben).
    const uint8_t ignore = flight_flags_ & FLIGHT_IGNORE_MASK;
    const uint8_t still  = ignore & asserted;
    if (still != ignore && !motorsRunning()) {
        flight_flags_ = (flight_flags_ & ~FLIGHT_IGNORE_MASK) | still;
        persist_.saveFlightFlags(flight_flags_);
    }
    const uint8_t effective = asserted & ~(flight_flags_ & FLIGHT_IGNORE_MASK);

    StateMachineInputs in;
    in.sods = effective & FLIGHT_IGNORE_SODS;
    in.lo   = effective & FLIGHT_IGNORE_LO;
    in.soe  = effective & FLIGHT_IGNORE_SOE;
    in.hdrms_open_done = hdrmsOpenDone();

    state_machine_.update(now_ms, in);

    // 2. Wechsel aus update() (Signale, Zeit) uebernehmen.
    commitStateChange(now_ms);

    // Nur ein Signal hat sich geaendert (ohne Zustandswechsel): trotzdem melden.
    if (asserted != last_event_signals_) reportChange(last_state_, now_ms);
}

void System::commitStateChange(uint32_t now_ms) {
    const MissionState current  = state_machine_.getState();
    const MissionState previous = last_state_;
    if (current == previous) return;

    onStateChanged(previous, current, now_ms);
    last_state_ = current;
    reportChange(previous, now_ms);
}

void System::reportChange(MissionState previous, uint32_t now_ms) {
    // Ereignis-Frame auf die SD-Karte (ms-genau) und SOFORT ein SYS-Frame in
    // den Downlink - nicht erst zum naechsten 1-s-Takt. So sieht der Boden
    // z.B. LO so schnell wie moeglich.
    logStateEvent(previous, last_event_signals_, now_ms);
    last_sys_telemetry_ms_ = now_ms;
    handleSystemTelemetry(now_ms);
}

void System::logStateEvent(MissionState previous, uint8_t prev_signals, uint32_t now_ms) {
    const uint8_t signals = currentSignalBits();
    last_event_signals_ = signals;
    if (!downlink_) return;
    downlink_->logEvent(static_cast<uint8_t>(state_machine_.getState()),
                        static_cast<uint8_t>(previous),
                        eventSignals(signals), eventSignals(prev_signals), now_ms);
}

uint8_t System::eventSignals(uint8_t flight_bits) {
    // FLIGHT_IGNORE_*-Bits -> Bitlage der REXUS-Rohbits (Bit 0 LO, 1 SOE, 2 SODS).
    uint8_t out = 0;
    if (flight_bits & FLIGHT_IGNORE_LO)   out |= DL_REXUS_L0;
    if (flight_bits & FLIGHT_IGNORE_SOE)  out |= DL_REXUS_SOE;
    if (flight_bits & FLIGHT_IGNORE_SODS) out |= DL_REXUS_SODS;
    return out;
}

void System::onStateChanged(MissionState previous, MissionState current, uint32_t now_ms) {
    // Flug beginnt = Wechsel vom Boden in einen Flugzustand. Das ist meist
    // ARMED, aber direkt ASCENT/EXPERIMENT, wenn SODS bzw. LO gefehlt hat.
    const bool flight_begins = StateMachine::isGround(previous) &&
                               !StateMachine::isGround(current);

    // 1. Motoren - ZUERST, damit die Flash-Schreibzugriffe unten keinen
    //    laufenden Motor treffen (sie sperren die Encoder-Interrupts).
    //    Noch geplante HDRM-Fahrten verfallen (z.B. STATE_RESET mitten im
    //    Ablauf). Stopp bei: TEST verlassen, ABORT, Flugbeginn und jeder
    //    Rueckkehr nach PRE_LAUNCH - off() loest dabei auch die Sperre aus
    //    DESCENT, die Motoren laufen danach frei aus.
    hdrm_move_[0].pending = false;
    hdrm_move_[1].pending = false;
    if (previous == MissionState::TEST || current == MissionState::ABORT ||
        current == MissionState::PRE_LAUNCH || flight_begins) {
        if (motor_  && motor_ready_)  motor_->off();
        if (motor2_ && motor2_ready_) motor2_->off();
        stopArm();
        Serial.printf("INFO  [System]: Aktoren gestoppt (Zustand %s).\n",
                      StateMachine::toString(current));
    }

    // 2. Flugdaten - VOR dem Zustand speichern. Faellt der Strom dazwischen,
    //    startet der OBC im alten Zustand mit passenden Daten statt im neuen
    //    mit falschen.
    if (flight_begins) {
        flight_flags_ = 0;                       // neuer Flug: keine HDRM-Fahrt bisher
        persist_.saveFlightFlags(flight_flags_);
    }
    if (current == MissionState::PRE_LAUNCH) {
        // Zurueck am Boden. Was jetzt noch anliegt, muss erst abfallen, bevor
        // es wieder einen Flug startet (siehe handleStateMachine).
        flight_flags_ = currentSignalBits();
        persist_.saveFlightFlags(flight_flags_);
    }
    if (flight_begins || current == MissionState::PRE_LAUNCH ||
        current == MissionState::ASCENT || current == MissionState::EXPERIMENT) {
        last_checkpoint_s_ = 0;                  // neuer Zeitbezug (LO bzw. t0)
        persist_.saveCheckpoint(0);
    }
    if (current == MissionState::DESCENT) planDescent(now_ms);

    // 3. Zustand selbst
    persist_.saveState(static_cast<uint8_t>(current));
}

// ---------------------------------------------------------------------------
// Flugablauf
// ---------------------------------------------------------------------------

void System::handleFlight(uint32_t now_ms) {
    const MissionState state = state_machine_.getState();
    const uint32_t t = state_machine_.elapsed(now_ms);

    // HDRMs oeffnen. Die Fahrt wird VOR dem Start im EEPROM vermerkt (die
    // Motoren stehen dann noch) und laeuft hoechstens einmal pro Flug - auch
    // wenn ein Neustart sie unterbricht. Ein zweiter Versuch aus unbekannter
    // Position koennte das HDRM gegen den Anschlag fahren.
    if (state == MissionState::EXPERIMENT && t >= timeline::HDRM_OPEN_AT_MS &&
        !(flight_flags_ & FLIGHT_HDRM_OPENED) && !motorsRunning()) {
        flight_flags_ |= FLIGHT_HDRM_OPENED;
        persist_.saveFlightFlags(flight_flags_);
        Serial.printf("INFO  [System]: t0+%lu ms - HDRMs oeffnen.\n",
                      static_cast<unsigned long>(t));
        scheduleHdrmMoves(now_ms, timeline::HDRM_OPEN_DEG);
    }

    // Geplante Fahrten starten, sobald faellig. Gleicher Weg wie MOTOR_TURN
    // im Testmodus (execMotorAction).
    MotorHAL* const motors[2] = { motor_, motor2_ };
    const bool ready[2]       = { motor_ready_, motor2_ready_ };
    uint32_t* const on_since[2] = { &motor_on_since_ms_, &motor2_on_since_ms_ };
    const char* const labels[2] = { "HDRM1", "HDRM2" };
    for (uint8_t i = 0; i < 2; i++) {
        HdrmMove& move = hdrm_move_[i];
        if (!move.pending || static_cast<int32_t>(now_ms - move.due_ms) < 0) continue;
        move.pending = false;
        execMotorAction(motors[i], ready[i], labels[i], MotorAction::TURN,
                        move.degrees, *on_since[i]);
    }

    if (state == MissionState::ROBOT_ARM) robotArmStep(t);

    // DESCENT: jeden Motor blockieren, sobald seine Schliessfahrt vorbei ist.
    if (state == MissionState::DESCENT) {
        for (uint8_t i = 0; i < 2; i++) {
            MotorHAL* m = motors[i];
            if (!ready[i] || !m || hdrm_move_[i].pending || m->isTurning() || m->isLocked()) continue;
            m->lock();
            Serial.printf("INFO  [System]: %s blockiert (alle Motor-Pins HIGH).\n", labels[i]);
        }
    }
}

void System::scheduleHdrmMoves(uint32_t now_ms, int16_t degrees) {
    hdrm_move_[0] = { true, now_ms, degrees };
    hdrm_move_[1] = { true, now_ms + timeline::HDRM_STAGGER_MS, degrees };
}

void System::planDescent(uint32_t now_ms) {
    // Nur schliessen, was geoeffnet wurde - und nur einmal pro Flug (siehe
    // handleFlight). Danach blockiert handleFlight() beide Motoren.
    if (!(flight_flags_ & FLIGHT_HDRM_OPENED) || (flight_flags_ & FLIGHT_HDRM_CLOSED)) return;
    flight_flags_ |= FLIGHT_HDRM_CLOSED;
    persist_.saveFlightFlags(flight_flags_);
    Serial.println("INFO  [System]: DESCENT - HDRMs schliessen.");
    scheduleHdrmMoves(now_ms, timeline::HDRM_CLOSE_DEG);
}

uint8_t System::currentSignalBits() const {
    if (!rexus_ready_ || !rexus_) return 0;
    uint8_t bits = 0;
    if (rexus_->readSODS()) bits |= FLIGHT_IGNORE_SODS;
    if (rexus_->readL0T())  bits |= FLIGHT_IGNORE_LO;
    if (rexus_->readSOE())  bits |= FLIGHT_IGNORE_SOE;
    return bits;
}

bool System::hdrmsOpenDone() const {
    if (!(flight_flags_ & FLIGHT_HDRM_OPENED)) return false;
    if (hdrm_move_[0].pending || hdrm_move_[1].pending) return false;
    return !(motor_ && motor_->isTurning()) && !(motor2_ && motor2_->isTurning());
}

void System::robotArmStep(uint32_t t_ms) {
    // PLATZHALTER - Roboterarm noch nicht implementiert. Wird jeden Loop in
    // ROBOT_ARM mit der Zeit seit t0 aufgerufen; um t0 + DESCENT_AT_MS
    // wechselt die Zustandsmaschine selbst nach DESCENT.
    //
    // Beim Implementieren beachten (Ergebnis der Vorbesprechung):
    //  - Trajektorie: EINE Datei auf der SD-Karte, in Segmente (je Bewegung)
    //    unterteilt, mit Header + CRC32. Beim Booten komplett ins RAM laden und
    //    pruefen - im Flug die Karte nur schreiben, nie lesen (100-250 ms
    //    Busy-Zeiten). Optional: eingebaute Standard-Trajektorie im Flash als
    //    Rueckfallebene, falls Datei fehlt oder CRC nicht passt.
    //  - Bewegungen sind DATEN, keine MissionStates: Segment-Index innerhalb
    //    von ROBOT_ARM.
    //  - Der Arm meldet seine Position: Segment erst fertig, wenn die Position
    //    im Toleranzfenster des Endpunkts liegt; pro Segment ein Zeitlimit,
    //    danach weiter zum naechsten. Keine Antwort = Arm ausgefallen, der
    //    Zeitplan laeuft ohne ihn weiter.
    //  - Neustart: Position vom Arm lesen und von dort fortsetzen; den
    //    Segment-Index zusaetzlich an Segmentgrenzen im EEPROM sichern
    //    (nur wenn der Arm steht - Flash sperrt Interrupts).
    //  - Letztes Segment = Verstauen, mit Abstand VOR DESCENT_AT_MS. Die HDRMs
    //    schliessen um t0 + 115 s - vorher pruefen, ob der Arm verstaut ist.
    //    OFFEN: Was tun, wenn er es nicht ist? (Mechanik klaeren)
    //  - Der Arm hat eine eigene Steuerplatine; der OBC schickt nur
    //    Positionsbefehle per UART (max. 10 Hz). Deshalb NICHT in
    //    motorsRunning() aufnehmen - sonst stuende der SD-Bordspeicher die
    //    ganze Arm-Phase still. Folgen davon:
    //      * Eine SD-Schreiboperation kann den Loop 100-250 ms anhalten - ein
    //        Befehl kommt dann entsprechend spaeter. Die Arm-Platine muss das
    //        abfangen (interpolieren bzw. letztes Ziel halten), nicht auf
    //        exakte 100-ms-Schritte bauen.
    //      * Checkpoint-Writes (Flash) sperren kurz die Interrupts - Bytes VOM
    //        Arm koennen dabei verloren gehen. Antworten deshalb mit Start-
    //        Marker + CRC rahmen (wie der Uplink, der sich selbst resynct).
    //  - Die Arm-Telemetrie (Gelenkwinkel, RCU-IMU, Link) geht bereits per
    //    handleArmTelemetry() in Downlink und SD-Karte. Befehle an den Arm:
    //    arm_->sendMoveRel() / sendCommand() (arm_hal.hpp).
    (void)t_ms;
}

void System::updateCheckpoint(uint32_t now_ms) {
    if (!StateMachine::isTimed(state_machine_.getState())) return;
    if (motorsRunning()) return;   // Flash sperrt Interrupts - siehe persistent_state.hpp

    const uint32_t s = state_machine_.elapsed(now_ms) / 1000;
    const uint16_t seconds = s > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(s);
    if (seconds == last_checkpoint_s_) return;
    last_checkpoint_s_ = seconds;
    persist_.saveCheckpoint(seconds);
}

bool System::commandAllowed(UplinkOpcode op) const {
    // STATE_RESET geht immer - reines Testwerkzeug, um aus jedem Zustand
    // zurueck nach PRE_LAUNCH zu kommen. Im Flug gibt es ohnehin keinen Uplink.
    if (op == UplinkOpcode::STATE_RESET) return true;

    switch (state_machine_.getState()) {
        // Ab LO: keinerlei Eingriff - kein Abbruch, kein Neustart, kein
        // vorzeitiges Schliessen. Das Experiment fliegt, wie es geplant ist.
        case MissionState::ASCENT:
        case MissionState::EXPERIMENT:
        case MissionState::ROBOT_ARM:
            return false;

        // Nach dem Experiment nur STATE_RESET (oben). MOTOR_OFF wuerde die
        // Motorsperre loesen und ist deshalb hier gesperrt.
        case MissionState::DESCENT:
            return false;

        // Rampe vor LO: Startabbruch muss moeglich sein, Motorfahrten nicht.
        case MissionState::ARMED:
            return op == UplinkOpcode::ABORT ||
                   op == UplinkOpcode::MOTOR_OFF || op == UplinkOpcode::MOTOR2_OFF ||
                   op == UplinkOpcode::ARM_STOP || op == UplinkOpcode::ARM_INHIBIT ||
                   op == UplinkOpcode::FORCE_TARE;

        // Boden: alles; Motorfahrten prueft handleUplink() weiter auf TEST.
        case MissionState::PRE_LAUNCH:
        case MissionState::TEST:
        case MissionState::ABORT:
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Telecommands
// ---------------------------------------------------------------------------

void System::handleUplink(uint32_t now_ms) {
    if (!uplink_) return;

    UplinkCommand cmd;
    while (uplink_->poll(cmd)) {
        const UplinkOpcode op = static_cast<UplinkOpcode>(cmd.opcode);

        if (!commandAllowed(op)) {
            Serial.printf("WARN  [System]: Telecommand 0x%02X abgewiesen - im Zustand %s "
                          "nicht zulaessig.\n",
                          cmd.opcode, StateMachine::toString(state_machine_.getState()));
            continue;
        }

        // System- und Sicherheitskommandos (Zustand pruefen die Funktionen selbst).
        switch (op) {
            case UplinkOpcode::TEST_ENTER:
                state_machine_.enterTest(now_ms);
                continue;
            case UplinkOpcode::TEST_EXIT:
                state_machine_.exitTest(now_ms);
                continue;
            case UplinkOpcode::ABORT:
                Serial.println("WARN  [System]: ABORT per Telecommand empfangen.");
                state_machine_.abort(now_ms);
                continue;
            case UplinkOpcode::STATE_RESET:
                state_machine_.resetToPreLaunch(now_ms);
                continue;

            case UplinkOpcode::FORCE_TARE:
                // Nullen ist ein reiner SENSOR-Eingriff, kein Aktor - deshalb
                // ausserhalb der TEST-Sperre. Es bewegt nichts und laesst sich
                // durch ein erneutes Tarieren jederzeit korrigieren.
                //
                // ARG waehlt den Sensor: 0 = beide, 1 = Sensor 1, 2 = Sensor 2.
                // Der Abgleich startet nur und laeuft im Hintergrund weiter - der
                // Loop (Uplink, Motor-Watchdog, Telemetrie) wird nicht angehalten.
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
                // Ausschalten ist ein SICHERHEITSkommando und deshalb in allen
                // Bodenzustaenden erlaubt, nicht nur in TEST (im Flug und in
                // DESCENT sperrt es commandAllowed()).
                Serial.println("INFO  [System]: TC MOTOR_OFF");
                execMotorAction(motor_, motor_ready_, "MOTOR", MotorAction::OFF, 0, motor_on_since_ms_);
                continue;

            case UplinkOpcode::MOTOR2_OFF:
                // Gleiche Ausnahme wie MOTOR_OFF, fuer Motor 2.
                Serial.println("INFO  [System]: TC MOTOR2_OFF");
                execMotorAction(motor2_, motor2_ready_, "MOTOR2", MotorAction::OFF, 0, motor2_on_since_ms_);
                continue;

            case UplinkOpcode::ARM_STOP:
            case UplinkOpcode::ARM_INHIBIT: {
                // Anhalten und Sperren sind SICHERHEITSkommandos wie MOTOR_OFF:
                // ausserhalb der TEST-Sperre. Gelenk 0 = alle.
                const bool stop = op == UplinkOpcode::ARM_STOP;
                const uint8_t node = armNode(cmd);
                Serial.printf("INFO  [System]: TC %s Gelenk %u%s\n",
                              stop ? "ARM_STOP" : "ARM_INHIBIT", node, node == 0 ? " (alle)" : "");
                // Laufende Trajektorie zuerst beenden, sonst schickt der
                // Player im naechsten Takt schon wieder ein Ziel.
                if (node == 0 || arm_traj_.uses(node)) {
                    arm_traj_.cancel(stop ? "ARM_STOP" : "ARM_INHIBIT");
                }
                if (!arm_ || !arm_ready_) continue;
                if (node == 0) {
                    if (stop) arm_->stopAll(); else arm_->inhibitAll();
                } else if (node <= ARM_MAX_JOINTS) {
                    arm_->sendCommand(node, stop ? ArmMsg::STOP : ArmMsg::INHIBIT);
                }
                continue;
            }

            case UplinkOpcode::ARM_BLINK: {
                // Bewegt nichts - zur Zuordnung Node-ID <-> Gelenk am Aufbau.
                const uint8_t node = armNode(cmd);
                Serial.printf("INFO  [System]: TC ARM_BLINK Gelenk %u\n", node);
                if (arm_ && arm_ready_ && node >= 1 && node <= ARM_MAX_JOINTS) {
                    arm_->sendCommand(node, ArmMsg::BLINK);
                }
                continue;
            }

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

        if (!handleMotorCommand(cmd) && !handleArmCommand(cmd)) {
            Serial.printf("WARN  [System]: Unbekanntes Telecommand 0x%02X ignoriert.\n",
                          cmd.opcode);
        }
    }
}

bool System::execMotorAction(MotorHAL* motor, bool ready, const char* label,
                             MotorAction action, int16_t arg, uint32_t& on_since_ms) {
    if (!ready || !motor) {
        Serial.printf("WARN  [System]: %s-Telecommand ohne initialisierten Motor - ignoriert.\n", label);
        return true;   // bekanntes Kommando, nur keine Hardware
    }

    switch (action) {
        case MotorAction::ON:
            Serial.printf("INFO  [System]: TC %s_ON (speed=%d)\n", label, arg);
            motor->on(arg);
            on_since_ms = millis();
            return true;
        case MotorAction::TURN:
            Serial.printf("INFO  [System]: TC %s_TURN (%d Grad relativ)\n", label, arg);
            if (!motor->hasEncoder()) {
                Serial.printf("WARN  [System]: %s_TURN ohne Encoder - ignoriert.\n", label);
                return true;
            }
            motor->turnBy(arg);
            // Der Watchdog gilt auch hier: Bleibt der Encoder stehen (Mechanik
            // fest, Kanal ab), erreicht update() sein Ziel nie.
            on_since_ms = millis();
            return true;
        case MotorAction::GOTO:
            Serial.printf("INFO  [System]: TC %s_GOTO (%d Grad absolut)\n", label, arg);
            if (!motor->hasEncoder()) {
                Serial.printf("WARN  [System]: %s_GOTO ohne Encoder - ignoriert.\n", label);
                return true;
            }
            motor->goTo(arg);
            on_since_ms = millis();
            return true;
        case MotorAction::OFF:
            Serial.printf("INFO  [System]: TC %s_OFF\n", label);
            motor->off();
            return true;
        case MotorAction::ZERO:
            Serial.printf("INFO  [System]: TC %s_ZERO - Encoder-Zaehler auf 0\n", label);
            motor->zeroPosition();
            return true;
    }
    return false;
}

bool System::handleMotorCommand(const UplinkCommand& cmd) {
    switch (static_cast<UplinkOpcode>(cmd.opcode)) {
        case UplinkOpcode::MOTOR_ON:
            return execMotorAction(motor_, motor_ready_, "MOTOR", MotorAction::ON, cmd.arg, motor_on_since_ms_);
        case UplinkOpcode::MOTOR_TURN:
            return execMotorAction(motor_, motor_ready_, "MOTOR", MotorAction::TURN, cmd.arg, motor_on_since_ms_);
        case UplinkOpcode::MOTOR_GOTO:
            return execMotorAction(motor_, motor_ready_, "MOTOR", MotorAction::GOTO, cmd.arg, motor_on_since_ms_);
        case UplinkOpcode::MOTOR_OFF:
            return execMotorAction(motor_, motor_ready_, "MOTOR", MotorAction::OFF, cmd.arg, motor_on_since_ms_);
        case UplinkOpcode::MOTOR_ZERO:
            return execMotorAction(motor_, motor_ready_, "MOTOR", MotorAction::ZERO, cmd.arg, motor_on_since_ms_);

        case UplinkOpcode::MOTOR2_ON:
            return execMotorAction(motor2_, motor2_ready_, "MOTOR2", MotorAction::ON, cmd.arg, motor2_on_since_ms_);
        case UplinkOpcode::MOTOR2_TURN:
            return execMotorAction(motor2_, motor2_ready_, "MOTOR2", MotorAction::TURN, cmd.arg, motor2_on_since_ms_);
        case UplinkOpcode::MOTOR2_GOTO:
            return execMotorAction(motor2_, motor2_ready_, "MOTOR2", MotorAction::GOTO, cmd.arg, motor2_on_since_ms_);
        case UplinkOpcode::MOTOR2_OFF:
            return execMotorAction(motor2_, motor2_ready_, "MOTOR2", MotorAction::OFF, cmd.arg, motor2_on_since_ms_);
        case UplinkOpcode::MOTOR2_ZERO:
            return execMotorAction(motor2_, motor2_ready_, "MOTOR2", MotorAction::ZERO, cmd.arg, motor2_on_since_ms_);
        default:
            return false;
    }
}

void System::checkMotorTimeout(MotorHAL* motor, bool ready, const char* label,
                               uint32_t on_since_ms, uint32_t now_ms) {
    if (!ready || !motor || !motor->isOn()) return;

    if (now_ms - on_since_ms >= MOTOR_ON_TIMEOUT_MS) {
        Serial.printf("WARN  [System]: %s-Laufzeit ueber %lu ms - "
                      "automatisch gestoppt. Erneut %s_ON senden.\n",
                      label, static_cast<unsigned long>(MOTOR_ON_TIMEOUT_MS), label);
        motor->off();
    }
}

void System::handleMotorTimeout(uint32_t now_ms) {
    // Ohne Positionsregelung endet eine Motorfahrt nicht mehr von selbst - der
    // Motor dreht, bis MOTOR_OFF kommt. Wenn ausgerechnet die Uplink-Strecke
    // der Prueflig ist, ist das am Tisch ein Risiko. Der Watchdog ist die
    // Rueckfallebene; MOTOR_ON_TIMEOUT_MS = 0 schaltet ihn ab.
    if (MOTOR_ON_TIMEOUT_MS == 0) return;

    checkMotorTimeout(motor_,  motor_ready_,  "MOTOR",  motor_on_since_ms_,  now_ms);
    checkMotorTimeout(motor2_, motor2_ready_, "MOTOR2", motor2_on_since_ms_, now_ms);
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
