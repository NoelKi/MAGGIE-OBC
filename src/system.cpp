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
    for (auto* camera : cameras_) {
        delete camera;
    }
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
        Serial.println("INFO  [System]: Motor 1 bereit (Encoder Pin 16/17).");
    } else {
        Serial.println("WARN  [System]: Motor 1 konnte nicht initialisiert werden.");
    }

    // -----------------------------------------------------------------------
    // Uplink-Telecommand-Empfang (teilt sich Serial8 mit dem Downlink, Pin 34 RX)
    // -----------------------------------------------------------------------
    uplink_ = new UplinkReceiver(Serial8);

    // -----------------------------------------------------------------------
    // Kameras (Anzahl/Intervall siehe camera_config.hpp)
    // Fehler hier sind NICHT fatal: einzelne Kameras können fehlen/ungeklärt sein.
    // -----------------------------------------------------------------------
    Serial.printf("INFO  [System]: Initialisiere %u Kamera(s)...\n", (unsigned)CAMERA_COUNT);
    for (size_t i = 0; i < CAMERA_COUNT; ++i) {
        cameras_[i] = new CameraHAL(CAMERA_CONFIGS[i]);
        if (!cameras_[i]->isPresent()) {
            Serial.printf("WARN  [System]: Kamera %u hat keinen UART-Port (siehe camera_config.hpp) - übersprungen.\n",
                          cameras_[i]->getCameraID());
            continue;
        }
        if (cameras_[i]->init()) {
            Serial.printf("INFO  [System]: Kamera %u bereit.\n", cameras_[i]->getCameraID());
        } else {
            Serial.printf("WARN  [System]: Kamera %u konnte nicht initialisiert werden.\n", cameras_[i]->getCameraID());
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

    // Eingehende Motor-Telecommands verarbeiten (jeden Loop, geringe Latenz)
    handleUplink();

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

    // Kameras: jede Kamera entscheidet anhand ihres CameraTriggerMode selbst,
    // ob gerade ein Kommando fällig ist (siehe camera_config.hpp).
    handleCameras(now);
}

void System::handleCameras(uint32_t now_ms) {
    for (auto* camera : cameras_) {
        if (camera) camera->update(now_ms);
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

    uint8_t status1 = 0;
    if (system_healthy) status1 |= DL_STATUS1_SYSTEM_HEALTHY;

    downlink_->sendMotor(position, speed, state, status1, 0);
}

void System::handleUplink() {
    if (!uplink_ || !motor_ready_ || !motor_) return;

    MotorCommand cmd;
    while (uplink_->poll(cmd)) {
        switch (static_cast<MotorOpcode>(cmd.opcode)) {
            case MotorOpcode::ON:        motor_->on();       break;
            case MotorOpcode::OFF:       motor_->off();      break;
            case MotorOpcode::HALF_TURN: motor_->halfTurn(); break;
            default: break;   // unbekanntes Kommando ignorieren
        }
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