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

    if (!weight_sensor_->init(ScaleType::HX711)) {
        Serial.println("ERROR [System]: Gewichtssensor konnte nicht initialisiert werden!");
        system_healthy = false;
        return false;
    }

    // Kalibrierung und Tare
    weight_sensor_->setCalibrationFactor(1, WEIGHT_CALIBRATION_FACTOR);
    weight_sensor_->tareScale1();

    Serial.println("INFO  [System]: Gewichtssensor bereit.");

    // -----------------------------------------------------------------------
    // Kraftsensor 2 (Custom Analog Sensor)
    // -----------------------------------------------------------------------
    Serial.println("INFO  [System]: Initialisiere Kraftsensor 2 (Analog)...");
    force_sensor_2_ = new ForceSensorHAL(
        FORCE_SENSOR_2_X_PIN,
        FORCE_SENSOR_2_Y_PIN,
        FORCE_SENSOR_2_Z_PIN
    );

    if (!force_sensor_2_->init()) {
        Serial.println("ERROR [System]: Kraftsensor 2 konnte nicht initialisiert werden!");
        system_healthy = false;
        return false;
    }

    Serial.println("INFO  [System]: Kraftsensor 2 bereit.");

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

    system_healthy = true;
    Serial.println("INFO  [System]: System erfolgreich initialisiert.\n");
    return true;
}

void System::run() {
    if (!system_healthy) return;

    const uint32_t now = millis();

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

    // Telemetrie-Downlink periodisch senden
    if (now - last_telemetry_ms_ >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_ms_ = now;
        handleTelemetry();
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

void System::handleWeightReading() {
    if (!weight_sensor_) return;

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
    if (!force_sensor_2_) return;

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