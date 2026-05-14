#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

// ===========================================================================
// Sensor Read Intervals (in milliseconds)
// ===========================================================================
#define WEIGHT_READ_INTERVAL_MS 500     ///< Read weight every 500ms
#define FORCE_READ_INTERVAL_MS 100      ///< Read force every 100ms
#define WEIGHT_CALIBRATION_FACTOR 1.0f  ///< Calibration factor for HX711

System::System() {
    // Konstruktor
}

System::~System() {
    delete weight_sensor_;
    delete force_sensor_2_;
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