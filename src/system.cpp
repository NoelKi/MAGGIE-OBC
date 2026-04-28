#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

System::System() {
    // Konstruktor
}

System::~System() {
    delete weight_sensor_;
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