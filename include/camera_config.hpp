#pragma once

#include <cstddef>
#include "hal/camera_hal.hpp"

/**
 * @file camera_config.hpp
 * @brief Kamera-Liste - EINZIGE Stelle, die für "mehr/weniger Kameras" oder
 *        ein anderes Trigger-Verhalten geändert werden muss.
 *
 * Kamera hinzufügen  -> eine Zeile unten ergänzen.
 * Kamera entfernen   -> Zeile löschen.
 *
 * serial = nullptr bedeutet "noch nicht angeschlossen/nicht geklärt" - die
 * Kamera wird dann zwar angelegt, init()/update() tun aber nichts.
 *
 * Pin -> HardwareSerial (Teensy 4.1, Zuordnung ist durch die Hardware fest
 * vorgegeben, siehe pin_config.hpp):
 *
 *   Kamera 1: Serial2  ->  Pin 8 = TX2 an Kamera RX (PIN_CAM_MAIN_RX)
 *                          Pin 7 = RX2 an Kamera TX (PIN_CAM1_TX)
 *   Kamera 2: Serial6  ->  Pin 24 = TX6 an Kamera RX (PIN_CAM_BACKUP_RX)
 *                          Pin 25 = RX6 an Kamera TX (PIN_CAM3_TX)
 *
 * Die Signalnamen in pin_config.hpp sind aus Sicht der KAMERA benannt
 * (CAM_..._RX = Eingang der Kamera), deshalb hängt PIN_CAM1_TX am Teensy-RX.
 *
 * Achtung: docs/teensyPins/MAGGIE-OCB-PIN-BELEGUNG.txt weist die Kamera auf
 * Pin 20/21 (= Serial5) aus und nennt zusätzlich CAMDIR1/CAMDIR2 (Pin 19/22)
 * für einen MUX. Bis die Pinbelegung final ist, gilt bewusst pin_config.hpp.
 */

static constexpr CameraConfig CAMERA_CONFIGS[] = {
    { /*camera_id=*/1, /*serial=*/&Serial2, RunCam::BAUDRATE,
      RunCamModel::SPLIT_4, CameraTriggerMode::RECORD_ON_BOOT,
      CameraHAL::DEFAULT_BOOT_DELAY_MS },

    // Kamera 2: Pin 24 (TX6) und Pin 25 (RX6)
    // { /*camera_id=*/2, /*serial=*/&Serial6, RunCam::BAUDRATE,
    //   RunCamModel::SPLIT_4, CameraTriggerMode::RECORD_ON_BOOT,
    //   CameraHAL::DEFAULT_BOOT_DELAY_MS },
};

static constexpr size_t CAMERA_COUNT = sizeof(CAMERA_CONFIGS) / sizeof(CAMERA_CONFIGS[0]);
