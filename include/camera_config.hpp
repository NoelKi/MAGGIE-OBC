#pragma once

#include <cstddef>
#include <Arduino.h>
#include "hal/camera_hal.hpp"

/**
 * @file camera_config.hpp
 * @brief Kamera-Liste - EINZIGE Stelle, die für "mehr/weniger Kameras" oder
 *        ein anderes Trigger-Verhalten geändert werden muss.
 *
 * Kamera abschalten -> enabled auf false setzen.
 * Kamera ergänzen   -> Zeile mit freiem mux_channel (0..3) hinzufügen.
 *
 * Aufbau: 4x RunCam Split 4 an EINEM UART (Serial2), umgeschaltet über einen
 * 4:1-Mux. Siehe hal/camera_bus.hpp für die Kanalkodierung.
 *
 *   mux_channel 0 -> Kamera 1     mux_channel 2 -> Kamera 3
 *   mux_channel 1 -> Kamera 2     mux_channel 3 -> Kamera 4
 *
 * Pins (Teensy 4.1, Zuordnung durch die Hardware fest vorgegeben,
 * siehe pin_config.hpp):
 *
 *   Serial2  ->  Pin 8 = TX2 an Mux-Eingang  (PIN_CAM_MAIN_RX)
 *                Pin 7 = RX2 an Mux-Ausgang  (PIN_CAM1_TX)
 *   Mux-Select -> Pin 19 = CAMDIR1 (PIN_CAM_MUX_A)
 *                 Pin 22 = CAMDIR2 (PIN_CAM_MUX_B)
 *
 * Die Signalnamen in pin_config.hpp sind aus Sicht der KAMERA benannt
 * (CAM_..._RX = Eingang der Kamera), deshalb hängt PIN_CAM1_TX am Teensy-RX.
 */

/// UART, an dem der Kamera-Mux hängt. nullptr = keine Kameras.
static HardwareSerial* const CAMERA_BUS_UART = &Serial2;

static constexpr CameraConfig CAMERA_CONFIGS[] = {
    { /*camera_id=*/1, /*mux_channel=*/0, /*enabled=*/true,
      RunCamModel::SPLIT_4, CameraTriggerMode::RECORD_ON_BOOT,
      CameraHAL::DEFAULT_BOOT_DELAY_MS },

    { /*camera_id=*/2, /*mux_channel=*/1, /*enabled=*/true,
      RunCamModel::SPLIT_4, CameraTriggerMode::RECORD_ON_BOOT,
      CameraHAL::DEFAULT_BOOT_DELAY_MS },

    { /*camera_id=*/3, /*mux_channel=*/2, /*enabled=*/true,
      RunCamModel::SPLIT_4, CameraTriggerMode::RECORD_ON_BOOT,
      CameraHAL::DEFAULT_BOOT_DELAY_MS },

    { /*camera_id=*/4, /*mux_channel=*/3, /*enabled=*/true,
      RunCamModel::SPLIT_4, CameraTriggerMode::RECORD_ON_BOOT,
      CameraHAL::DEFAULT_BOOT_DELAY_MS },
};

static constexpr size_t CAMERA_COUNT = sizeof(CAMERA_CONFIGS) / sizeof(CAMERA_CONFIGS[0]);
