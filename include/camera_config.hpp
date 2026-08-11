#pragma once

#include <cstddef>
#include "hal/camera_hal.hpp"

/**
 * @brief Kamera-Liste - EINZIGE Stelle, die für "mehr/weniger Kameras" oder
 *        ein anderes Speicherintervall geändert werden muss.
 *
 * Kamera hinzufügen  -> eine Zeile unten ergänzen.
 * Kamera entfernen   -> Zeile löschen.
 * Intervall ändern   -> interval_ms anpassen.
 *
 * serial = nullptr bedeutet "noch nicht angeschlossen/nicht geklärt" - die
 * Kamera wird dann zwar angelegt, init()/update() tun aber nichts.
 */

static constexpr CameraConfig CAMERA_CONFIGS[] = {
    { /*camera_id=*/1, /*serial=*/&Serial2, /*baudrate=*/115200,
      CameraTriggerMode::SNAPSHOT_INTERVAL, /*interval_ms=*/5000 },

    // TODO Kamera 2/3: Pin 24 (TX6) und Pin 25 (RX6) 
    // { /*camera_id=*/2, /*serial=*/&Serial6, /*baudrate=*/115200,
    //   CameraTriggerMode::SNAPSHOT_INTERVAL, /*interval_ms=*/5000 },
};

static constexpr size_t CAMERA_COUNT = sizeof(CAMERA_CONFIGS) / sizeof(CAMERA_CONFIGS[0]);
