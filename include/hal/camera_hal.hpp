#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

/**
 * TODO Bytes Protocoll der Kamera beachten
 */

// Platzhalter-Kommandos
namespace CamCmd {
    static constexpr uint8_t START_RECORDING[] = { 0x00 };  //< TODO
    static constexpr uint8_t STOP_RECORDING[]  = { 0x00 };   
    static constexpr uint8_t SNAPSHOT[]        = { 0x00 };  
}

enum class CameraTriggerMode : uint8_t {
    MANUAL_ONLY,         // Kein automatisches Triggern - nur explizite Aufrufe
    SNAPSHOT_INTERVAL,   // Alle interval_ms ein Snapshot-Kommando senden
    CONTINUOUS,          // Einmalig "Start" senden, läuft bis stopRecording()
};

// Konfiguration einer einzelnen Kamera
struct CameraConfig {
    uint8_t camera_id;            
    HardwareSerial* serial;       // UART-Port TODO
    uint32_t baudrate;
    CameraTriggerMode mode;
    uint32_t interval_ms;         
};

class CameraHAL {
public:
    explicit CameraHAL(const CameraConfig& config);

    // config.serial == nullptr
    bool init();

    void update(uint32_t now_ms);

    size_t sendCommand(const uint8_t* command, size_t length);
    size_t receiveData(uint8_t* buffer, size_t max_length);
    bool available();

    bool startRecording();
    bool stopRecording();
    bool captureSnapshot();

    uint8_t getCameraID() const { return camera_id_; }

    // TODO
    bool isPresent() const { return serial_ != nullptr; }

private:
    uint8_t camera_id_;
    HardwareSerial* serial_;
    uint32_t baudrate_;
    CameraTriggerMode mode_;
    uint32_t interval_ms_;

    bool initialized_ = false;
    uint32_t last_trigger_ms_ = 0;
    bool continuous_started_ = false;
};
