#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief CAN-Bus Kommunikations-Treiber
 * Bidirektionale Kommunikation zwischen Teensy 4.1 (OBC) und STM32 (Roboterarm)
 * 
 * Nachrichtenformat:
 * OBC → STM32: Befehle für Roboterarm
 * STM32 → OBC: Sensor-Daten, Status
 */

// CAN Message IDs
namespace CAN_IDs {
    // Befehle OBC → STM32
    constexpr uint32_t CMD_ARM_MOVE = 0x100;        // Bewegungsbefehl
    constexpr uint32_t CMD_ARM_STOP = 0x101;        // Notfall-Stop
    constexpr uint32_t CMD_GRIPPER = 0x102;         // Greifer-Steuerung
    
    // Telemetrie STM32 → OBC
    constexpr uint32_t TELEMETRY_ARM_STATUS = 0x200;  // Arm-Status
    constexpr uint32_t TELEMETRY_SENSORS = 0x201;     // Sensor-Werte
    constexpr uint32_t HEARTBEAT_STM32 = 0x300;       // Heartbeat von STM32
}

// Struktur für Arm-Bewegungsbefehl
struct ArmCommand {
    float joint1_angle;      // Joint 1 Position (Grad)
    float joint2_angle;      // Joint 2 Position (Grad)
    float gripper_force;     // Greifer-Kraft (0-100%)
    uint8_t command_flags;   // Zusätz-Flags
} __attribute__((packed));

// Struktur für Arm-Status von STM32
struct ArmStatus {
    float joint1_position;   // Aktuelle Position Joint 1
    float joint2_position;   // Aktuelle Position Joint 2
    float joint1_current;    // Strom Joint 1 Motor (mA)
    float joint2_current;    // Strom Joint 2 Motor (mA)
    uint8_t error_code;      // Error Code von STM32
    uint8_t status_flags;    // Status Flags
} __attribute__((packed));

class CANBusDriver {
public:
    CANBusDriver();
    ~CANBusDriver();
    
    /**
     * @brief Initialisiert CAN-Bus auf Teensy 4.1
     * @param baudrate CAN Baudrate (500kbps für Standard-Avionik)
     * @return true wenn erfolgreich
     */
    bool init(uint32_t baudrate = 500000);
    
    /**
     * @brief Sendet Arm-Bewegungsbefehl an STM32
     * @param cmd Bewegungsbefehl
     * @return true wenn erfolgreich
     */
    bool sendArmCommand(const ArmCommand& cmd);
    
    /**
     * @brief Empfängt Arm-Status von STM32 (non-blocking)
     * @param status Zeiger auf Status-Struktur
     * @return true wenn neue Daten verfügbar
     */
    bool receiveArmStatus(ArmStatus& status);
    
    /**
     * @brief Sendet allgemeinen CAN-Frame
     * @param msg_id Message ID (11-bit oder 29-bit extended)
     * @param data Zeiger auf Daten
     * @param length Daten-Länge (max 8 Bytes für CAN 2.0)
     * @return true wenn erfolgreich
     */
    bool sendMessage(uint32_t msg_id, const uint8_t* data, uint8_t length);
    
    /**
     * @brief Empfängt CAN-Message (non-blocking)
     * @param msg_id Zeiger auf Message ID
     * @param data Zeiger auf Daten-Buffer
     * @param length Zeiger auf Länge
     * @return true wenn neue Message verfügbar
     */
    bool receiveMessage(uint32_t& msg_id, uint8_t* data, uint8_t& length);
    
    /**
     * @brief Prüft Verbindungsstatus mit STM32
     */
    bool isConnected() const;
    
    /**
     * @brief Gibt letzter erhaltene Heartbeat-Zeit zurück
     */
    uint32_t getLastHeartbeatTime() const;
    
    /**
     * @brief Prüft CAN-Bus Fehler
     */
    uint32_t getErrorCount() const;

private:
    bool initialized = false;
    uint32_t last_heartbeat = 0;
    uint32_t error_count = 0;
    ArmStatus last_arm_status = {};
    
    static constexpr uint32_t HEARTBEAT_TIMEOUT = 2000;  // 2 Sekunden
};
