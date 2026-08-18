#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

/**
 * @file camera_bus.hpp
 * @brief Gemeinsamer UART für alle Kameras, umgeschaltet über einen 4:1-Mux.
 *
 * Alle vier RunCam Split 4 hängen an EINEM UART des Teensy. TX und RX werden
 * gemeinsam über einen Analog-Mux (Bauform 74HC4052) auf die gewünschte
 * Kamera geschaltet. Die Kanalwahl erfolgt über zwei Select-Leitungen:
 *
 *     Kanal = (CAMDIR2 << 1) | CAMDIR1
 *
 *     Kanal 0 -> Kamera 1     Kanal 2 -> Kamera 3
 *     Kanal 1 -> Kamera 2     Kanal 3 -> Kamera 4
 *
 * Konsequenz für die Firmware: es ist immer nur GENAU EINE Kamera erreichbar.
 * Antworten können nur gelesen werden, solange der passende Kanal anliegt -
 * deshalb läuft der Device-Info-Handshake blockierend ab (siehe CameraHAL),
 * statt über mehrere update()-Durchläufe verteilt.
 *
 * Nicht angewählte Kameras hören nichts. Eine laufende Aufnahme stört das
 * nicht: sie läuft in der Kamera selbstständig weiter.
 */
class CameraBus {
public:
    /// Anzahl der Mux-Kanäle bei zwei Select-Leitungen.
    static constexpr uint8_t MAX_CHANNELS = 4;
    /// "noch kein Kanal angewählt"
    static constexpr uint8_t NO_CHANNEL = 0xFF;
    /// Einschwingzeit des Mux nach dem Umschalten. Der 74HC4052 schaltet in
    /// deutlich unter 1 us; 50 us sind großzügig und im Ablauf irrelevant.
    static constexpr uint32_t SETTLE_US = 50;

    CameraBus(HardwareSerial* serial, uint8_t select_a_pin, uint8_t select_b_pin);

    /// Öffnet den UART und legt die Select-Leitungen auf Kanal 0.
    bool begin(uint32_t baudrate);

    /// Schaltet den Mux auf den Kanal. false bei ungültigem Kanal/ohne begin().
    bool select(uint8_t channel);

    /// Verwirft alles, was im Empfangspuffer steht.
    void flushInput();

    HardwareSerial* serial() const { return serial_; }
    uint8_t currentChannel() const { return current_channel_; }
    bool isReady() const { return initialized_; }

private:
    HardwareSerial* serial_;
    uint8_t select_a_pin_;
    uint8_t select_b_pin_;
    uint8_t current_channel_ = NO_CHANNEL;
    bool initialized_ = false;
};
