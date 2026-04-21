#pragma once
#include <Arduino.h>

/**
 * HAL UART Interface - Abstrahiert serielle Kommunikation
 * 
 * UART für Telemetrie:
 * - Serial (USB): 115200 baud für Debug/Telemetry
 */

namespace HAL {

// UART Buffer Größen
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256

/**
 * Initialisiert UART/Serial
 * @param baudrate Baud-Rate (default 115200)
 * @return true wenn erfolgreich
 */
bool uartInit(uint32_t baudrate = 115200);

/**
 * Schreibt einzelnes Zeichen
 * @param c Zeichen
 */
void uartWrite(char c);

/**
 * Schreibt String
 * @param str String
 */
void uartWriteString(const char* str);

/**
 * Liest einzelnes Zeichen (non-blocking)
 * @return Zeichen oder -1 wenn nichts verfügbar
 */
int16_t uartRead();

/**
 * Liest bis zu len Bytes
 * @param buffer Buffer für Daten
 * @param len Maximum zu lesende Bytes
 * @return Anzahl der tatsächlich gelesenen Bytes
 */
uint16_t uartReadBytes(uint8_t* buffer, uint16_t len);

/**
 * Prüft ob Daten verfügbar sind
 * @return Anzahl verfügbarer Bytes
 */
uint16_t uartAvailable();

/**
 * Schreibt Daten (Telemetrie Paket)
 * @param data Daten
 * @param len Länge
 */
void uartWriteData(const uint8_t* data, uint16_t len);

/**
 * Flush - Sendet alle gepufferten Daten
 */
void uartFlush();

/**
 * Debug Print (wie Serial.println)
 * @param message Debug-Nachricht
 */
void uartDebugPrint(const char* message);

} // namespace HAL
