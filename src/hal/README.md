# 🏗️ MAGGIE HAL (Hardware Abstraction Layer)

Eine vollständige Hardware-Abstraktionsschicht für das MAGGIE OBC auf Teensy 4.1

## 📁 Struktur

```
hal/
├── hal.h                    ← Master Header (importiere nur diese!)
├── hal.cpp                  ← Zentrale Initialisierung
├── hal_config.h             ← Alle Pin-Definitionen (zentral)
├── hal_gpio.h / .cpp        ← GPIO & LEDs
├── hal_i2c.h / .cpp         ← I2C Bus (IMU, Barometer)
├── hal_spi.h / .cpp         ← SPI Bus (Drucksensor, SD-Card)
├── hal_adc.h / .cpp         ← Analog Digital Conversion (Weight Sensors)
├── hal_pwm.h / .cpp         ← PWM Motor Control
├── hal_uart.h / .cpp        ← Serielle Kommunikation (Telemetry)
├── hal_can.h / .cpp         ← CAN-Bus (STM32 Roboterarm)
└── README.md                ← Diese Datei
```

---

## 🎯 Features

✅ **Zentrale Pin-Konfiguration** - Alle Pins in `hal_config.h`  
✅ **Hardware-Unabhängigkeit** - Nur HAL-Interface nutzen  
✅ **Error Handling** - Alle Funktionen geben bool zurück  
✅ **Logging** - Alle Init-Vorgänge werden geloggt  
✅ **Namespace** - Alles in `namespace HAL`  
✅ **Dokumentation** - Alle Funktionen dokumentiert  

---

## 📚 Module

### 🔧 `hal_config.h` - Zentrale Konfiguration

```cpp
// I2C
#define HAL_I2C_SDA 18
#define HAL_I2C_SCL 19

// SPI
#define HAL_SPI_MOSI 11
#define HAL_SPI_MISO 12
#define HAL_SPI_SCK 13

// CAN
#define HAL_CAN_TX 22
#define HAL_CAN_RX 23

// PWM (Motor)
#define HAL_PWM_SPEED 9
#define HAL_PWM_DIR1 7
#define HAL_PWM_DIR2 8
#define HAL_PWM_ENABLE 6

// ADC (Weight Sensors)
#define HAL_ADC_SCALE1 A0
#define HAL_ADC_SCALE2 A1

// UART
#define HAL_UART_BAUD 115200
```

**Wenn Hardware-Pins ändern:** Nur hier anpassen!

---

### 💡 `hal_gpio.h` - GPIO & LEDs

```cpp
// Initialisierung (optional)
HAL::setLed(true);  // LED an
HAL::setLed(false); // LED aus
```

---

### 📡 `hal_i2c.h` - I2C Bus

Für IMU (MPU9250) und Barometer (BMP390)

```cpp
// Initialisierung
bool success = HAL::i2cInit();

// Schreibe Daten
uint8_t data[] = {0x68, 0x01};
HAL::i2cWrite(0x68, data, 2);

// Lese Daten
uint8_t buffer[6];
HAL::i2cRead(0x68, buffer, 6);

// Write-Read (Register Read Pattern)
uint8_t value;
HAL::i2cWriteRead(0x68, 0x3B, &value, 1);

// Scan nach Geräten
uint8_t count = HAL::i2cScan();
```

---

### 🔌 `hal_spi.h` - SPI Bus

Für Drucksensor (MS5607) und SD-Card

```cpp
// Initialisierung
bool success = HAL::spiInit();

// Transfer (vollduplex)
uint8_t data[] = {0xFF, 0xFF, 0xFF};
HAL::spiTransfer(HAL_SPI_CS_PRESSURE, data, 3);

// Schreibe
uint8_t cmd[] = {0x1E};  // Reset Command
HAL::spiWrite(HAL_SPI_CS_PRESSURE, cmd, 1);

// Lese
uint8_t buffer[3];
HAL::spiRead(HAL_SPI_CS_PRESSURE, buffer, 3);

// Register I/O
uint8_t regValue;
HAL::spiReadRegister(HAL_SPI_CS_PRESSURE, 0x50, regValue);

HAL::spiWriteRegister(HAL_SPI_CS_PRESSURE, 0x50, 0xFF);
```

---

### 🔊 `hal_adc.h` - Analog Digital Conversion

Für Gewichtssensoren

```cpp
// Initialisierung (12-bit)
bool success = HAL::adcInit(ADC_RES_12BIT);

// Lese ADC-Wert (0-4095 bei 12-bit)
uint16_t raw = HAL::adcRead(HAL_ADC_SCALE1);

// Lese mit Averaging (Stabiler)
uint16_t average = HAL::adcReadAverage(HAL_ADC_SCALE1, 10);

// Konvertiere zu Spannung (0.0 - 3.3V)
float voltage = HAL::adcToVoltage(raw);

// Kalibriere (Nullpunkt)
uint16_t calibValue = HAL::adcCalibrate(HAL_ADC_SCALE1);
```

---

### ⚡ `hal_pwm.h` - PWM Motor Control

Für HDRM Motor

```cpp
// Initialisierung
bool success = HAL::pwmInit();

// Setze Geschwindigkeit (0-100%)
HAL::pwmSetSpeed(50);  // 50%

// Setze Richtung
HAL::pwmSetDirection(HAL::MOTOR_FORWARD);   // Vorwärts
HAL::pwmSetDirection(HAL::MOTOR_BACKWARD);  // Rückwärts
HAL::pwmSetDirection(HAL::MOTOR_BRAKE);     // Bremse
HAL::pwmSetDirection(HAL::MOTOR_STOP);      // Stopp

// Enable/Disable
HAL::pwmSetEnabled(true);   // Motor an
HAL::pwmSetEnabled(false);  // Motor aus

// Shortcut
HAL::pwmSetMotor(75, HAL::MOTOR_FORWARD);

// Emergency Stop
HAL::pwmEmergencyStop();

// Debug
HAL::pwmDebugStatus();
```

---

### 📨 `hal_uart.h` - Serielle Kommunikation

Für Telemetry über USB Serial

```cpp
// Initialisierung
bool success = HAL::uartInit(115200);

// Schreibe Zeichen
HAL::uartWrite('H');

// Schreibe String
HAL::uartWriteString("Hello");

// Schreibe Daten (z.B. Telemetry Paket)
uint8_t packet[] = {0x01, 0x02, 0x03};
HAL::uartWriteData(packet, 3);

// Lese Zeichen (non-blocking)
int16_t c = HAL::uartRead();
if (c != -1) {
    Serial.print((char)c);
}

// Lese mehrere Bytes
uint8_t buffer[32];
uint16_t bytesRead = HAL::uartReadBytes(buffer, 32);

// Prüfe verfügbare Bytes
if (HAL::uartAvailable() > 0) {
    // Daten verfügbar
}

// Flush (Sendet alle gepufferten Daten)
HAL::uartFlush();

// Debug Print
HAL::uartDebugPrint("Motor started");
```

---

### 🚗 `hal_can.h` - CAN-Bus

Für STM32 Roboterarm Kommunikation

```cpp
// Initialisierung
bool success = HAL::canInit(500000);  // 500 kbps

// Sende CAN-Nachricht
uint8_t data[] = {0x00, 0x01, 0x02, 0x03};
HAL::canSend(0x100, data, 4);

// Empfange CAN-Nachricht
uint32_t id;
uint8_t rxData[8];
uint8_t len;
if (HAL::canReceive(id, rxData, len)) {
    // Nachricht empfangen
    Serial.print("ID: 0x");
    Serial.println(id, HEX);
}
```

---

## 🚀 Verwendung

### In `main.cpp`:

```cpp
#include "hal/hal.h"

void setup() {
    // Initialisiere alle HAL-Module auf einmal
    if (!HAL::initializeAll()) {
        while(1) {
            delay(1000);  // Fehler: Hänge fest
        }
    }
    
    Serial.println("All systems ready!");
}

void loop() {
    // Nutze HAL-Funktionen...
    uint16_t weight = HAL::adcRead(HAL_ADC_SCALE1);
    HAL::pwmSetMotor(50, HAL::MOTOR_FORWARD);
}
```

### In Services (z.B. `imu_driver.cpp`):

```cpp
#include "hal/hal.h"

bool IMUDriver::init() {
    // Initialisiere I2C
    if (!HAL::i2cInit()) {
        return false;
    }
    
    // Scanne I2C Bus
    HAL::i2cScan();
    
    // Lese IMU Register
    uint8_t whoAmI;
    if (!HAL::i2cWriteRead(0x68, 0x75, &whoAmI, 1)) {
        return false;
    }
    
    return true;
}

void IMUDriver::readAccel(float &x, float &y, float &z) {
    uint8_t buffer[6];
    HAL::i2cRead(0x68, buffer, 6);
    
    // Konvertiere zu g's
    x = (buffer[0] << 8 | buffer[1]) / 16384.0;
    y = (buffer[2] << 8 | buffer[3]) / 16384.0;
    z = (buffer[4] << 8 | buffer[5]) / 16384.0;
}
```

---

## 🔄 Hardware-Wechsel

Das ist die Kraft der HAL!

### Szenario: Von Teensy 4.1 zu STM32H7

**Ohne HAL:**
- Durchsuche alle ~2400 Zeilen Code
- Ändere Arduino-Befehle zu STM32-Befehlen
- Fehler-anfällig, zeitintensiv

**Mit HAL:**
- Ändere nur `hal/hal_i2c.cpp`, `hal/hal_spi.cpp`, etc.
- Services bleiben unverändert
- Schnell und sicher

---

## 📊 Pinout Übersicht (Teensy 4.1)

```
Teensy 4.1 MAGGIE OBC Pinout
═════════════════════════════════

I2C (Wire):
  Pin 18 (SDA) ────→ IMU + Barometer
  Pin 19 (SCL) ────→ IMU + Barometer

SPI:
  Pin 11 (MOSI) ────→ Drucksensor (MS5607) + SD-Card
  Pin 12 (MISO) ────→ Drucksensor (MS5607) + SD-Card
  Pin 13 (SCK) ─────→ Drucksensor (MS5607) + SD-Card
  Pin 10 (CS) ──────→ Drucksensor (MS5607)
  Pin 4  (CS) ──────→ SD-Card

CAN:
  Pin 22 (TX) ──────→ STM32 Roboterarm
  Pin 23 (RX) ──────→ STM32 Roboterarm

PWM Motor:
  Pin 9 (Speed) ────→ HDRM Motor PWM
  Pin 7 (Dir1) ─────→ HDRM H-Bridge
  Pin 8 (Dir2) ─────→ HDRM H-Bridge
  Pin 6 (Enable) ───→ HDRM Enable

ADC:
  Pin A0 ───────────→ Gewichtssensor 1
  Pin A1 ───────────→ Gewichtssensor 2

Serial/UART:
  Pin 0 (RX) ───────→ USB Serial
  Pin 1 (TX) ───────→ USB Serial
```

---

## 🎓 Best Practices

### ✅ DO's

```cpp
// ✅ Nutze HAL Funktionen
HAL::i2cInit();
HAL::adcRead(HAL_ADC_SCALE1);

// ✅ Nutze HAL Pin-Konstanten
digitalWrite(HAL_LED_PIN, HIGH);
```

### ❌ DON'Ts

```cpp
// ❌ Hartcodiere Pins nicht
digitalWrite(13, HIGH);

// ❌ Nutze Arduino-Funktionen direkt
Wire.beginTransmission(0x68);

// Stattdessen nutzen:
HAL::i2cWrite(0x68, ...);
```

---

## 🧪 Testing HAL

```cpp
// In setup() oder Test-Sketch

void testHAL() {
    Serial.println("Testing HAL...");
    
    // Test I2C
    Serial.print("I2C Scan: ");
    uint8_t count = HAL::i2cScan();
    Serial.println(count);
    
    // Test ADC
    Serial.print("ADC Value: ");
    Serial.println(HAL::adcRead(HAL_ADC_SCALE1));
    
    // Test PWM
    HAL::pwmSetMotor(25, HAL::MOTOR_FORWARD);
    delay(2000);
    HAL::pwmSetMotor(0, HAL::MOTOR_STOP);
    
    // Test Serial
    HAL::uartDebugPrint("All tests completed");
}
```

---

## 📝 Checkliste für neue Hardware

Wenn Sie neue Hardware hinzufügen:

1. ✅ Pin-Definition in `hal_config.h` hinzufügen
2. ✅ Neues HAL-Modul erstellen (z.B. `hal_temp.h`)
3. ✅ Implementierung in `.cpp` schreiben
4. ✅ In `hal.h` includieren
5. ✅ In `HAL::initializeAll()` aufrufen
6. ✅ Dokumentation updaten

---

## 🔗 Referenzen

- [Teensy 4.1 Pins](https://www.pjrc.com/teensy/pinout.html)
- [Arduino I2C (Wire)](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- [Arduino SPI](https://www.arduino.cc/en/reference/SPI)
- [FlexCAN Library](https://github.com/tonton81/FlexCAN_T4)

---

**Version**: 1.0.0  
**Created**: 2026-03-06  
**Target**: Teensy 4.1  
**Status**: ✅ Production Ready
