# MAGGIE RoboArm Experiment - Hardware Konfiguration

## 🤖 Systemübersicht

```
┌─────────────────────────────────────────────────────────┐
│         TEENSY 4.1 (OBC - On-Board Computer)           │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  CAN Bus (500 kbps)                              │  │
│  │  ↔ STM32 (Roboterarm-Steuerung)                  │  │
│  └──────────────────────────────────────────────────┘  │
│                      │                                  │
│    ┌─────────────────┼──────────────────┐             │
│    │                 │                  │             │
│  [I2C Bus]      [SPI Bus]         [ADC/PWM]          │
│    │                 │                  │             │
│    ├─ IMU            ├─ Barometer       ├─ Weight1   │
│    │  (MPU9250)      ├─ Pressure Sens   ├─ Weight2   │
│    │                 │                  ├─ Motor PWM │
│    │                 │                  │            │
│    └─────────────────┴──────────────────┴─────────── │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## 📍 Pin-Belegung (Teensy 4.1)

### CAN-Bus (Kritisch - STM32 Kommunikation)
```
Pin 22: CAN1 TX (CTX1)
Pin 23: CAN1 RX (CRX1)
Baudrate: 500 kbps (Standard Avionik)
```

### I2C Bus (Sensoren)
```
Pin 18: I2C1_SDA (Sensoren: IMU, Barometer)
Pin 19: I2C1_SCL
Frequenz: 400 kHz
```

### SPI Bus (Drucksensor, Barometer)
```
Pin 11: MOSI (SPI Master Out)
Pin 12: MISO (SPI Master In)
Pin 13: SCK (Serial Clock)
Pin 10: CS (Chip Select - Drucksensor)
Frequenz: 8-10 MHz (abhängig vom Sensor)
```

### Analog Inputs (Gewichtssensoren)
```
A0 (Pin 14): Weight Sensor 1 (Waage 1)
A1 (Pin 15): Weight Sensor 2 (Waage 2)
Resolution: 12-bit (0-4095)
Reference: 3.3V
```

### PWM Motor (HDRM)
```
Pin 9:  PWM (Geschwindigkeit)
Pin 7:  Direction 1 (Vorwärts)
Pin 8:  Direction 2 (Rückwärts)
Pin 6:  Enable (optional, High = aktiv)
PWM Frequenz: 1 kHz (konfigurierbar)
PWM Range: 0-255 (0% - 100%)
```

### Debug / Optional
```
Pin 0/1: UART1 (Serial für Debug)
Pin 13: LED (Status)
```

## 🔌 Sensor-Details

### 1. IMU (Inertial Measurement Unit)
```
Sensor: MPU9250 / ICM-20948
Interface: I2C
I2C Address: 0x68 (Default) oder 0x69 (mit AD0=High)
Pins: SDA=18, SCL=19
Data: 
  - Accelerometer: ±16g (0x3B-0x40)
  - Gyroscope: ±2000°/s (0x43-0x48)
  - Magnetometer: über AK8963 Modul
Sample Rate: bis 8 kHz (Standard: 100 Hz)
Stromverbrauch: 3.5 mA
```

### 2. Barometer (Barometerdruck & Temperatur)
```
Sensor: BMP390 oder BMP280
Interface: I2C
I2C Address: 0x77 (Default) oder 0x76
Pins: SDA=18, SCL=19
Data:
  - Druck: 300-1100 hPa
  - Temperatur: -40 bis +85°C
  - Höhenauflösung: ±1 m
Sample Rate: bis 200 Hz
```

### 3. Drucksensor (Pneumatik/Flüssigkeit)
```
Sensor: MS5607 (oder ähnlich)
Interface: SPI
CS Pin: 10
Pins: MOSI=11, MISO=12, SCK=13
Auflösung: Sensor-abhängig
Typ: Barometrischer Drucksensor
Output: 0-1100 mbar
Stromverbrauch: ~5 mA
```

### 4. Gewichtssensoren (Waagen)
```
Typ 1: Analog Load Cells (direkt über ADC)
  - Sensor 1: A0 (Pin 14)
  - Sensor 2: A1 (Pin 15)
  - Range: 0-4095 ADC Wert
  - Voltage: 0-3.3V

Typ 2: HX711 Load Cell Amplifier (optional)
  - DOUT1: Pin X, SCK1: Pin Y
  - DOUT2: Pin X, SCK2: Pin Y
  - Output: 24-Bit digital

Typische Last-Zellen:
  - 50 kg Load Cell
  - 100 kg Load Cell
  - Genauigkeit: ±0.05% FS
```

### 5. HDRM Motor (PWM)
```
Typ: Brushed DC Motor mit H-Bridge (L298N / L293D)
Interface: PWM + 2x Direction Pins

Pins:
  - PWM Pin: 9 (Geschwindigkeit 0-255)
  - Dir 1: 7 (HIGH = Vorwärts)
  - Dir 2: 8 (HIGH = Rückwärts)
  - Enable: 6 (optional)

Betrieb:
  - Pin7=HIGH, Pin8=LOW  → Vorwärts
  - Pin7=LOW, Pin8=HIGH  → Rückwärts
  - Pin7=HIGH, Pin8=HIGH → Bremse (dynamisch)
  - Pin7=LOW, Pin8=LOW   → Stop

Stromverbrauch:
  - Ohne Last: 50-100 mA
  - Mit Last: bis 500 mA (abhängig von Motor)
```

## 🔗 CAN-Bus Kommunikation (STM32)

### Message IDs
```
OBC → STM32 (Befehle):
  0x100: CMD_ARM_MOVE      - Bewegungsbefehl
  0x101: CMD_ARM_STOP      - Notfall-Stop
  0x102: CMD_GRIPPER       - Greifer-Steuerung

STM32 → OBC (Telemetrie):
  0x200: TELEMETRY_ARM_STATUS  - Arm Position, Motorstrom
  0x201: TELEMETRY_SENSORS     - Zusätz Sensoren
  0x300: HEARTBEAT_STM32       - Alive-Signal
```

### Message Format (CAN 2.0 - 8 Byte)

**ARM_MOVE (0x100):**
```
Byte 0-3: Joint1 Angle (float32, Grad)
Byte 4-7: Joint2 Angle (float32, Grad)
oder
Byte 0-3: Joint1 Angle (float16, float16)
Byte 4-7: Joint2 Angle + Gripper Force
```

**ARM_STATUS (0x200):**
```
Byte 0-3: Joint1 Position (float32)
Byte 4-5: Joint1 Current (int16, mA)
Byte 6-7: Status/Error Code (uint16)
```

## ⚙️ Software-Integration

### Teensy 4.1 Hardware Libraries:
```cpp
#include <Wire.h>          // I2C
#include <SPI.h>           // SPI
#include <FlexCAN_T4.h>    // CAN Bus
```

### Konfiguration:

**I2C:**
```cpp
Wire.begin();
Wire.setClock(400000);  // 400 kHz
```

**SPI:**
```cpp
SPI.begin();
SPI.setClockDivider(SPI_CLOCK_DIV16);  // 16 MHz / 16 = 1 MHz
```

**CAN:**
```cpp
Can0.begin();
Can0.setBaudRate(500000);
Can0.setMaxMB(16);
```

**PWM:**
```cpp
pinMode(9, OUTPUT);
analogWriteFrequency(9, 1000);  // 1 kHz
analogWrite(9, 0-255);
```

**ADC:**
```cpp
analogRead(A0);  // 12-bit, 0-4095
analogRead(A1);
```

## 📊 Strom- & Power-Budget

| Komponente | Stromaufnahme | Spannung |
|-----------|---|---|
| Teensy 4.1 | ~100 mA | 5V |
| IMU | 3.5 mA | 3.3V |
| Barometer | 1 mA | 3.3V |
| Drucksensor | 5 mA | 3.3V |
| Waagen (2x) | 10 mA | 3.3V |
| Motor (idle) | 50 mA | 5V |
| Motor (Last) | 200-500 mA | 5V |
| **Gesamt (idle)** | ~170 mA | |
| **Gesamt (mit Motor)** | ~400-800 mA | |

**Batterie-Empfehlung:**
- Kapazität: min. 1000 mAh (für 5V)
- Typ: Li-Po (4S = 14.8V nominal) mit Regler
- Laufzeit bei 500 mA: ~2 Stunden

## 🔧 Kalibrierung & Setup

### Waagen-Kalibrierung:
1. Belaste Waagen nicht → "Tare" aufrufen
2. Bekannte Last auf Waagen legen (z.B. 100g)
3. Kalibrierungsfaktor berechnen: `factor = known_weight / raw_adc_value`
4. Mit `setCalibrationFactor()` setzen

### Motor-Test:
```cpp
motor.setSpeed(50, MotorDirection::FORWARD);  // 50% vorwärts
delay(2000);
motor.brake();  // Bremsen
delay(500);
motor.stop();   // Stop
```

### CAN-Bus-Test:
```cpp
ArmCommand cmd = {45.0f, 90.0f, 50.0f, 0};  // 45°, 90°, 50% Gripper
can_bus.sendArmCommand(cmd);

ArmStatus status;
if (can_bus.receiveArmStatus(status)) {
    Serial.printf("J1: %.1f°, J2: %.1f°\n", 
                 status.joint1_position, 
                 status.joint2_position);
}
```

## ⚠️ Sicherheitsaspekte

1. **Motor-Sicherheit:**
   - Immer Enable Pin überprüfen
   - Notfall-Stop-Funktion (CAN: 0x101)
   - Timeout: Motor stoppt nach 2s ohne Signal

2. **CAN-Bus-Sicherheit:**
   - Heartbeat-Monitoring (STM32 muss alle 500ms senden)
   - Fehlerbehandlung für Nachrichten-Verlust
   - Arbitration bei ID-Kollisionen

3. **Stromversorgung:**
   - BEC (Battery Elimination Circuit) für 3.3V
   - Bulk-Kapazität (min. 100µF)
   - Ferritperle auf Motor-Stromkreis

## 📋 Pre-Flight Checklist

- [ ] I2C Sensor Scan (Wire Scanner)
- [ ] CAN-Bus Verbindung zu STM32 getestet
- [ ] Motor läuft in beide Richtungen
- [ ] Waagen tariert und kalibriert
- [ ] Drucksensor liest Werte
- [ ] Barometer zeigt realistische Höhe
- [ ] Alle Sensoren in Serial Monitor sichtbar
- [ ] Stromverbrauch im Budget
- [ ] Notfall-Stop reagiert
- [ ] Telemetrie wird auf SD-Karte gespeichert

---

**Hardware-Konfiguration für MAGGIE RoboArm Experiment**
**Zielflug:** REXUS Programm (ESA)
**Stand:** März 2026
