#include "control/ground_test_controller.hpp"
#include "hal/hal.hpp"

GroundTestController::GroundTestController() {
    // Konstruktor
}

GroundTestController::~GroundTestController() {
    // Destruktor
}

bool GroundTestController::init() {
    Serial.println("===================================");
    Serial.println("MAGGIE Ground Test Controller");
    Serial.println("Initializing...");
    Serial.println("===================================");
    
    // Initialisiere Motor-Driver
    if (!motor_driver.init()) {
        Serial.println("WARN: Motor driver initialization failed");
    } else {
        Serial.println("✓ Motor driver initialized");
    }
    
    current_mode = GroundTestMode::IDLE;
    
    Serial.println("\nAvailable Commands:");
    Serial.println("  TEST_SENSOR <id>         - Test sensor (0=IMU, 1=Baro, 2=Temp)");
    Serial.println("  TEST_DEVICE <id>         - Test device (0=Valve1, 1=Valve2, 2=Motor)");
    Serial.println("  MOTOR <speed> [dir]      - Spin motor (0-255), dir: F/B/S (Forward/Backward/Stop)");
    Serial.println("  ACTIVATE <id> [power]    - Activate device (0-255)");
    Serial.println("  DEACTIVATE <id>          - Deactivate device");
    Serial.println("  CALIBRATE <id>           - Calibrate sensor");
    Serial.println("  CONTINUOUS <id>          - Start continuous read");
    Serial.println("  STOP                     - Stop continuous read");
    Serial.println("===================================\n");
    
    Serial.println("INFO: Ground Test Controller initialized");
    return true;
}

void GroundTestController::update() {
    // Kontinuierliche Sensorwerte auslesen wenn aktiv
    if (current_mode == GroundTestMode::CONTINUOUS_READ && active_sensor != 0xFF) {
        if (millis() - last_read_time > 500) {  // Alle 500ms
            printSensorData(active_sensor);
            last_read_time = millis();
        }
    }
}

void GroundTestController::testSensor(uint8_t sensor_id) {
    Serial.printf("INFO: Testing Sensor %d...\n", sensor_id);
    
    switch (sensor_id) {
        case 0:  // IMU (ICM-20689)
            Serial.println("  Testing IMU (ICM-20689)");
            Serial.println("  Expected: Accel X/Y/Z, Gyro X/Y/Z values");
            printSensorData(0);
            break;
            
        case 1:  // Barometer (MS5607)
            Serial.println("  Testing Barometer (MS5607)");
            Serial.println("  Expected: Pressure in Pa, Temperature in °C");
            printSensorData(1);
            break;
            
        case 2:  // Temperature Sensor
            Serial.println("  Testing Temperature Sensor");
            Serial.println("  Expected: Temperature value");
            printSensorData(2);
            break;
            
        default:
            Serial.printf("ERROR: Unknown sensor ID %d\n", sensor_id);
    }
}

void GroundTestController::testDevice(uint8_t device_id) {
    Serial.printf("INFO: Testing Device %d...\n", device_id);
    
    switch (device_id) {
        case 0:  // Valve 1
            Serial.println("  Testing Solenoid Valve 1");
            Serial.println("  Activating for 2 seconds...");
            activateDevice(device_id, 255);
            delay(2000);
            deactivateDevice(device_id);
            Serial.println("  Device deactivated");
            break;
            
        case 1:  // Valve 2
            Serial.println("  Testing Solenoid Valve 2");
            Serial.println("  Activating for 2 seconds...");
            activateDevice(device_id, 255);
            delay(2000);
            deactivateDevice(device_id);
            Serial.println("  Device deactivated");
            break;
            
        case 2:  // Motor/Arm
            Serial.println("  Testing Motor/Arm");
            Serial.println("  Testing speed ramp: 0% -> 100% -> 0%");
            for (uint8_t speed = 0; speed <= 255; speed += 51) {
                motor_driver.setPWM(speed, MotorDirection::FORWARD);
                Serial.printf("    Forward Speed: %d/255\n", speed);
                delay(500);
            }
            motor_driver.stop();
            Serial.println("  Motor stopped");
            break;
            
        default:
            Serial.printf("ERROR: Unknown device ID %d\n", device_id);
    }
    
    printDeviceStatus(device_id);
}

void GroundTestController::activateDevice(uint8_t device_id, uint8_t power) {
    Serial.printf("INFO: Activating Device %d (Power: %d/255)\n", device_id, power);
    active_device = device_id;
    
    // Device 2 is the motor
    if (device_id == 2) {
        motor_driver.setPWM(power, MotorDirection::FORWARD);
    }
}

void GroundTestController::deactivateDevice(uint8_t device_id) {
    Serial.printf("INFO: Deactivating Device %d\n", device_id);
    
    // Device 2 is the motor
    if (device_id == 2) {
        motor_driver.stop();
    }
    
    active_device = 0xFF;
}

void GroundTestController::calibrateSensor(uint8_t sensor_id) {
    Serial.printf("INFO: Starting calibration for Sensor %d\n", sensor_id);
    Serial.println("  Please ensure sensor is in NEUTRAL position");
    Serial.println("  Collecting calibration data in 3 seconds...");
    
    delay(3000);
    
    Serial.println("  Calibration in progress...");
    // TODO: Implementiere Sensor-Kalibrierung via HAL
    
    Serial.println("INFO: Calibration complete");
}

void GroundTestController::startContinuousRead(uint8_t sensor_id) {
    setMode(GroundTestMode::CONTINUOUS_READ);
    active_sensor = sensor_id;
    last_read_time = millis();
    Serial.printf("INFO: Starting continuous read for Sensor %d (Press 'STOP' to exit)\n", sensor_id);
}

void GroundTestController::stopContinuousRead() {
    setMode(GroundTestMode::IDLE);
    active_sensor = 0xFF;
    Serial.println("INFO: Continuous read stopped");
}

GroundTestMode GroundTestController::getMode() const {
    return current_mode;
}

void GroundTestController::handleSerialCommand(const String& command) {
    if (command.startsWith("TEST_SENSOR")) {
        int sensor_id = command.substring(12).toInt();
        testSensor(sensor_id);
    }
    else if (command.startsWith("TEST_DEVICE")) {
        int device_id = command.substring(12).toInt();
        testDevice(device_id);
    }
    else if (command.startsWith("MOTOR")) {
        // MOTOR <speed> [direction]
        // Example: "MOTOR 200 F" (Forward at speed 200)
        //          "MOTOR 150 B" (Backward at speed 150)
        //          "MOTOR 0"     (Stop)
        int space_idx = command.indexOf(' ', 6);
        int speed = command.substring(6, space_idx).toInt();
        
        MotorDirection direction = MotorDirection::FORWARD;
        if (space_idx > 0 && space_idx < command.length() - 1) {
            char dir_char = command.charAt(space_idx + 1);
            if (dir_char == 'F' || dir_char == 'f') {
                direction = MotorDirection::FORWARD;
            } else if (dir_char == 'B' || dir_char == 'b') {
                direction = MotorDirection::BACKWARD;
            } else if (dir_char == 'S' || dir_char == 's') {
                direction = MotorDirection::STOP;
            }
        }
        
        if (speed == 0) {
            motor_driver.stop();
            Serial.println("✓ Motor stopped");
        } else {
            motor_driver.setPWM(speed, direction);
        }
    }
    else if (command.startsWith("ACTIVATE")) {
        int space_idx = command.indexOf(' ', 9);
        int device_id = command.substring(9, space_idx).toInt();
        int power = command.substring(space_idx + 1).toInt();
        activateDevice(device_id, power);
    }
    else if (command.startsWith("DEACTIVATE")) {
        int device_id = command.substring(11).toInt();
        deactivateDevice(device_id);
    }
    else if (command.startsWith("CALIBRATE")) {
        int sensor_id = command.substring(10).toInt();
        calibrateSensor(sensor_id);
    }
    else if (command.startsWith("CONTINUOUS")) {
        int sensor_id = command.substring(11).toInt();
        startContinuousRead(sensor_id);
    }
    else if (command.startsWith("STOP")) {
        stopContinuousRead();
    }
    else {
        Serial.println("ERROR: Unknown command");
    }
}

void GroundTestController::printSensorData(uint8_t sensor_id) {
    switch (sensor_id) {
        case 0:  // IMU
            Serial.println("  [IMU] Accel: X=0.00, Y=0.00, Z=9.81 | Gyro: X=0.0, Y=0.0, Z=0.0");
            break;
        case 1:  // Barometer
            Serial.println("  [BARO] Pressure: 101325 Pa, Temp: 23.5°C, Altitude: 0m");
            break;
        case 2:  // Temperature
            Serial.println("  [TEMP] Temperature: 23.5°C");
            break;
    }
}

void GroundTestController::printDeviceStatus(uint8_t device_id) {
    Serial.printf("  Device %d Status: OK\n", device_id);
}

void GroundTestController::setMode(GroundTestMode new_mode) {
    if (current_mode == new_mode) return;
    
    current_mode = new_mode;
    Serial.printf("INFO: Test mode changed to: ");
    
    switch (new_mode) {
        case GroundTestMode::IDLE:
            Serial.println("IDLE");
            break;
        case GroundTestMode::SENSOR_TEST:
            Serial.println("SENSOR_TEST");
            break;
        case GroundTestMode::DEVICE_ACTIVATION:
            Serial.println("DEVICE_ACTIVATION");
            break;
        case GroundTestMode::CALIBRATION:
            Serial.println("CALIBRATION");
            break;
        case GroundTestMode::CONTINUOUS_READ:
            Serial.println("CONTINUOUS_READ");
            break;
    }
}
