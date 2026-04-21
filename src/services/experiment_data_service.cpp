#include "services/experiment_data_service.hpp"

ExperimentDataService::ExperimentDataService()
    : imu(),  // BMI088 mit Default-Pins (Accel=10, Gyro=9)
      pressure_sensor(10),  // CS Pin 10 für SPI
      weight_sensors(A0, A1),  // Analog Pins A0, A1 für Waagen
      motor(9, 7, 8)  // PWM Pin 9, Dir Pin 7, Dir Pin 8
{
    // Konstruktor
}

ExperimentDataService::~ExperimentDataService() {
    // Destruktor
}

bool ExperimentDataService::init() {
    bool success = true;
    
    // Initialisiere CAN-Bus zuerst (kritisch für STM32 Kommunikation)
    if (!can_bus.init(500000)) {
        Serial.println("ERROR: CAN-Bus initialization failed");
        success = false;
    } else {
        Serial.println("INFO: CAN-Bus initialized");
    }
    
    // Initialisiere IMU
    if (!imu.init()) {
        Serial.println("ERROR: IMU initialization failed");
        success = false;
    } else {
        Serial.println("INFO: IMU initialized");
    }
    
    // Initialisiere Barometer
    if (!barometer.init()) {
        Serial.println("ERROR: Barometer initialization failed");
        success = false;
    } else {
        Serial.println("INFO: Barometer initialized");
    }
    
    // Initialisiere Drucksensor
    if (!pressure_sensor.init("MS5607")) {
        Serial.println("WARNING: Pressure sensor initialization failed");
    } else {
        Serial.println("INFO: Pressure sensor initialized");
    }
    
    // Initialisiere Waagen
    if (!weight_sensors.init()) {
        Serial.println("WARNING: Weight sensors initialization failed");
    } else {
        Serial.println("INFO: Weight sensors initialized");
    }
    
    // Initialisiere Motor
    if (!motor.init(1000)) {  // 1 kHz PWM Frequenz
        Serial.println("ERROR: Motor initialization failed");
        success = false;
    } else {
        Serial.println("INFO: Motor initialized");
    }
    
    return success;
}

bool ExperimentDataService::collectData(ExperimentTelemetry& data) {
    // Prüfe Sample-Rate
    uint32_t now = millis();
    uint32_t dt = now - last_sample_time;
    uint32_t min_interval = 1000 / target_sample_rate;
    
    if (dt < min_interval) {
        return false;  // Noch nicht bereit
    }
    
    last_sample_time = now;
    
    // Lese IMU
    IMUData imu_data;
    if (!imu.read(imu_data)) {
        Serial.println("ERROR: Failed to read IMU");
        return false;
    }
    data.accel_x = imu_data.accel_x;
    data.accel_y = imu_data.accel_y;
    data.accel_z = imu_data.accel_z;
    data.gyro_x = imu_data.gyro_x;
    data.gyro_y = imu_data.gyro_y;
    data.gyro_z = imu_data.gyro_z;
    
    // Lese Barometer
    BarometerData baro_data;
    if (barometer.read(baro_data)) {
        data.barometer_pressure = baro_data.pressure;
        data.barometer_temperature = baro_data.temperature;
        data.altitude = baro_data.altitude;
    }
    
    // Lese Drucksensor
    PressureReading pressure_data;
    if (pressure_sensor.read(pressure_data)) {
        data.pressure_sensor = pressure_data.pressure;
        data.pressure_temp = pressure_data.temperature;
    }
    
    // Lese Waagen
    ScaleReading scale1, scale2;
    if (weight_sensors.readBothScales(scale1, scale2)) {
        data.weight_scale1 = scale1.weight;
        data.weight_scale2 = scale2.weight;
        data.weight_total = scale1.weight + scale2.weight;
    }
    
    // Motor Status
    data.motor_speed = motor.getSpeed();
    data.motor_direction = (uint8_t)motor.getDirection();
    
    // CAN-Bus Status
    data.can_connected = can_bus.isConnected();
    
    // Allgemein
    data.timestamp = now;
    data.sequence = data_count++;
    
    last_telemetry = data;
    return true;
}

bool ExperimentDataService::sendArmCommand(const ArmCommand& command) {
    return can_bus.sendArmCommand(command);
}

bool ExperimentDataService::receiveArmStatus(ArmStatus& status) {
    return can_bus.receiveArmStatus(status);
}

void ExperimentDataService::controlMotor(uint8_t speed, uint8_t direction) {
    MotorDirection dir = MotorDirection::STOP;
    
    switch (direction) {
        case 1: dir = MotorDirection::FORWARD; break;
        case 2: dir = MotorDirection::BACKWARD; break;
        default: dir = MotorDirection::STOP;
    }
    
    motor.setSpeed(speed, dir);
}

void ExperimentDataService::tareWeightSensors() {
    weight_sensors.tareScale1();
    weight_sensors.tareScale2();
    Serial.println("INFO: Weight sensors tared");
}

float ExperimentDataService::getTotalWeight() const {
    return weight_sensors.getTotalWeight();
}

uint8_t ExperimentDataService::getSensorStatus() const {
    return getSensorStatusByte();
}

uint8_t ExperimentDataService::getSensorStatusByte() const {
    uint8_t status = 0;
    
    if (imu.isHealthy()) status |= 0x01;
    if (barometer.isHealthy()) status |= 0x02;
    if (pressure_sensor.isHealthy()) status |= 0x04;
    if (weight_sensors.isHealthy()) status |= 0x08;
    if (can_bus.isConnected()) status |= 0x10;
    
    return status;
}
