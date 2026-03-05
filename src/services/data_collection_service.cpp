#include "services/data_collection_service.h"

DataCollectionService::DataCollectionService() {
    // Konstruktor
}

DataCollectionService::~DataCollectionService() {
    // Destruktor
}

bool DataCollectionService::init() {
    bool success = true;
    
    if (!imu.init()) {
        Serial.println("ERROR: IMU initialization failed");
        success = false;
    } else {
        Serial.println("INFO: IMU initialized");
    }
    
    if (!barometer.init()) {
        Serial.println("ERROR: Barometer initialization failed");
        success = false;
    } else {
        Serial.println("INFO: Barometer initialized");
    }
    
    imu.setSampleRate(target_sample_rate);
    
    return success;
}

bool DataCollectionService::collectData(TelemetryData& data) {
    // Prüfe Sample-Rate
    uint32_t now = millis();
    uint32_t dt = now - last_sample_time;
    uint32_t min_interval = 1000 / target_sample_rate;
    
    if (dt < min_interval) {
        return false;  // Noch nicht bereit für nächsten Sample
    }
    
    last_sample_time = now;
    
    // Lese IMU
    IMUData imu_data;
    if (!imu.read(imu_data)) {
        Serial.println("ERROR: Failed to read IMU");
        return false;
    }
    
    // Lese Barometer
    BarometerData baro_data;
    if (!barometer.read(baro_data)) {
        Serial.println("ERROR: Failed to read barometer");
        return false;
    }
    
    // Fülle TelemetryData-Struktur
    data.accel_x = imu_data.accel_x;
    data.accel_y = imu_data.accel_y;
    data.accel_z = imu_data.accel_z;
    
    data.gyro_x = imu_data.gyro_x;
    data.gyro_y = imu_data.gyro_y;
    data.gyro_z = imu_data.gyro_z;
    
    data.mag_x = imu_data.mag_x;
    data.mag_y = imu_data.mag_y;
    data.mag_z = imu_data.mag_z;
    
    data.pressure = baro_data.pressure;
    data.temperature = baro_data.temperature;
    data.altitude = baro_data.altitude;
    
    data.timestamp = now;
    data.sequence = data_count++;
    data.sensor_status = getStatusByte();
    
    return true;
}

uint8_t DataCollectionService::getSensorStatus() const {
    return getStatusByte();
}

void DataCollectionService::calibrateAll() {
    Serial.println("INFO: Starting sensor calibration...");
    
    imu.calibrateAccel();
    delay(1000);
    
    imu.calibrateGyro();
    delay(1000);
    
    Serial.println("INFO: Calibration complete");
}

void DataCollectionService::setSampleRate(uint16_t rate_hz) {
    if (rate_hz < 1 || rate_hz > 1000) return;
    
    target_sample_rate = rate_hz;
    imu.setSampleRate(rate_hz);
}

uint32_t DataCollectionService::getDataPointCount() const {
    return data_count;
}

uint8_t DataCollectionService::getStatusByte() const {
    uint8_t status = 0;
    
    if (imu.isHealthy()) status |= 0x01;
    if (barometer.isHealthy()) status |= 0x02;
    
    return status;
}
