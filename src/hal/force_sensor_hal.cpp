#include "hal/force_sensor_hal.hpp"

ForceSensorHAL::ForceSensorHAL(uint8_t pin_x, uint8_t pin_y, uint8_t pin_z)
    : pin_x_(pin_x), pin_y_(pin_y), pin_z_(pin_z) {
}

bool ForceSensorHAL::init() {
    // Configure pins as inputs
    pinMode(pin_x_, INPUT);
    pinMode(pin_y_, INPUT);
    pinMode(pin_z_, INPUT);
    
    initialized_ = true;
    return true;
}

bool ForceSensorHAL::read(ForceSensorReading& reading,
                          float calibration_x,
                          float calibration_y,
                          float calibration_z) {
    if (!initialized_) {
        return false;
    }
    
    // Read raw ADC values
    reading.raw_x = analogRead(pin_x_);
    reading.raw_y = analogRead(pin_y_);
    reading.raw_z = analogRead(pin_z_);
    
    // Apply offset and calibration
    int32_t adjusted_x = static_cast<int32_t>(reading.raw_x) - static_cast<int32_t>(offset_x_);
    int32_t adjusted_y = static_cast<int32_t>(reading.raw_y) - static_cast<int32_t>(offset_y_);
    int32_t adjusted_z = static_cast<int32_t>(reading.raw_z) - static_cast<int32_t>(offset_z_);
    
    // Convert to force values
    reading.force_x = static_cast<float>(adjusted_x) * calibration_x;
    reading.force_y = static_cast<float>(adjusted_y) * calibration_y;
    reading.force_z = static_cast<float>(adjusted_z) * calibration_z;
    
    reading.timestamp = millis();
    reading.valid = true;
    
    return true;
}

void ForceSensorHAL::setOffset(uint32_t offset_x, uint32_t offset_y, uint32_t offset_z) {
    offset_x_ = offset_x;
    offset_y_ = offset_y;
    offset_z_ = offset_z;
}

void ForceSensorHAL::getOffset(uint32_t& offset_x, uint32_t& offset_y, uint32_t& offset_z) const {
    offset_x = offset_x_;
    offset_y = offset_y_;
    offset_z = offset_z_;
}
