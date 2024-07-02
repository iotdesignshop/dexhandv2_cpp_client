/***
 * @file dexhand_servo.cpp
 * @brief DexHand Servo class implementation
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */


#include "dexhand_servo.hpp"

namespace dexhand_connect {

Servo::Servo(uint8_t id) : servoID(id) {}

Servo::Servo(uint8_t id, uint16_t hMinP, uint16_t hMaxP, uint16_t sMinP, uint16_t sMaxP, uint16_t hp, uint8_t maxLoad, uint8_t maxTemp) 
: servoID(id), hwMinPosition(hMinP), hwMaxPosition(hMaxP), swMinPosition(sMinP), swMaxPosition(sMaxP), homePosition(hp), maxLoadPct(maxLoad), maxTemp(maxTemp) {}

void Servo::setFullStatus(uint16_t position, int16_t speed, int16_t load, uint8_t temperature, uint8_t voltage, uint8_t status) {
    this->position = position;
    this->speed = speed;
    this->load = load;
    this->temperature = temperature;
    this->voltage = voltage;
    this->status = status;

    // If torque is disabled, adjust the target to be current to avoid jumps
    if (getTorqueEnable() == false) {
        target = position;
    }
}

void Servo::setDynamics(uint16_t position, int16_t speed, int16_t load, uint8_t status) {
    this->position = position;
    this->speed = speed;
    this->load = load;
    this->status = status;

    // If torque is disabled, adjust the target to be current to avoid jumps
    if (getTorqueEnable() == false) {
        target = position;
    }
}

} // namespace dexhand_connect