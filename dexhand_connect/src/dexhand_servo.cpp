/// @file dexhand_servo.cpp
/// @brief DexHand Servo class implementation


#include "dexhand_servo.hpp"

namespace dexhand_connect {

Servo::Servo(uint8_t id) : servoID(id) {}

void Servo::initVars(uint16_t hwMinPosition, uint16_t hwMaxPosition, uint16_t swMinPosition, uint16_t swMaxPosition, uint16_t homePosition, uint8_t maxLoadPct, uint8_t maxTemp) {
    this->hwMinPosition = hwMinPosition;
    this->hwMaxPosition = hwMaxPosition;
    this->swMinPosition = swMinPosition;
    this->swMaxPosition = swMaxPosition;
    this->homePosition = homePosition;
    this->maxLoadPct = maxLoadPct;
    this->maxTemp = maxTemp;
}

void Servo::setFullStatus(uint16_t position, int16_t speed, int16_t load, uint8_t temperature, uint8_t voltage, uint8_t status) {
    this->position = position;
    this->speed = speed;
    this->load = load;
    this->temperature = temperature;
    this->voltage = voltage;
    this->status = status;
}

void Servo::setDynamics(uint16_t position, int16_t speed, int16_t load, uint8_t status) {
    this->position = position;
    this->speed = speed;
    this->load = load;
    this->status = status;
}

} // namespace dexhand_connect