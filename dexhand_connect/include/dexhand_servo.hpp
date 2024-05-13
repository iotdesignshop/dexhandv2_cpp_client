/// @file dexhand_servo.hpp
/// @brief DexHand Servo class

#pragma once

#include <cstdint>
#include <algorithm>
#include <assert.h>

namespace dexhand_connect {
class ServoManager;

class Servo
{
public:
    Servo(uint8_t id);
    Servo(uint8_t id, uint16_t hwMinPosition, uint16_t hwMaxPosition, uint16_t swMinPosition, uint16_t swMaxPosition, uint16_t homePosition, uint8_t maxLoadPct, uint8_t maxTemp);
    virtual ~Servo() = default;

    uint8_t getID() const { return servoID; }
    uint8_t getStatus() const { return status; }
    uint16_t getPosition() const { return position; }
    int16_t getSpeed() const { return speed; }
    int16_t getLoad() const { return load; }
    uint8_t getVoltage() const { return voltage; }
    uint8_t getTemperature() const { return temperature; }

    uint16_t getTarget() const { return target; }
    void setTarget(uint16_t t) { target = clampToSWLimits(t); }

    uint16_t getHWMinPosition() const { return hwMinPosition; }
    uint16_t getHWMaxPosition() const { return hwMaxPosition; }

    uint16_t getSWMinPosition() const { return swMinPosition; }
    void setSWMinPosition(uint16_t pos) { assert(swMinPosition <= swMaxPosition); swMinPosition = clampToHWLimits(pos); }

    uint16_t getSWMaxPosition() const { return swMaxPosition; }
    void setSWMaxPosition(uint16_t pos) { assert(swMaxPosition <= swMinPosition); swMaxPosition = clampToHWLimits(pos); }

    uint16_t getHomePosition() const { return homePosition; }
    void setHomePosition(uint16_t pos) { homePosition = clampToSWLimits(pos); }

    uint8_t getMaxLoadPct() const { return maxLoadPct; }
    void setMaxLoadPct(uint8_t pct) { maxLoadPct = std::min(pct,(uint8_t)100); }

    uint8_t getMaxTemp() const { return maxTemp; }
    void setMaxTemp(uint8_t temp) { maxTemp = std::min(temp,(uint8_t)100); }

    friend class FullServoStatusSubscriber;
    friend class DynamicsSubscriber;

protected:

    void setFullStatus(uint16_t position, int16_t speed, int16_t load, uint8_t temperature, uint8_t voltage, uint8_t status);
    void setDynamics(uint16_t position, int16_t speed, int16_t load, uint8_t status);
     
    

private:
    uint8_t servoID = 0;
    uint16_t position = 0;
    int16_t speed = 0;
    int16_t load = 0;
    uint8_t temperature = 0;
    uint8_t voltage = 0;
    uint8_t status = 0;
    uint16_t target = 0;

    uint16_t hwMinPosition = 0;
    uint16_t hwMaxPosition = 0;
    uint16_t swMinPosition = 0;
    uint16_t swMaxPosition = 0;
    uint16_t homePosition = 0;
    uint8_t maxLoadPct = 0;
    uint8_t maxTemp = 0;

    uint16_t clampToSWLimits(uint16_t pos) const { return pos < swMinPosition ? swMinPosition : (pos > swMaxPosition ? swMaxPosition : pos);}
    uint16_t clampToHWLimits(uint16_t pos) const { return pos < hwMinPosition ? hwMinPosition : (pos > hwMaxPosition ? hwMaxPosition : pos);}
};




} // namespace dexhand_connect