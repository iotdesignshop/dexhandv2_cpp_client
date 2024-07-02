/***
 * @file dexhand_servo.hpp
 * @brief The DexHand Servo class abstracts a single servo on the DexHand and provides methods to read and set the servo state.
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#pragma once

#include <cstdint>
#include <algorithm>
#include <assert.h>
#include "dexhand_message.hpp"

namespace dexhand_connect {
class ServoManager;

/// @brief Class to represent a single servo on the DexHand. This class is with the ServoManager to read and write the
/// servo state at an abstract level. When using the ServoManager, the Servo objects are created and managed by the
/// ServoManager, and the user interacts with the objects to read and set the servo state.
class Servo
{
public:
    Servo(uint8_t id);
    Servo(uint8_t id, uint16_t hwMinPosition, uint16_t hwMaxPosition, uint16_t swMinPosition, uint16_t swMaxPosition, uint16_t homePosition, uint8_t maxLoadPct, uint8_t maxTemp);
    virtual ~Servo() = default;

    /// @brief Retrieve the hardware ID of this servo
    /// @return Servo ID
    uint8_t getID() const { return servoID; }

    /// @brief Get current status of the servo
    /// @return Bitfield of status flags
    /// @see ServoStatusFlags in dexhand_message.hpp for definitions of the status flags
    uint8_t getStatus() const { return status; }

    /// @brief Get the current position of the servo
    /// @return Value from 0-4095 representing the servo position
    uint16_t getPosition() const { return position; }

    /// @brief Get the current speed of the servo
    /// @return Current speed
    int16_t getSpeed() const { return speed; }

    /// @brief Get the current load on the servo
    /// @return Current load in mA (0-1000mA)
    int16_t getLoad() const { return load; }

    /// @brief Get the current voltage of the servo
    /// @return Current voltage in volts*10, ie 60 = 6.0V
    uint8_t getVoltage() const { return voltage; }

    /// @brief Get the current temperature of the servo
    /// @return Current temperature in degrees C
    uint8_t getTemperature() const { return temperature; }

    /// @brief Get the target position of the servo
    /// @return Target position
    uint16_t getTarget() const { return target; }

    /// @brief Set the target position of the servo
    /// @param t Target position (0-4095)
    void setTarget(uint16_t t) { target = clampToSWLimits(t); }

    /// @brief Get the target position normalized to the software limits
    /// @return Normalized target position (0-1)
    float getTargetNormalized() const { return static_cast<float>(target - swMinPosition) / (swMaxPosition - swMinPosition); }  

    /// @brief Set the target position normalized to the software limits
    /// @param t Normalized target position (0-1)
    void setTargetNormalized(float t) { setTarget(static_cast<uint16_t>(clampN(t) * (swMaxPosition - swMinPosition) + swMinPosition)); }

    /// @brief Get the hardware minimim position of the servo. You can't move the servo below this position.
    /// @return Position (0-4095)
    uint16_t getHWMinPosition() const { return hwMinPosition; }

    /// @brief Get the hardware maximum position of the servo. You can't move the servo above this position.
    /// @return Position (0-4095)
    uint16_t getHWMaxPosition() const { return hwMaxPosition; }

    /// @brief Get the software minimum position of the servo. This is a soft endpoint to tune the lower range of motion.
    /// @return Position (0-4095)
    uint16_t getSWMinPosition() const { return swMinPosition; }

    /// @brief  Set the software minimum position of the servo to tune it's lower range of motion. This value must be within the
    /// hardware limits.
    /// @param pos Position (0-4095)
    void setSWMinPosition(uint16_t pos) { assert(swMinPosition <= swMaxPosition); swMinPosition = clampToHWLimits(pos); }

    /// @brief Get the software maximum position of the servo. This is a soft endpoint to tune the upper range of motion.
    /// @return Position (0-4095)
    uint16_t getSWMaxPosition() const { return swMaxPosition; }

    /// @brief Set the software maximum position of the servo to tune it's upper range of motion. This value must be within the
    /// hardware limits.
    /// @param pos Position (0-4095)
    void setSWMaxPosition(uint16_t pos) { assert(swMaxPosition <= swMinPosition); swMaxPosition = clampToHWLimits(pos); }

    /// @brief Get the home position of the servo. This is the position the servo will move to when the hand is reset.
    uint16_t getHomePosition() const { return homePosition; }

    /// @brief Set the home position of the servo. This position must be within the soft limits.
    /// @param pos Position (0-4095)
    void setHomePosition(uint16_t pos) { homePosition = clampToSWLimits(pos); }

    /// @brief Get the maximum load percentage of the servo. If the load exceeds this percentage for a couple seconds, 
    /// the servo will stop moving and reduce torque to protect itself.
    /// @return Load percentage (0-100)
    uint8_t getMaxLoadPct() const { return maxLoadPct; }

    /// @brief Set the maximum load percentage of the servo. If the load exceeds this percentage for a couple seconds,
    /// the servo will stop moving and reduce torque to protect itself.
    /// @param pct Load percentage (0-100)
    void setMaxLoadPct(uint8_t pct) { maxLoadPct = std::min(pct,(uint8_t)100); }

    /// @brief Get the maximum temperature of the servo. If the temperature exceeds this value, the servo will stop moving
    /// and reduce torque to protect itself.
    /// @return Temperature in degrees C
    uint8_t getMaxTemp() const { return maxTemp; }

    /// @brief Set the maximum temperature of the servo. If the temperature exceeds this value, the servo will stop moving
    /// and reduce torque to protect itself.
    /// @param temp Temperature in degrees C
    void setMaxTemp(uint8_t temp) { maxTemp = std::min(temp,(uint8_t)100); }

    /// @brief Get the torque enable status of the servo
    bool getTorqueEnable() const { return (getStatus() & static_cast<uint8_t>(ServoStatusFlags::TORQUE_OFF)) == 0; }

    /// @brief Set the torque enable status of the servo
    /// @param enable True to enable torque, false to disable
    void setTorqueEnable(bool enable) { torqueEnableRequest = enable; }

    friend class FullServoStatusSubscriber;
    friend class DynamicsSubscriber;
    friend class ServoManager;

protected:

    void setFullStatus(uint16_t position, int16_t speed, int16_t load, uint8_t temperature, uint8_t voltage, uint8_t status);
    void setDynamics(uint16_t position, int16_t speed, int16_t load, uint8_t status);
    bool getTorqueEnableRequest() const { return torqueEnableRequest; }

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
    bool torqueEnableRequest = true;

    uint16_t clampToSWLimits(uint16_t pos) const { return pos < swMinPosition ? swMinPosition : (pos > swMaxPosition ? swMaxPosition : pos);}
    uint16_t clampToHWLimits(uint16_t pos) const { return pos < hwMinPosition ? hwMinPosition : (pos > hwMaxPosition ? hwMaxPosition : pos);}
    float clampN(float pos) const { return pos < 0.0f ? 0.0f : (pos > 1.0f ? 1.0f : pos); }
};




} // namespace dexhand_connect