/***
 * @file dexhand_joint_angle_controller.hpp
 * @brief Joint angle controller for the DexHand allowing for approximate control of the hand joints by specifying angles in degrees. 
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
*/ 

#pragma once

#include <cstdint>
#include <string>

namespace dexhand_connect {
class DexhandConnect;
class ServoManager;

/// @brief Types of mixers available to joint angle controller
enum MixType {
    MIX_LINEAR, ///< Linear mapping between single angle and mechanical range
    MIX_FINGER, ///< Custom mapping for finger knuckle joints
    MIX_THUMB,   ///< Custom mapping for thumb joints
    NUM_MIX_TYPES
};
        
/// @brief Configuration for a given joint angle mapping on the hand
struct MechanicalMapping {
    float minAngle;
    float maxAngle;
    std::vector<uint16_t> servoIDs;
    MixType mixer;
};

/// @brief JointAngleController provides approximate joint angle control over the DexHand allowing
/// for control of the finger joints by specifying rotation angles in degrees. This is not an exact
/// control method, but provides a simple way to control the hand for basic tasks.
class JointAngleController {
    public:
        JointAngleController(ServoManager& servoManager);
        virtual ~JointAngleController() = default;

        /// @brief Joint IDs
        enum JointID {
            INDEX_PITCH,
            INDEX_YAW,
            INDEX_FLEXOR,
            MIDDLE_PITCH,
            MIDDLE_YAW,
            MIDDLE_FLEXOR,
            RING_PITCH,
            RING_YAW,
            RING_FLEXOR,
            PINKY_PITCH,
            PINKY_YAW,
            PINKY_FLEXOR,
            THUMB_PITCH,
            THUMB_YAW,
            THUMB_FLEXOR,
            NUM_JOINT_IDS
        };

        const std::vector<std::string> jointNames = {
            "index_pitch",
            "index_yaw",
            "index_flexor",
            "middle_pitch",
            "middle_yaw",
            "middle_flexor",
            "ring_pitch",
            "ring_yaw",
            "ring_flexor",
            "pinky_pitch",
            "pinky_yaw",
            "pinky_flexor",
            "thumb_pitch",
            "thumb_yaw",
            "thumb_flexor"
        };

        /// @brief Start the joint angle controller
        void start();

        /// @brief Get the joint ID from a joint name
        JointID getJointID(const std::string& jointName) const;

        /// @brief Set a joint angle
        void setJointAngle(JointID jointID, float angle);

        /// @brief Get the minimum angle of a joint
        /// @param jointID Joint ID
        /// @return Minimum angle
        float getJointMin(JointID jointID) const { return (jointID < NUM_JOINT_IDS)?mechanicalMappings[jointID].minAngle:0.0; }

        /// @brief Get the maximum angle of a joint
        /// @param jointID Joint ID
        /// @return Maximum angle
        float getJointMax(JointID jointID) const { return (jointID < NUM_JOINT_IDS)?mechanicalMappings[jointID].maxAngle:0.0; }

        /// @brief Set the range of motion for a joint - used to fine tune joint ranges
        /// @param jointID Joint ID
        /// @param min Minimum angle
        void setJointRange(JointID jointID, float min, float max);

        /// @brief Get the handedness of the hand
        bool isRightHand() const { return rightHand; }

    private:
        ServoManager& sm;
        bool rightHand = true;
        float jointAngles[NUM_JOINT_IDS] = {0.0f};

        

        // Defaults table - safe mappings for standard hand configuration
        MechanicalMapping mechanicalMappings[NUM_JOINT_IDS] = {
            {0.0f, 75.0f, {101,102}, MixType::MIX_FINGER},       // index_pitch
            {-20.0f, 20.0f, {101, 102}, MixType::MIX_FINGER},    // index_yaw
            {0.0f, 60.0f, {111}, MixType::MIX_LINEAR},           // index_flexor
            {0.0f, 75.0f, {103,104}, MixType::MIX_FINGER},       // middle_pitch
            {-20.0f, 20.0f, {103, 104}, MixType::MIX_FINGER},    // middle_yaw
            {0.0f, 60.0f, {112}, MixType::MIX_LINEAR},           // middle_flexor
            {0.0f, 75.0f, {105,106}, MixType::MIX_FINGER},       // ring_pitch
            {-20.0f, 20.0f, {105, 106}, MixType::MIX_FINGER},    // ring_yaw
            {0.0f, 60.0f, {113}, MixType::MIX_LINEAR},           // ring_flexor
            {0.0f, 75.0f, {107,108}, MixType::MIX_FINGER},       // pinky_pitch
            {-20.0f, 20.0f, {107, 108}, MixType::MIX_FINGER},    // pinky_yaw
            {0.0f, 60.0f, {114}, MixType::MIX_LINEAR},           // pinky_flexor
            {0.0f, 60.0f, {109,110,115}, MixType::MIX_THUMB},   // thumb_pitch
            {-30.0f, 30.0f, {109,110,115}, MixType::MIX_THUMB},// thumb_yaw
            {0.0f, 60.0f, {109,110,115}, MixType::MIX_THUMB}   // thumb_flexor
        };


       

};





}