/***
 * @file dexhand_joint_angle_controller.cpp
 * @brief The JointAngleController class provides approximate joint angle control over the DexHand
 * @copyright Copyright (c) 2024 IoT Design Shop Inc.
 */

#include <algorithm>
#include <cctype>

#include "dexhand_servomgr.hpp"
#include "dexhand_joint_angle_controller.hpp"


using namespace std;

namespace dexhand_connect {


using Mixer = void(*)(JointAngleController::JointID joint, const MechanicalMapping* mapping, const float* currentAngles, ServoManager& sm);

float clamp(float angle, const MechanicalMapping& mm) {
    if (angle < mm.minAngle) angle = mm.minAngle;
    if (angle > mm.maxAngle) angle = mm.maxAngle;
    return angle;
}

float clampAndNormalize(float angle, const MechanicalMapping& mm) {
    return (clamp(angle, mm) - mm.minAngle) / (mm.maxAngle - mm.minAngle);
}

void linearMixer(JointAngleController::JointID joint, const MechanicalMapping* mapping, const float* currentAngles, ServoManager& sm) {
    // Get angle and clamp to limits
    MechanicalMapping mm = mapping[joint];
    float angle = clampAndNormalize(currentAngles[joint], mm);
    
    // Set servo in normalized range
    auto servo = sm.getServo(mm.servoIDs[0]);
    if (servo) {
        servo->setTargetNormalized(angle);
    }
}

void fingerMixer(JointAngleController::JointID joint, const MechanicalMapping* mapping, const float* currentAngles, ServoManager& sm) {
    
    float pitchAngle = 0.0;
    float yawAngle = 0.0;
    MechanicalMapping pm, ym;

    // Get pitch and yaw angles
    switch (joint) {
        case JointAngleController::INDEX_PITCH:
        case JointAngleController::MIDDLE_PITCH:
        case JointAngleController::PINKY_PITCH:
        case JointAngleController::RING_PITCH:
            pitchAngle = currentAngles[joint];
            yawAngle = currentAngles[joint + 1];
            pm = mapping[joint];
            ym = mapping[joint + 1];
            break;

        case JointAngleController::INDEX_YAW:
        case JointAngleController::MIDDLE_YAW:
        case JointAngleController::PINKY_YAW:
        case JointAngleController::RING_YAW:
            pitchAngle = currentAngles[joint - 1];
            yawAngle = currentAngles[joint];
            pm = mapping[joint - 1];
            ym = mapping[joint];
            break;

        default:
            // Invalid joint
            return;
    }

    // Normalize angles
    float pNorm = clampAndNormalize(pitchAngle, pm);
    float yNorm = clamp(yawAngle, ym);
    yNorm = yNorm  / ym.maxAngle; // Center around 0.0
    
    // Scale yaw by pitch, as we want fingers to center when closed
    yNorm = yNorm * (1.0 - pNorm);

    // Attenuate yaw - +/- 20 degrees is approximately 30% of normalized range of servo travel
    yNorm *= 0.3;

    // Set servos in normalized range
    sm.getServo(pm.servoIDs[0])->setTargetNormalized(pNorm - yNorm);
    sm.getServo(pm.servoIDs[1])->setTargetNormalized(pNorm + yNorm);
}

void thumbMixer(JointAngleController::JointID joint, const MechanicalMapping* mapping, const float* currentAngles, ServoManager& sm) {
    float pitchAngle = 0.0;
    float yawAngle = 0.0;
    float flexorAngle = 0.0;
    MechanicalMapping pm, ym, fm;

    // Get pitch and yaw angles
    switch (joint) {
        case JointAngleController::THUMB_PITCH:
            pitchAngle = currentAngles[joint];
            yawAngle = currentAngles[joint + 1];
            flexorAngle = currentAngles[joint + 2];
            pm = mapping[joint];
            ym = mapping[joint + 1];
            fm = mapping[joint + 2];
            break;

        case JointAngleController::THUMB_YAW:
            pitchAngle = currentAngles[joint - 1];
            yawAngle = currentAngles[joint];
            flexorAngle = currentAngles[joint + 1];
            pm = mapping[joint - 1];
            ym = mapping[joint];
            fm = mapping[joint + 1];
            break;

        case JointAngleController::THUMB_FLEXOR:
            pitchAngle = currentAngles[joint - 2];
            yawAngle = currentAngles[joint - 1];
            flexorAngle = currentAngles[joint];
            pm = mapping[joint - 2];
            ym = mapping[joint - 1];
            fm = mapping[joint];
            break;

        default:
            // Invalid joint
            return;
    }

    // Normalize angles
    pitchAngle = clampAndNormalize(pitchAngle, pm);
    flexorAngle = clampAndNormalize(flexorAngle, fm);
    yawAngle = clamp(yawAngle, ym);
    yawAngle = yawAngle / (ym.maxAngle); // Center around 0.0
    
    // Yaw of 30 degrees is approximately 50% of normalized range of servo travel
    yawAngle *= 0.5;

    // Flexor needs some gain applied if pitch is high
    flexorAngle = MIN(flexorAngle + 0.5* pitchAngle, 1.0);

    // Set servos in normalized range
    sm.getServo(pm.servoIDs[0])->setTargetNormalized(pitchAngle + yawAngle);
    sm.getServo(pm.servoIDs[1])->setTargetNormalized(pitchAngle - yawAngle);
    sm.getServo(pm.servoIDs[2])->setTargetNormalized(flexorAngle);
}

static Mixer mixers[NUM_MIX_TYPES] = {
    linearMixer,
    fingerMixer,
    thumbMixer
};


JointAngleController::JointAngleController(ServoManager& servoManager) : sm(servoManager) {
}

void JointAngleController::start() {
    // Are we connected to a left hand?
    if (sm.getServo(201)) {
        // Add 100 to all servo IDs for left hand
        for (auto& mm : mechanicalMappings) {
            for (auto& id : mm.servoIDs) {
                id += 100;
            }
        }
        rightHand = false;
    }
}

JointAngleController::JointID JointAngleController::getJointID(const std::string& jointName) const {
    string jointNameLower = jointName;
    transform(jointNameLower.begin(), jointNameLower.end(), jointNameLower.begin(), ::tolower);
    auto it = find(jointNames.begin(), jointNames.end(), jointNameLower);
    if (it != jointNames.end()){
        return (JointID)(it - jointNames.begin());
    }
    return NUM_JOINT_IDS;
}

void JointAngleController::setJointRange(JointID jointID, float minAngle, float maxAngle) {
    if (jointID >= NUM_JOINT_IDS) return;   // Out of range

    // Range sanity check
    if (minAngle < 0.0) minAngle = 0.0;
    if (maxAngle > 180.0) maxAngle = 180.0;

    mechanicalMappings[jointID].minAngle = minAngle;
    mechanicalMappings[jointID].maxAngle = maxAngle;
}

void JointAngleController::setJointAngle(JointID jointID, float angle) {
    if (jointID >= NUM_JOINT_IDS) return;   // Out of range

    // Clamp to range
    angle = clamp(angle, mechanicalMappings[jointID]);

    // Set angle
    jointAngles[jointID] = angle;

    // Call mixer for this joint
    mixers[mechanicalMappings[jointID].mixer](jointID, mechanicalMappings, jointAngles, sm);
} 


} // namespace dexhand_connect