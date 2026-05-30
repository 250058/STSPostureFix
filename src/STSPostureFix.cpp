/**
 * STSPostureFix.cpp
 */

#include "STSPostureFix.h"
#include "STSPostureFix_Models.h"

STSPostureFix::STSPostureFix(STSServoDriver* driver)
    : driver_(driver),
      active_posture_(POSE_UNKNOWN),
      manual_override_(false),
      compensation_enabled_(true),
      gain_(STS_PF_DEFAULT_GAIN)
{
    reset();
}

void STSPostureFix::reset() {
    for (int i = 0; i <= STS_PF_MAX_JOINTS; i++) orientations_[i] = UNSET;
    active_posture_ = POSE_UNKNOWN;
    manual_override_ = false;
}

void STSPostureFix::setJointOrientation(byte joint_id, JointOrientation o) {
    if (joint_id < 1 || joint_id > STS_PF_MAX_JOINTS) return;
    orientations_[joint_id] = o;
    if (!manual_override_) active_posture_ = autoMatchPosture();
}

void STSPostureFix::setReferencePosture(ReferencePosture pose) {
    active_posture_  = pose;
    manual_override_ = (pose != POSE_UNKNOWN);
}

ReferencePosture STSPostureFix::getReferencePosture() const { return active_posture_; }

int STSPostureFix::countVerticalJoints() const {
    int n = 0;
    for (int i = 1; i <= STS_PF_MAX_JOINTS; i++)
        if (orientations_[i] == VERTICAL) n++;
    return n;
}

ReferencePosture STSPostureFix::autoMatchPosture() const {
    bool any = false;
    for (int i = 1; i <= STS_PF_MAX_JOINTS; i++)
        if (orientations_[i] != UNSET) { any = true; break; }
    if (!any) return POSE_UNKNOWN;
    int v = countVerticalJoints();
    if (v == 0) return POSE_EXTENDED;
    if (v == 1) return POSE_HALF_FOLDED;
    return POSE_FOLDED;
}

void STSPostureFix::refreshAutoMatch() {
    manual_override_ = false;
    active_posture_  = autoMatchPosture();
}

void STSPostureFix::setGain(float g) {
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    gain_ = g;
}
float STSPostureFix::getGain() const { return gain_; }

int STSPostureFix::predictError(byte joint_id, int target) const {
    if (active_posture_ == POSE_UNKNOWN) return 0;
    for (int i = 0; i < STS_PF_N_MODELS; i++) {
        const PostureModel& m = STS_PF_MODELS[i];
        if (m.posture == active_posture_ && m.joint == joint_id) {
            float t = (float)target;
            return (int)(m.intercept + m.c1 * t + m.c2 * t * t);
        }
    }
    return 0;
}

int STSPostureFix::compensateTarget(byte joint_id, int target) const {
    if (!compensation_enabled_) return target;
    return target + (int)(predictError(joint_id, target) * gain_);
}

bool STSPostureFix::setTargetPositionCompensated(byte joint_id, int target) {
    if (!driver_) return false;
    return driver_->setTargetPosition(joint_id, compensateTarget(joint_id, target));
}

bool STSPostureFix::setTargetPositionCompensated(byte joint_id, int target, int speed) {
    if (!driver_) return false;
    return driver_->setTargetPosition(joint_id, compensateTarget(joint_id, target), speed);
}

void STSPostureFix::enableCompensation(bool e) { compensation_enabled_ = e; }
bool STSPostureFix::isCompensationEnabled() const { return compensation_enabled_; }
