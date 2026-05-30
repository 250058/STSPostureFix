/**
 * STSPostureFix.h
 * Posture-aware position error compensation for STS3215 servos.
 *
 * === Quick Start ===
 *
 *   STSServoDriver servos;
 *   STSPostureFix  fix(&servos);
 *
 *   // Step 1: describe your arm's current posture
 *   fix.setJointOrientation(2, HORIZONTAL);
 *   fix.setJointOrientation(3, VERTICAL);
 *   fix.setJointOrientation(4, HORIZONTAL);
 *
 *   // Step 2: (optional) tune gain  0.0 = no correction  1.0 = full
 *   fix.setGain(0.5f);
 *
 *   // Step 3: send a compensated command
 *   fix.setTargetPositionCompensated(3, 1000);
 *
 * === Posture matching ===
 *   Number of VERTICAL joints → matched posture
 *   0  →  POSE_EXTENDED    (all horizontal)
 *   1  →  POSE_HALF_FOLDED (one vertical)
 *   2+ →  POSE_FOLDED      (two or more vertical)
 *
 * License: MIT
 */

#ifndef STS_POSTURE_FIX_H
#define STS_POSTURE_FIX_H

#include <Arduino.h>
#include "STSServoDriver.h"

// ---------------------------------------------------------------------------
// Public enums
// ---------------------------------------------------------------------------
enum JointOrientation {
    UNSET      = 0,
    HORIZONTAL = 1,
    VERTICAL   = 2
};

enum ReferencePosture {
    POSE_EXTENDED    = 0,
    POSE_HALF_FOLDED = 1,
    POSE_FOLDED      = 2,
    POSE_UNKNOWN     = -1
};

// ---------------------------------------------------------------------------
// Class
// ---------------------------------------------------------------------------
#define STS_PF_MAX_JOINTS   16
#define STS_PF_DEFAULT_GAIN 0.5f

class STSPostureFix {
public:
    explicit STSPostureFix(STSServoDriver* driver);

    // --- posture setup ---
    void reset();
    void setJointOrientation(byte joint_id, JointOrientation orientation);
    void setReferencePosture(ReferencePosture pose);  // manual override
    ReferencePosture getReferencePosture() const;
    void refreshAutoMatch();
    int  countVerticalJoints() const;

    // --- gain ---
    void  setGain(float gain);   // 0.0 ~ 1.0, default 0.5
    float getGain() const;

    // --- query ---
    int predictError(byte joint_id, int target) const;      // raw prediction (no gain)
    int compensateTarget(byte joint_id, int target) const;  // gain applied

    // --- commands ---
    bool setTargetPositionCompensated(byte joint_id, int target);
    bool setTargetPositionCompensated(byte joint_id, int target, int speed);

    // --- toggle ---
    void enableCompensation(bool enabled);
    bool isCompensationEnabled() const;

private:
    STSServoDriver*  driver_;
    JointOrientation orientations_[STS_PF_MAX_JOINTS + 1];
    ReferencePosture active_posture_;
    bool             manual_override_;
    bool             compensation_enabled_;
    float            gain_;

    ReferencePosture autoMatchPosture() const;
};

#endif // STS_POSTURE_FIX_H
