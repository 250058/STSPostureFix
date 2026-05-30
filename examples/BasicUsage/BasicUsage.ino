/**
 * BasicUsage.ino
 * Minimal example of STSPostureFix v1.2.
 *
 * Pick the diagram below that matches the SHAPE of your arm right now,
 * and copy the matching setJointOrientation() block into setup().
 *
 *  =================================================================
 *  [A] EXTENDED  -- J1-J2 vertical, then horizontal from J2
 *
 *        J2---J3---J4---J5---J6
 *         |
 *        J1
 *
 *      fix.setJointOrientation(2, HORIZONTAL);
 *      fix.setJointOrientation(3, HORIZONTAL);
 *      fix.setJointOrientation(4, HORIZONTAL);
 *
 *  =================================================================
 *  [B] HALF-FOLDED  -- J1-J2-J3 vertical, then horizontal from J3
 *
 *        J3---J4---J5---J6
 *         |
 *        J2
 *         |
 *        J1
 *
 *      fix.setJointOrientation(2, HORIZONTAL);
 *      fix.setJointOrientation(3, VERTICAL);
 *      fix.setJointOrientation(4, HORIZONTAL);
 *
 *  =================================================================
 *  [C] FOLDED  -- J1-J2-J3-J4 vertical, then horizontal from J4
 *
 *        J4---J5---J6
 *         |
 *        J3
 *         |
 *        J2
 *         |
 *        J1
 *
 *      fix.setJointOrientation(2, VERTICAL);
 *      fix.setJointOrientation(3, VERTICAL);
 *      fix.setJointOrientation(4, HORIZONTAL);
 *  =================================================================
 */

#include <Arduino.h>
#include "STSServoDriver.h"
#include "STSPostureFix.h"

#define RXD 18
#define TXD 17

STSServoDriver servos;
STSPostureFix  fix(&servos);

void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, RXD, TXD);
    delay(1000);

    servos.init(&Serial1);
    servos.setMode(0xFE, STSMode::POSITION);

    // ---- describe your posture here (pick A / B / C above) ----
    fix.setJointOrientation(2, HORIZONTAL);
    fix.setJointOrientation(3, VERTICAL);
    fix.setJointOrientation(4, HORIZONTAL);
    // -----------------------------------------------------------

    fix.setGain(0.5f);

    Serial.printf("Posture matched: %d, gain=%.2f\n",
                  (int)fix.getReferencePosture(), fix.getGain());

    int target = 735;
    Serial.printf("Joint 3: requested=%d, predicted_err=%d, adjusted=%d\n",
                  target,
                  fix.predictError(3, target),
                  fix.compensateTarget(3, target));

    fix.setTargetPositionCompensated(3, target);
}

void loop() {}
