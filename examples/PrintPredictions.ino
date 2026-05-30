/**
 * PrintPredictions.ino
 * Show what correction the library would apply across the range,
 * for each posture. No servo movement.
 */

#include <Arduino.h>
#include "STSServoDriver.h"
#include "STSPostureFix.h"

#define RXD 18
#define TXD 17

STSServoDriver servos;
STSPostureFix  fix(&servos);

void printTable(const char* name, float gain) {
    fix.setGain(gain);
    Serial.printf("\n# === %s (gain=%.2f) ===\n", name, gain);
    Serial.printf("posture,gain,joint,target,raw_err,scaled_correction,adjusted\n");
    for (byte j = 2; j <= 4; j++) {
        for (int t = 0; t <= 4095; t += 500) {
            int raw = fix.predictError(j, t);
            int adj = fix.compensateTarget(j, t);
            Serial.printf("%s,%.2f,%d,%d,%d,%d,%d\n",
                          name, gain, j, t, raw, adj-t, adj);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, RXD, TXD);
    delay(1000);
    servos.init(&Serial1);

    // POSE_EXTENDED
    fix.reset();
    fix.setJointOrientation(2, HORIZONTAL);
    fix.setJointOrientation(3, HORIZONTAL);
    fix.setJointOrientation(4, HORIZONTAL);
    printTable("EXTENDED", 0.5f);

    // POSE_HALF_FOLDED
    fix.reset();
    fix.setJointOrientation(2, HORIZONTAL);
    fix.setJointOrientation(3, VERTICAL);
    fix.setJointOrientation(4, HORIZONTAL);
    printTable("HALF_FOLDED", 0.5f);

    // POSE_FOLDED
    fix.reset();
    fix.setJointOrientation(2, VERTICAL);
    fix.setJointOrientation(3, VERTICAL);
    fix.setJointOrientation(4, HORIZONTAL);
    printTable("FOLDED", 0.5f);

    Serial.println("\n# Done.");
}

void loop() {}
