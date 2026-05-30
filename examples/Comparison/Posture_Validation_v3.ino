// Posture Validation v3
// =====================================================
// Measures OFF (5 trials) then ON (5 trials) for each
// (pose, joint, target) combination.
//
// On startup you select which pose to run by looking at
// ASCII diagrams that show the arm shape.
//
// CSV output: mode,pose,joint,trial,target,actual,error

#include <Arduino.h>
#include "STSServoDriver.h"
#include <STSPostureFix.h>

#define RXD 18
#define TXD 17

STSServoDriver servos;
STSPostureFix  fix(&servos);

// ===== Fixed joints =====
const int J1_FIXED = 2047;
const int J5_FIXED = 2348;
const int J6_FIXED = 0;

// ===== Pose fixed values =====
const int A_J2_FIX = 4095;  const int A_J3_FIX = 0;    const int A_J4_FIX = 2866;
const int B_J2_FIX = 3200;  const int B_J3_FIX = 1024; const int B_J4_FIX = 2866;
const int C_J2_FIX = 3200;  const int C_J3_FIX = 0;

// ===== Sweep =====
struct Sweep { int joint; int lo; int hi; };

const Sweep A_SWEEPS[] = {{ 2, 3200, 4095 }, { 3, 0, 180 }, { 4, 1800, 3200 }};
const int A_N = 3;
const Sweep B_SWEEPS[] = {{ 3, 0, 1470 }, { 4, 1800, 4095 }};
const int B_N = 2;
const Sweep C_SWEEPS[] = {{ 4, 1800, 4095 }};
const int C_N = 1;

const int SWEEP_POINTS = 6;
const int SWEEP_REPEAT = 5;   // 5 trials per mode
const unsigned long SETTLE_MS = 2500;
const int SAMPLES = 10;
const int SAMPLE_INTERVAL_MS = 80;
const int MOVE_SPEED = 300;
const float COMP_GAIN = 0.5f;


// ===== Helpers =====
int safeRead(byte id) {
    for (int i = 0; i < 3; i++) {
        int p = servos.getCurrentPosition(id);
        if (p >= 0 && p < 4096) return p;
        delay(20);
    }
    return -1;
}

void waitSettle(byte id) {
    delay(100);
    unsigned long t = millis();
    while (servos.isMoving(id)) {
        if (millis() - t > 5000) break;
        delay(50);
    }
    delay(SETTLE_MS);
}

void moveDirect(byte id, int target) {
    servos.setTargetPosition(id, target, MOVE_SPEED);
    delay(15);
    servos.setTargetPosition(id, target, MOVE_SPEED);
    waitSettle(id);
}

void moveComp(byte id, int target) {
    fix.setTargetPositionCompensated(id, target, MOVE_SPEED);
    delay(15);
    fix.setTargetPositionCompensated(id, target, MOVE_SPEED);
    waitSettle(id);
}

void moveTarget(byte id, int target, bool comp) {
    if (comp) moveComp(id, target);
    else      moveDirect(id, target);
}

int measureMean(byte id) {
    long s = 0; int v = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int p = safeRead(id);
        if (p >= 0) { s += p; v++; }
        delay(SAMPLE_INTERVAL_MS);
    }
    return v > 0 ? (int)(s / v) : -1;
}

void commonFixed() {
    servos.setTargetPosition(1, J1_FIXED, MOVE_SPEED);
    servos.setTargetPosition(5, J5_FIXED, MOVE_SPEED);
    servos.setTargetPosition(6, J6_FIXED, MOVE_SPEED);
}

void poseA(byte sw) {
    commonFixed();
    if (sw!=2) servos.setTargetPosition(2, A_J2_FIX, MOVE_SPEED);
    if (sw!=3) servos.setTargetPosition(3, A_J3_FIX, MOVE_SPEED);
    if (sw!=4) servos.setTargetPosition(4, A_J4_FIX, MOVE_SPEED);
    delay(2500);
}
void poseB(byte sw) {
    commonFixed();
    servos.setTargetPosition(2, B_J2_FIX, MOVE_SPEED);
    if (sw!=3) servos.setTargetPosition(3, B_J3_FIX, MOVE_SPEED);
    if (sw!=4) servos.setTargetPosition(4, B_J4_FIX, MOVE_SPEED);
    delay(2500);
}
void poseC(byte sw) {
    commonFixed();
    servos.setTargetPosition(2, C_J2_FIX, MOVE_SPEED);
    servos.setTargetPosition(3, C_J3_FIX, MOVE_SPEED);
    delay(2500);
}

void fixA() { fix.reset(); fix.setJointOrientation(2,HORIZONTAL); fix.setJointOrientation(3,HORIZONTAL); fix.setJointOrientation(4,HORIZONTAL); }
void fixB() { fix.reset(); fix.setJointOrientation(2,HORIZONTAL); fix.setJointOrientation(3,VERTICAL);   fix.setJointOrientation(4,HORIZONTAL); }
void fixC() { fix.reset(); fix.setJointOrientation(2,VERTICAL);   fix.setJointOrientation(3,VERTICAL);   fix.setJointOrientation(4,HORIZONTAL); }

void logRow(const char* mode, const char* pose, byte joint, int trial, int target, int actual) {
    Serial.printf("%s,%s,%d,%d,%d,%d,%d\n\r", mode, pose, joint, trial, target, actual, target-actual);
}

// ===== One sweep =====
void runSweep(const char* mode, const char* pose, byte joint, int lo, int hi,
              void(*applyFn)(byte), bool comp) {
    Serial.printf("# --- mode=%s Pose %s Joint %d range %d..%d ---\n\r",
                  mode, pose, joint, lo, hi);
    applyFn(joint);
    int home = (lo + hi) / 2;
    for (int idx = 0; idx < SWEEP_POINTS; idx++) {
        int target = lo + (int)(((long)(hi-lo)*idx) / (SWEEP_POINTS-1));
        for (int trial = 0; trial < SWEEP_REPEAT; trial++) {
            applyFn(joint);
            moveDirect(joint, home);
            moveTarget(joint, target, comp);
            int actual = measureMean(joint);
            logRow(mode, pose, joint, trial, target, actual);
        }
    }
}

// ===== One pose (OFF then ON) =====
void validatePose(const char* label,
                  const Sweep* sweeps, int n,
                  void(*applyFn)(byte), void(*cfgFix)()) {
    Serial.printf("\n\r# ===================================\n\r");
    Serial.printf("# Pose %s : OFF %d trials then ON %d trials\n\r",
                  label, SWEEP_REPEAT, SWEEP_REPEAT);
    Serial.printf("# ===================================\n\r");
    cfgFix();
    Serial.printf("# matched posture id=%d gain=%.2f\n\r",
                  (int)fix.getReferencePosture(), fix.getGain());

    fix.enableCompensation(false);
    for (int i = 0; i < n; i++)
        runSweep("OFF", label, sweeps[i].joint, sweeps[i].lo, sweeps[i].hi, applyFn, false);

    fix.enableCompensation(true);
    for (int i = 0; i < n; i++)
        runSweep("ON", label, sweeps[i].joint, sweeps[i].lo, sweeps[i].hi, applyFn, true);

    Serial.printf("# Pose %s done\n\r", label);
}

void runA() { validatePose("A", A_SWEEPS, A_N, poseA, fixA); }
void runB() { validatePose("B", B_SWEEPS, B_N, poseB, fixB); }
void runC() { validatePose("C", C_SWEEPS, C_N, poseC, fixC); }

// ===== ASCII arm diagrams =====
void showPostureDiagrams() {
    Serial.printf("\n\r");
    Serial.printf("# Select the posture that matches your current arm shape.\n\r");
    Serial.printf("# --------------------------------------------------------\n\r");
    Serial.printf("#\n\r");
    Serial.printf("#  [A] EXTENDED  (0 vertical joints, arm fully stretched)\n\r");
    Serial.printf("#\n\r");
    Serial.printf("#         J4---J5\n\r");
    Serial.printf("#        /\n\r");
    Serial.printf("#       J3\n\r");
    Serial.printf("#      /\n\r");
    Serial.printf("#     J2           <- J2 horizontal\n\r");
    Serial.printf("#     |\n\r");
    Serial.printf("#    J1 (base)\n\r");
    Serial.printf("#\n\r");
    Serial.printf("# --------------------------------------------------------\n\r");
    Serial.printf("#\n\r");
    Serial.printf("#  [B] HALF-FOLDED  (1 vertical joint, elbow up)\n\r");
    Serial.printf("#\n\r");
    Serial.printf("#    J3---J4---J5\n\r");
    Serial.printf("#    |              <- J3 vertical\n\r");
    Serial.printf("#    J2             <- J2 slightly raised\n\r");
    Serial.printf("#    |\n\r");
    Serial.printf("#   J1 (base)\n\r");
    Serial.printf("#\n\r");
    Serial.printf("# --------------------------------------------------------\n\r");
    Serial.printf("#\n\r");
    Serial.printf("#  [C] FOLDED  (2 vertical joints, arm upright)\n\r");
    Serial.printf("#\n\r");
    Serial.printf("#    J4---J5\n\r");
    Serial.printf("#    |              <- J4 vertical\n\r");
    Serial.printf("#    J3             <- J3 vertical\n\r");
    Serial.printf("#    |              <- J2 raised\n\r");
    Serial.printf("#    J2\n\r");
    Serial.printf("#    |\n\r");
    Serial.printf("#   J1 (base)\n\r");
    Serial.printf("#\n\r");
    Serial.printf("# --------------------------------------------------------\n\r");
    Serial.printf("# Enter  A / B / C  to start that pose.\n\r");
    Serial.printf("# Enter  0  to run ALL poses (A then B then C).\n\r");
    Serial.printf("# Enter  p  to ping all servos.\n\r");
    Serial.printf("# Enter  ?  to show this diagram again.\n\r");
    Serial.printf("# --------------------------------------------------------\n\r");
    Serial.printf("# > ");
}

// ===== Setup / Loop =====
void setup() {
    Serial.begin(115200);
    Serial1.begin(1000000, SERIAL_8N1, RXD, TXD);
    delay(1000);

    if (Serial1) Serial.printf("Serial1(%d,%d) is ready\n\r", RXD, TXD);
    if (servos.init(&Serial1)) Serial.printf("Driver initialized.\n\r");
    servos.setMode(0xFE, STSMode::POSITION);

    fix.setGain(COMP_GAIN);

    for (byte id = 1; id <= 6; id++) {
        bool ok = servos.ping(id);
        Serial.printf("# Servo %d : %s\n\r", id, ok ? "ping OK" : "no response");
    }

    Serial.printf("# CSV columns: mode,pose,joint,trial,target,actual,error\n\r");
    Serial.printf("mode,pose,joint,trial,target,actual,error\n\r");

    showPostureDiagrams();
}

void loop() {
    if (!Serial.available()) return;
    char c = Serial.read();
    while (Serial.available()) Serial.read();

    switch (c) {
        case 'A': case 'a': runA(); showPostureDiagrams(); break;
        case 'B': case 'b': runB(); showPostureDiagrams(); break;
        case 'C': case 'c': runC(); showPostureDiagrams(); break;
        case '0': runA(); runB(); runC(); showPostureDiagrams(); break;
        case 'p': case 'P':
            for (byte id = 1; id <= 6; id++)
                Serial.printf("# Servo %d : %s pos=%d\n\r", id,
                              servos.ping(id) ? "OK" : "NO",
                              servos.getCurrentPosition(id));
            showPostureDiagrams();
            break;
        case '?': showPostureDiagrams(); break;
        case '\n': case '\r': break;
        default: showPostureDiagrams(); break;
    }
}
