# STSPostureFix v1.2

Posture-aware position error compensation for STS3215 serial servos.

## What it does

STS3215 servos reach slightly different positions depending on the arm posture
(due to gravity load). This library corrects that by applying a polynomial
regression model trained on real measurements.

## Posture selection — by arm shape

Pick the diagram that matches your arm's current shape.

```
[A] EXTENDED  (J1-J2 vertical, then horizontal from J2)

      J2---J3---J4---J5---J6
       |
      J1


[B] HALF-FOLDED  (J1-J2-J3 vertical, then horizontal from J3)

      J3---J4---J5---J6
       |
      J2
       |
      J1


[C] FOLDED  (J1-J2-J3-J4 vertical, then horizontal from J4)

      J4---J5---J6
       |
      J3
       |
      J2
       |
      J1
```

## Quick start

```cpp
#include <STSServoDriver.h>
#include <STSPostureFix.h>

STSServoDriver servos;
STSPostureFix  fix(&servos);

void setup() {
    Serial1.begin(1000000, SERIAL_8N1, 18, 17);
    servos.init(&Serial1);
    servos.setMode(0xFE, STSMode::POSITION);

    // Pose A: arm extended
    fix.setJointOrientation(2, HORIZONTAL);
    fix.setJointOrientation(3, HORIZONTAL);
    fix.setJointOrientation(4, HORIZONTAL);

    fix.setGain(0.5f);  // start here; adjust if over-correction

    fix.setTargetPositionCompensated(3, 1000);
}
```

## API

| Function | Description |
|---|---|
| `setJointOrientation(id, H/V)` | Describe each joint |
| `setGain(0.0..1.0)` | Compensation strength (default 0.5) |
| `setTargetPositionCompensated(id, target)` | Send compensated command |
| `predictError(id, target)` | Raw predicted error (no gain) |
| `compensateTarget(id, target)` | What target will actually be sent |
| `enableCompensation(bool)` | Toggle ON/OFF for A/B testing |

## Validation example

Use `examples/Comparison/Posture_Validation_v3.ino`. It runs OFF 5 trials
then ON 5 trials at each measurement point and prints CSV for analysis.

## Model coverage

| Posture | Joints compensated |
|---|---|
| `POSE_EXTENDED` | J2, J3, J4 |
| `POSE_HALF_FOLDED` | J3, J4 |
| `POSE_FOLDED` | J4 |

Other joints pass through unchanged.

## License

MIT
