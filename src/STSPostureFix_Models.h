/**
 * STSPostureFix_Models.h
 * Pre-trained polynomial regression coefficients.
 * Trained on 166 clean measurements (v1.2).
 *
 * Model:  error(t) = intercept + c1*t + c2*t^2
 *
 * Coverage:
 *   POSE_EXTENDED    -> J2, J3, J4
 *   POSE_HALF_FOLDED -> J3, J4
 *   POSE_FOLDED      -> J4
 *
 * R² per model:
 *   A-J2: 0.311   A-J3: 0.584   A-J4: 0.869
 *   B-J3: 0.266   B-J4: 0.781
 *   C-J4: 0.665
 */

#ifndef STS_POSTURE_FIX_MODELS_H
#define STS_POSTURE_FIX_MODELS_H

#include "STSPostureFix.h"

struct PostureModel {
    ReferencePosture posture;
    byte             joint;
    float            intercept;
    float            c1;
    float            c2;
};

const PostureModel STS_PF_MODELS[] = {
    // posture            joint   intercept         c1                  c2
    { POSE_EXTENDED,      2,      297.611000f,     -0.175113170f,       0.000024791696f },
    { POSE_EXTENDED,      3,      -15.397800f,      0.067367470f,       0.000204086759f },
    { POSE_EXTENDED,      4,       89.289400f,     -0.086188700f,       0.000019935582f },
    { POSE_HALF_FOLDED,   3,       -2.235400f,     -0.010636890f,       0.000007563817f },
    { POSE_HALF_FOLDED,   4,      120.509400f,     -0.098849370f,       0.000018859533f },
    { POSE_FOLDED,        4,       98.622800f,     -0.079811000f,       0.000015360242f },
};
const int STS_PF_N_MODELS = sizeof(STS_PF_MODELS) / sizeof(STS_PF_MODELS[0]);

#endif // STS_POSTURE_FIX_MODELS_H
