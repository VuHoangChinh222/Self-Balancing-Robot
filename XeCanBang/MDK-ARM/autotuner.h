#ifndef AUTOTUNER_H
#define AUTOTUNER_H

#include "pid.h"

typedef struct {
    float errorSum;
    float lastError;
} AutoTuner_t;

void AutoTunePID(PID_t *pid, AutoTuner_t *tuner, float pitchError);

#endif
