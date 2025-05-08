#include "autotuner.h"
#include <math.h>
#include "stm32f1xx_hal.h"
#include "flash_storage.h"

void AutoTunePID(PID_t *pid, AutoTuner_t *tuner, float pitchError) {
    static uint32_t lastTuningTime = 0;
    uint32_t now = HAL_GetTick();
    static PID_t lastSavedPID = {0};

    if (now - lastTuningTime < 500) return;

    tuner->errorSum += fabs(pitchError);
    float errorDelta = fabs(pitchError - tuner->lastError);

    if (tuner->errorSum > 30.0 || errorDelta > 15.0) {
        pid->Kp *= 0.9;
    } else if (tuner->errorSum < 5.0 && errorDelta < 3.0) {
        pid->Kp *= 1.05;
    }

    tuner->lastError = pitchError;
    tuner->errorSum *= 0.8;

    if (fabs(pid->Kp - lastSavedPID.Kp) > (lastSavedPID.Kp * 0.1) || 
        fabs(pid->Ki - lastSavedPID.Ki) > (lastSavedPID.Ki * 0.1) ||
				fabs(pid->Kd - lastSavedPID.Kd) > (lastSavedPID.Kd * 0.1)){ 
        SavePIDToFlash(pid);
        lastSavedPID = *pid;  // Luu lai de so sánh lan sau
    }

    lastTuningTime = now;
}
