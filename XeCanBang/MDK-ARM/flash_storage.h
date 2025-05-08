#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "pid.h"

void SavePIDToFlash(PID_t *pid);
void LoadPIDFromFlash(PID_t *pid);

#endif
