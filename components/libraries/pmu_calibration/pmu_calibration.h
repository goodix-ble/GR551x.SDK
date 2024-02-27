#ifndef _PMU_CALIB_H
#define _PMU_CALIB_H

#include <stdio.h>
#include <stdint.h>

#ifdef ENV_USE_FREERTOS
#include "FreeRTOS.h"
#include "timers.h"
#else
#include "app_timer.h"
#endif

void system_pmu_calibration_stop(void);

void system_pmu_calibration_init(uint32_t interval);

#ifdef ENV_USE_FREERTOS
void system_pmu_calibration_start(void);
#endif

#endif
