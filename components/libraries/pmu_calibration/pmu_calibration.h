#ifndef _PMU_CALIB_H
#define _PMU_CALIB_H

#include <stdio.h>
#include <stdint.h>

void system_pmu_calibration_stop(void);

void system_pmu_calibration_init(uint32_t interval);

#endif /* _PMU_CALIB_H */
