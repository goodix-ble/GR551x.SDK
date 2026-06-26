#ifndef IO_PIN_MONITOR_H
#define IO_PIN_MONITOR_H

#include "custom_config.h"

#define GPIO0_MONITOR_ENABLE
#if defined(SOC_GR5405) || defined(SOC_GR533X)
#define GPIO0_PIN_NUM          (14)
#elif defined(SOC_SGP)
#define GPIO0_PIN_NUM          (12)
#else
#define GPIO0_PIN_NUM          (16)
#endif

#if defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR5515) || defined(SOC_SGP)
#define GPIO1_MONITOR_ENABLE
#if defined(SOC_SGP)
#define GPIO1_PIN_NUM          (14)
#else
#define GPIO1_PIN_NUM          (16)
#endif
#endif

#if defined(SOC_GR5X25) || defined(SOC_GR5526)
#define GPIO2_MONITOR_ENABLE
#define GPIO2_PIN_NUM          (2)
#endif

#if !defined(SOC_SGP)
#define MSIO_MONITOR_ENABLE
#if defined(SOC_GR5405) || defined(SOC_GR533X)
#define MSIO_PIN_NUM           (10)
#elif defined(SOC_GR5515)
#define MSIO_PIN_NUM           (5)
#else
#define MSIO_PIN_NUM           (8)
#endif
#endif

#if !defined(SOC_SGP)
#define AON_GPIO_MONITOR_ENABLE
#define AON_GPIO_PIN_NUM       (8)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @brief  Initialize IO pin monitor module.
 * @note   After initialization, io_pin_monitor will automatically collect the status of all IOs before each sleep.
 ****************************************************************************************
 */
void io_pin_monitor_init(void);

/**
 ****************************************************************************************
 * @brief  Print the monitored IO configuration status.
 ****************************************************************************************
 */
void io_pin_monitor_output(void);

/**
 ****************************************************************************************
 * @brief  Print the raw values of the monitored IO configuration status.
 * @note   Description of printed values:
           1. direction state (uint32_t) : From bit 0 to bit 31, every 2 bits represent the direction of one IO PIN; for example, bit 0 - bit 1 represent the direction of GPIO/AON/MSIO PIN0.
               00: none (high impedance).
               01: input.
               02: output.
               03: mux (the pinmux is configured for a peripheral).
           2. pull_state (uint32_t) : From bit 0 to bit 31, every 2 bits represent the pull mode of one IO PIN; for example, bit 0 - bit 1 represent the pull mode of GPIO/AON/MSIO PIN0.
               00: no pull.
               01: pull up.
               02: pull down.
           3. input state (uint16_t) : From bit 0 to bit 15, every 1 bits represent the input data level of one IO PIN; for example, bit 0 represent the data level of GPIO/AON/MSIO PIN0.
                            Only takes effect when the corresponding GPIO pin is in input or inout mode.
               0: low level.
               1: high level.
           4. output state (uint16_t) : From bit 0 to bit 15, every 1 bits represent the output data level of one IO PIN; for example, bit 0 represent the output level of GPIO/AON/MSIO PIN0.
                            Only takes effect when the corresponding GPIO pin is in output or inout mode.
                            For GPIO : The returned value is the configured output level.
                            For MSIO/AON_GPIO : The returned value is the read-back bus level.
               0: low level.
               1: high level.
           5. mode state (uint16_t) : From bit 0 to bit 15, every 1 bits represent the output data level of one IO PIN; for example, bit 0 represent the output level of MSIO PIN0.
                            Only for MSIO.
               0: digital mode.
               1: analog mode.
 ****************************************************************************************
 */
void io_pin_monitor_output_raw(void);

#ifdef __cplusplus
}
#endif

#endif /* IO_PIN_MONITOR_H */
