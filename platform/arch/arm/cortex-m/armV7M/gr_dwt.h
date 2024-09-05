
#ifndef GR_DWT_H
#define GR_DWT_H

#include "gr5x.h"


/*
 
    support for ARMv7
  
*/
#define HAL_TIMEOUT_INIT()                                               \
    uint32_t _demcr_initial = CoreDebug->DEMCR;                          \
    uint32_t _dwt_ctrl_initial = DWT->CTRL;                              \
do {                                                                     \
    hal_dwt_enable(_demcr_initial, _dwt_ctrl_initial);                   \
} while (0)

#define HAL_TIMEOUT_DEINIT()                                             \
do {                                                                     \
    hal_dwt_disable(_demcr_initial, _dwt_ctrl_initial);                  \
} while(0)

/**
 ****************************************************************************************
 * @brief  This function enable the DWT function
 *
 * @return none
 ****************************************************************************************
 */
void hal_dwt_enable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial);

/**
 ****************************************************************************************
 * @brief  This function disable the DWT function
 *
 * @return none
 ****************************************************************************************
 */
void hal_dwt_disable(uint32_t _demcr_initial, uint32_t _dwt_ctrl_initial);


#endif
