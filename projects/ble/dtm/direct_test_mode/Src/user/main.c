/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "grx_sys.h"
#include "user_app.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "patch.h"
#include "hci_uart.h"

/*
 * MACRO VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#define FCC_ENABLE                 (0)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

extern void ble_sdk_patch_env_init(void);
extern void ble_test_evn_init(void);

int main (void)
{
    pwr_mgmt_mode_set(PMR_MGMT_IDLE_MODE);
    ble_hci_uart_init();

    #if(defined SOC_GR533X)
    ble_rf_tx_mode_set((ble_rf_tx_mode_t)RF_TX_PA_SELECT);
    #endif

    #if (defined SOC_GR5X25) || (defined SOC_GR533X)
    ble_sdk_patch_env_init();
    ble_test_evn_init();
    ble_stack_controller_init(&heaps_table);
    #else
    ble_stack_init(ble_evt_handler, &heaps_table);
    #endif

    //loop
    while(1)
    {
        pwr_mgmt_schedule();
    }
}

