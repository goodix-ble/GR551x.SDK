/**
 ******************************************************************************
 *
 * @file patch.h
 *
 * @brief offer the interface for the patch function based on the FPB of the cortex arm-m4;
 *
 ******************************************************************************
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
 
#ifndef __PATCH_H_
#define __PATCH_H_

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
enum
{
    BIT_HCI_SEND_2_CONTROLLER,
    BIT_HAL_XQSPI_SET_XIP_PRESENT_STATUS,
    BIT_LL_XQSPI_INIT,
    BIT_HAL_EXFLASH_WRITE,
    BIT_HAL_EXFLASH_ERASE,
    BIT_HAL_EXFLASH_READ,
    BIT_BLE_BM_SET_SEC_INFO,
    BIT_KE_TASK_SCHEDULE,
};


/*
 * MACRO DECLARATIONS
 ****************************************************************************************
 */
#define PATCH_ENABLE_FLAG(BIT)      (1<<BIT)


#define ENCRYPT_PATCH               (PATCH_ENABLE_FLAG(BIT_HCI_SEND_2_CONTROLLER) |\
                                     PATCH_ENABLE_FLAG(BIT_HAL_EXFLASH_WRITE)     |\
                                     PATCH_ENABLE_FLAG(BIT_HAL_EXFLASH_ERASE)     |\
                                     PATCH_ENABLE_FLAG(BIT_HAL_EXFLASH_READ)      |\
                                     PATCH_ENABLE_FLAG(BIT_BLE_BM_SET_SEC_INFO))  |\
                                     PATCH_ENABLE_FLAG(BIT_KE_TASK_SCHEDULE)


#define NO_ENCRYPT_PATCH            (PATCH_ENABLE_FLAG(BIT_HCI_SEND_2_CONTROLLER) |\
                                     PATCH_ENABLE_FLAG(BIT_HAL_EXFLASH_WRITE)     |\
                                     PATCH_ENABLE_FLAG(BIT_HAL_EXFLASH_ERASE)     |\
                                     PATCH_ENABLE_FLAG(BIT_BLE_BM_SET_SEC_INFO))  |\
                                     PATCH_ENABLE_FLAG(BIT_KE_TASK_SCHEDULE)

/**
 *****************************************************************************************
 * @brief  Apply Cold Patch
 *         This function is used for cold patch
 *****************************************************************************************
 */
extern void cold_patch_apply(void);

/**
 *****************************************************************************************
 * @brief  Patch Enabling Function 
 *         This function can not be used directly. It needs to be registered with FPB unit 
 *         and automatically enabled by the system.
 *****************************************************************************************
 */
extern void fpb_patch_enable(void);

/**
 *****************************************************************************************
 * @brief  Patch Enabling Function for encrypt
 *         This function can not be used directly. It needs to be registered with FPB unit 
 *         and automatically enabled by the system.
 *****************************************************************************************
 */
extern void fpb_encrypt_mode_patch_enable(void);

#endif  // __PATCH_H_

