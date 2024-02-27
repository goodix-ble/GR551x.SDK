/**
 ******************************************************************************
 *
 * @file gr55xx_fpb.h
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

 /**
 * @addtogroup SYSTEM
 * @{
 */
 
/**
 * @addtogroup FPB
 * @{
 * @brief Definitions and prototypes for FPB interface.
 */

#ifndef __GR55XX_FPB_H_
#define __GR55XX_FPB_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/**
 *@addtogroup GR55XX_FPB_ENUMERATIONS Enumerations
 * @{
 */
/**@brief FPB mode. */
typedef enum
{
    FPB_MODE_PATCH_ONLY = 0,                /**< FPB MODE ENABLE FOR PATCH ONLY*/
    FPB_MODE_DEBUG_ONLY,                    /**< FPB MODE ENABLE FOR DEBUG ONLY*/
    FPB_MODE_PATCH_AND_DEBUG,               /**< FPB MODE ENABLE FOR PATCH AND DEBUG*/
} fpb_mode_t ;

/**@brief FPB state. */
typedef enum
{
    FPB_PATCH_OFF = 0,                      /**< FPB patch disable */
    FPB_PATCH_ON,                           /**< FPB patch enable  */
} fpb_state_t;

/**@brief FPB register. */
typedef struct
{
    volatile uint32_t CTRL;                 /**< Offset: 0x000 (R/W)  Data */ 
    volatile uint32_t REMAP;                /**< Offset: 0x004 (R/W)  Data */
    volatile uint32_t COMP[8];              /**< Offset: 0x008  (R)   Data */
} FPB_REG_TypeDef;
/** @} */

/** @addtogroup GR55XX_FPB_DEFINES Defines
 * @{
 */
#define FPB  ((FPB_REG_TypeDef *)  0xE0002000UL) /**< FPB Register Address. */
/** @} */


/**
 * @defgroup GR55XX_FPB_TYPEDEF Typedefs
 * @{
 */
/**@brief FPB function.*/
typedef void(*fun_t)(void);
/** @} */

/**
 * @defgroup GR55XX_FPB_FUNCTION Functions
 * @{
 */
 /**
 ****************************************************************************************
 * @brief  Enabling patch function
 * @param[in] index_start :  Start Index Number
 * @param[in] index_end   :  End Index Number
 ****************************************************************************************
 */
void fpb_enable(uint8_t index_start ,uint8_t index_end);

/**
 ****************************************************************************************
 * @brief  Replace old and new functions
 * @param[in] ori_func : primitive function address
 * @param[in] rep_func : replacement function address
 * @param[in] patch_table_num : group number
 * @return -1: Error Otherwise: Patch table number
 ****************************************************************************************
 */
int fun_replace_by_svc(uint32_t ori_func, uint32_t rep_func, uint8_t patch_table_num);

 /**
 ****************************************************************************************
 * @brief  SVC handler process function
 * @return 0: Function not found in SVc table, Otherwise: Function address for hardware patching
 ****************************************************************************************
 */
uint32_t SVC_handler_proc(uint32_t *svc_args);
    
 /**
 ****************************************************************************************
 * @brief  Register FPB patch enable function
 * @param[in] patch_enable_func : pointer of function
 ****************************************************************************************
 */
void fpb_register_patch_init_func(fun_t patch_enable_func);

 /**
 ****************************************************************************************
 * @brief  FPB init function
 * @param[in] fpb_mode : the mode of FPB
 ****************************************************************************************
 */
void fpb_init(fpb_mode_t fpb_mode);

 /**
 ****************************************************************************************
 * @brief  svc sub-function register
 * @param[in] svc_num : the number of svc 
 * @param[in] func : sub-function callback
 ****************************************************************************************
 */
void svc_func_register(uint8_t svc_num, uint32_t func);

 /**
 ****************************************************************************************
 * @brief  register sve table function
 * @param[in] p_svc_table : the pointer of sve table
 ****************************************************************************************
 */
void svc_table_register(uint32_t *p_svc_table);

/**
 ****************************************************************************************
 * @brief  register fpb space from user layer
 * @param[in] user_fpb_space : the pointer of fpb user space
 ****************************************************************************************
 */
void fpb_register_user_space(uint32_t *user_fpb_space);

 /**
 ****************************************************************************************
 * @brief  save the FPB state
 *
 * @returns FPB state ::fpb_state_t
 ****************************************************************************************
 */
fpb_state_t fpb_save_state(void);

 /**
 ****************************************************************************************
 * @brief load the FPB state
 * @param[in] state : the FPB state that needs to be loaded
 ****************************************************************************************
 */
void fpb_load_state(fpb_state_t state);

/** @} */
#endif
/** @} */
/** @} */

