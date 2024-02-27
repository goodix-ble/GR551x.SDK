/**
 ****************************************************************************************
 *
 * @file scatter_config.h
 *
 * @brief Common scatter file definition file.
 *
 *
 ****************************************************************************************
 */

#ifndef __SCATTER_CONFIG_H__
#define __SCATTER_CONFIG_H__

#include "custom_config.h"

/*****************************************************************
 * if SYSTEM_STACK_SIZE is not defined in custom_config.h, 
 * keep default setting to 32KB
 */
#ifndef SYSTEM_STACK_SIZE
    #define SYSTEM_STACK_SIZE    0x8000
#endif

#define FLASH_START_ADDR         0x01000000
#define FLASH_SIZE               0x00800000

/* size of ROM reserved RAM in retention cell */
#ifndef ROM_RTN_RAM_SIZE
#define ROM_RTN_RAM_SIZE        0x4100
#endif

#define RAM_ALIAS

/*****************************************************************
 * Warning: User App developer never change the six macros below
 */
#ifdef RAM_ALIAS
#define RAM_START_ADDR          0x00800000
#else
#define RAM_START_ADDR          0x30000000
#endif

#if (CHIP_TYPE == 6) || (CHIP_TYPE == 7) //GR5513
    #define RAM_SIZE            0x00020000
#else //GR5515
    #define RAM_SIZE            0x00040000
#endif

#define RAM_END_ADDR            (RAM_START_ADDR + RAM_SIZE)


#define FERP_SIZE               0x8000     //32K
#define CRITICAL_CODE_MAX_SIZE  0x10000    // maximum size of critical code reserved

#if (APP_CODE_RUN_ADDR == APP_CODE_LOAD_ADDR && \
        APP_CODE_RUN_ADDR >= FLASH_START_ADDR && \
        APP_CODE_RUN_ADDR < FLASH_START_ADDR + FLASH_SIZE)
    #define XIP_MODE    
#endif
/****************************************************************/

/**************************************************************************/
/* sections on retention RAM cells */
#ifdef CFG_FERP
    #define STACK_END_ADDR         (RAM_END_ADDR-FERP_SIZE)
#else
    #define STACK_END_ADDR         (RAM_END_ADDR)
#endif

#define USE_TINY_RAM_SPACE

#define TINY_RAM_SPACE_START    (0x30000000 + 0x35CC)     /* DONT MODIFY ME !!! */
#define TINY_RAM_SPACE_SIZE     (0x750)                   /* DONT MODIFY ME !!! */

#define FPB_SECTION_START       0x30004000
#define FPB_SECTION_SIZE        0x100

#define RAM_RESERVE_SECTION_SIZE    0x64

// Code size of Application
#ifndef APP_MAX_CODE_SIZE
#define APP_MAX_CODE_SIZE       0x00800000
#endif

// RAM size of Application
#ifndef APP_RAM_SIZE
#define APP_RAM_SIZE            0x00030000
#endif

#endif // __SCATTER_CONFIG_H__

