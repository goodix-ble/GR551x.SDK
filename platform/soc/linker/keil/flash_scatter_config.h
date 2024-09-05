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

#define FLASH_START_ADDR         0x01000000
#define FLASH_SIZE               0x00800000

#define RAM_START_ADDR            0x00800000
#define HIGH_RAM_OFFSET           0x2F800000
#define FPB_DATA_SPACE_SIZE       0x100
#define RAM_CODE_SPACE_SIZE       (0x5000 - FPB_DATA_SPACE_SIZE)

/* size of ROM reserved RAM in retention cell */
#ifndef ROM_RTN_RAM_SIZE
#define ROM_RTN_RAM_SIZE        0x4100
#endif

#define RAM_CODE_SPACE_START      (RAM_START_ADDR + ROM_RTN_RAM_SIZE)
#define FPB_DATA_SPACE_START      (RAM_START_ADDR + ROM_RTN_RAM_SIZE + RAM_CODE_SPACE_SIZE + HIGH_RAM_OFFSET)


#if (CHIP_TYPE == 6) || (CHIP_TYPE == 7) //GR5513
    #define RAM_SIZE            0x00020000
#else //GR5515
    #define RAM_SIZE            0x00040000
#endif

#define RAM_END_ADDR            (RAM_START_ADDR + HIGH_RAM_OFFSET + RAM_SIZE)

#define FERP_SIZE               0x8000     //32K
#define CRITICAL_CODE_MAX_SIZE  0x10000    // maximum size of critical code reserved

#if (APP_CODE_RUN_ADDR == APP_CODE_LOAD_ADDR && \
        APP_CODE_RUN_ADDR >= FLASH_START_ADDR && \
        APP_CODE_RUN_ADDR < FLASH_START_ADDR + FLASH_SIZE)
    #define XIP_MODE    
#endif

#if ((APP_CODE_RUN_ADDR > (RAM_START_ADDR + HIGH_RAM_OFFSET)) && \
        (APP_CODE_RUN_ADDR < (RAM_START_ADDR + HIGH_RAM_OFFSET + RAM_SIZE)))
    #define HMIRROR_MODE
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

// Code size of Application
#ifndef APP_MAX_CODE_SIZE
#define APP_MAX_CODE_SIZE       0x00800000
#endif

// RAM size of Application
#ifndef APP_RAM_SIZE
#define APP_RAM_SIZE            0x00030000
#endif

#endif // __SCATTER_CONFIG_H__

