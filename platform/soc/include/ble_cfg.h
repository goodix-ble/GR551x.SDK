/**
 ****************************************************************************************
 *
 * @file scatter_common.h
 *
 * @brief decalare the symbols in scatter_common.sct.
 *
 *
 ****************************************************************************************
 */

#ifndef __BLE_CFG_H__
#define __BLE_CFG_H__

#include <stdint.h>
#include "custom_config.h"

/* ************************************************************************
 * developer must define CFG_MAX_CONNECTIONS in custom_config.h .
 * Max value for GR551X: 10 which must be same with CFG_CON
 * in ROM's configs.opt
 */
#ifndef CFG_MAX_CONNECTIONS
    #error "CFG_MAX_CONNECTIONS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_CONNECTIONS <= 10)
    #define USER_MAX_CONNECTIONS CFG_MAX_CONNECTIONS
#else
    #define USER_MAX_CONNECTIONS (1)
#endif

#ifndef CFG_MAX_ADVS
    #error "CFG_MAX_ADVS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_ADVS <= 5)
    #define USER_MAX_ADVS CFG_MAX_ADVS
#else
    #define USER_MAX_ADVS (1)
#endif

#ifndef CFG_MAX_PER_ADVS
    #error "CFG_MAX_PER_ADVS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_PER_ADVS <= 5)
    #define USER_MAX_PER_ADVS CFG_MAX_PER_ADVS
#else
    #define USER_MAX_PER_ADVS (0)
#endif

#if ((USER_MAX_ADVS+USER_MAX_PER_ADVS) > 5)
    #error "The number of BLE Legacy/Extended/Periodic Advertising exceeds the limit."
#endif

#ifndef CFG_MAX_SCAN
    #error "CFG_MAX_SCAN is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_SCAN <= 1)
    #define USER_MAX_SCAN CFG_MAX_SCAN
#else
    #define USER_MAX_SCAN (1)
#endif

#ifndef CFG_SCAN_DUP_FILT_LIST_NUM
    #define USER_SCAN_DUP_FILT_NUM   0
#else
    #define USER_SCAN_DUP_FILT_NUM   CFG_SCAN_DUP_FILT_LIST_NUM
#endif

#ifndef CFG_MAX_SYNCS
    #error "CFG_MAX_SYNCS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_SYNCS <= 5)
    #define USER_MAX_SYNCS CFG_MAX_SYNCS
#else
    #define USER_MAX_SYNCS (0)
#endif

#if ((USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) > 12)
    #error "The number of BLE Activities exceeds the limit."
#endif

#ifndef CFG_MAX_BOND_DEVS
    #error "CFG_MAX_BOND_DEVS is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_BOND_DEVS <= 10)
    #define USER_MAX_BOND_DEVS CFG_MAX_BOND_DEVS
#else
    #define USER_MAX_BOND_DEVS (1)
#endif

#ifndef CFG_MAX_PRFS
    #error "CFG_MAX_PRFS is not defined in app's custom_config.h ."
#endif

#ifndef CFG_MESH_SUPPORT
    #error "CFG_MESH_SUPPORT is not defined in app's custom_config.h ."
#endif

#if (CFG_MAX_PRFS <= 64)
    #define USER_MAX_PRFS CFG_MAX_PRFS
#else
    #define USER_MAX_PRFS (1)
#endif

/* The macro is used to compute size of the heap block in bytes. */
#define MEM_HEAP_HEADER                     (12 / sizeof(uint32_t))
#define MEM_CALC_HEAP_LEN(len)              ((((len) + (sizeof(uint32_t) - 1)) / sizeof(uint32_t)) + MEM_HEAP_HEADER)
#define MEM_CALC_HEAP_LEN_IN_BYTES(len)     (MEM_CALC_HEAP_LEN(len) * sizeof(uint32_t))

#define ENV_HEAP_SIZE       MEM_CALC_HEAP_LEN_IN_BYTES(292 * USER_MAX_CONNECTIONS \
                                                     + 426 * (USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                     + 600)
/* The size of heap for ATT database depends on the number of attributes in
 * profiles. The value can be tuned based on supported profiles. */
#if (CFG_MESH_SUPPORT == 1)
#include "mesh_stack_config.h"
#define ATT_DB_HEAP_SIZE    MEM_CALC_HEAP_LEN_IN_BYTES(1000 + MESH_HEAP_SIZE_ADD)
#else
#define ATT_DB_HEAP_SIZE    MEM_CALC_HEAP_LEN_IN_BYTES(1024)
#endif

#define KE_MSG_HEAP_SIZE    MEM_CALC_HEAP_LEN_IN_BYTES(1650 * (USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                     + 112 *(USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS) \
                                                     + 408 *(USER_MAX_CONNECTIONS+USER_MAX_ADVS+2*USER_MAX_PER_ADVS+USER_MAX_SCAN+USER_MAX_SYNCS) \
                                                     + 3072)
/* The size of non-retention heap is customized. This heap will used by BLE
 * stack only when other three heaps are full. */
#define NON_RET_HEAP_SIZE   MEM_CALC_HEAP_LEN_IN_BYTES(328 * 2)

#define PRF_BUF_SIZE              (92*USER_MAX_PRFS + 4)
#define BOND_BUF_SIZE             (8*USER_MAX_BOND_DEVS + 4)
#define CONN_BUF_SIZE             (372*USER_MAX_CONNECTIONS + 4)
#define SCAN_DUP_FILT_BUF_SIZE    (2 * USER_SCAN_DUP_FILT_NUM + 4)

#if (CFG_MAX_CONNECTIONS < 3) && (CFG_MAX_ADVS < 2) && (CFG_MESH_SUPPORT < 1)
#define EM_BASE_ADDR               (0xB0008000)
#define EM_BLE_ADVDATATXBUF_OFFSET (0x13A0)
#define EM_BLE_ADVDATATXBUF_END    (0x4F28)
#define EM_NVDS_OFFSET             (0x6690)

// (EM_BASE_ADDR + EM_BLE_ADVDATATXBUF_OFFSET + 4 * 1270 + 16(reserved))
#define ENV_HEAP_ADDR              (0xB000A788)

// (ENV_HEAP_ADDR + ENV_HEAP_SIZE:2900 bytes)
#define KE_MSG_HEAP_ADDR           (0xB000B2DC)

// (KE_MSG_HEAP_ADDR + MSG_HEAP_SIZE:6700 bytes)
#define KE_MSG_HEAP_END_ADDR       (0xB000CD08)

// (EM_BASE_ADDR + EM_NVDS_OFFSET + 4096 + 16(reserved))
#define ATT_DB_HEAP_ADDR           (0xB000F6A0)

// (ATT_DB_HEAP_ADDR + ATT_DB_HEAP_SIZE:1024)
#define NON_RET_HEAP_ADDR          (0xB000FAA0)

// (NON_RET_HEAP_ADDR + ATT_DB_HEAP_SIZE:1024)
#define NON_RET_HEAP_END_ADDR      (0xB000FEA0)

#define STACK_HEAP_INIT(heaps_table)    uint8_t prf_buf[PRF_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t bond_buf[BOND_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t conn_buf[CONN_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t scan_dup_filt_buf[SCAN_DUP_FILT_BUF_SIZE] __attribute__((aligned (32))) = {0};\
stack_heaps_table_t heaps_table = {     (uint32_t *)(ENV_HEAP_ADDR),\
                                        (uint32_t *)(ATT_DB_HEAP_ADDR),\
                                        (uint32_t *)(KE_MSG_HEAP_ADDR),\
                                        (uint32_t *)(NON_RET_HEAP_ADDR),\
                                        2900,\
                                        1024,\
                                        6700,\
                                        1024,\
                                        (uint8_t *)prf_buf,\
                                        PRF_BUF_SIZE,\
                                        (uint8_t *)bond_buf,\
                                        BOND_BUF_SIZE,\
                                        (uint8_t *)conn_buf,\
                                        CONN_BUF_SIZE,\
                                        scan_dup_filt_buf,\
                                        SCAN_DUP_FILT_BUF_SIZE}
#else
#define STACK_HEAP_INIT(heaps_table)    uint8_t prf_buf[PRF_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t bond_buf[BOND_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t conn_buf[CONN_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t scan_dup_filt_buf[SCAN_DUP_FILT_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t env_heap_buf[ENV_HEAP_SIZE] __attribute__((aligned (32)))= {0};\
                                        uint8_t att_db_heap_buf[ATT_DB_HEAP_SIZE] __attribute__((aligned (32)))= {0};\
                                        uint8_t ke_msg_heap_buf[KE_MSG_HEAP_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t non_ret_heap_buf[NON_RET_HEAP_SIZE]__attribute__((aligned (32))) = {0};\
stack_heaps_table_t heaps_table = { (uint32_t *)env_heap_buf,\
                                        (uint32_t *)att_db_heap_buf,\
                                        (uint32_t *)ke_msg_heap_buf,\
                                        (uint32_t *)non_ret_heap_buf,\
                                        ENV_HEAP_SIZE,\
                                        ATT_DB_HEAP_SIZE,\
                                        KE_MSG_HEAP_SIZE,\
                                        NON_RET_HEAP_SIZE,\
                                        (uint8_t *)prf_buf,\
                                        PRF_BUF_SIZE,\
                                        (uint8_t *)bond_buf,\
                                        BOND_BUF_SIZE,\
                                        (uint8_t *)conn_buf,\
                                        CONN_BUF_SIZE,\
                                        scan_dup_filt_buf,\
                                        SCAN_DUP_FILT_BUF_SIZE}
#endif

#endif // __SCATTER_COMMON_H__
