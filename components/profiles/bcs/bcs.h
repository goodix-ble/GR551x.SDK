/**
 *****************************************************************************************
 *
 * @file bcs.h
 *
 * @brief Body Composition Service API.
 *
 *****************************************************************************************
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
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_BCS Body Composition Service (BCS)
 * @{
 * @brief Definitions and prototypes for the BCS interface.
 *
 * @details The Body Composition Service (BCS) exposes data related to body composition from a
 *          body composition analyzer (Server) intended for consumer healthcare as well as
 *          sports/fitness applications. This module implements the Body Compositon Service
 *          with the Body Composition Feature and Body Composition Measurement characteristics. 
 *
 *          After \ref bcs_init_t variable is initialized, the application must call \ref bcs_service_init()
 *          to optionally add the Body Compisition Service, Body Composition Meaturement characteristics to 
 *          the BLE stack database according to \ref bcs_init_t.char_mask.
 */

#ifndef _BCS_H_
#define _BCS_H_

#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>


/**
 * @defgroup BCS_MACRO Defines
 * @{
 */
#define BCS_CONNECTION_MAX                10                              /**< Maximum number of Body Composition Service connections. */
#define BCS_MEAS_VAL_LEN_MAX              20                              /**< Maximum length of BC Measurment value. */
#define BCS_FEAT_VAL_LEN_MAX              4                               /**< Maximum length of BC Feature value. */

#define INDI_PAYLOAD_HEADER_LEN           3                               /**< The length of indication payload header. */

/**
 * @defgroup BCS_MEAS_PACKETS_INDEX Measurement packets index
 * @{
 * @brief BCS Measurement Packet Index.
 *        The least value of MTU is 23 octets. The size of Attribute Value is (23-3)
 *        octets which could be less than the size (30 octets) of all fields of Body
 *        Composition Measurement. If the required data exceeds the current size (MTU - 3)
 *        octets, the remaining optional fields shall be sent in the subsequent indication. 
 *        So we need no more than 2 packets. 
 */
#define NUM_PACKETS                       2                                /**< Measurement Packet numbers. */
#define MEAS_PACKET_FIRST                 0                                /**< The first Measurement Packet. */
#define MEAS_PACKET_SUB                   1                                /**< The second Measurement Packet. */
/** @} */

#define BCS_CACHE_MEAS_NUM_MAX            25                               /**< Maximum number of cache muasurements value for each user. */

#define BCS_MEAS_UNSUCCESS                0xFFFF                           /**< Measurement unsuccessful. */

#define BCS_MEAS_FLAG_DEFAULT             0x01FF                           /**< Measurement default flag. */

/**
 * @defgroup BCS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref bcs_init_t.char_mask.
 */
#define BCS_CHAR_FEAT_MANDATORY           0x3F                             /**< Bit mask for mandatory characteristic in BCS. */
/** @} */
/** @} */


/**
 * @defgroup BCS_ENUM Enumerations
 * @{
 */
/**
 * @defgroup BCS_MEAS_FLAG_BIT Measurement Flag Bits
 * @{
 */
/**@brief Body Composition Measurement Flags. */
enum bcs_meas_flag_bits
{
    BCS_MEAS_FLAG_UNIT_SI              = 0x0000,     /**< Flag bit for SI Measurement Units Present. */
    BCS_MEAS_FLAG_UNIT_IMPERIAL        = 0x0001,     /**< Flag bit for Imperial Measurement Units Present. */
    BCS_MEAS_FLAG_DATE_TIME_PRESENT    = 0x0002,     /**< Flag bit for Time Stamp Present. */
    BCS_MEAS_FLAG_USER_ID_PRESENT      = 0x0004,     /**< Flag bit for User ID Present. */
    BCS_MEAS_FLAG_BASAL_METABOLISM     = 0x0008,     /**< Flag bit for Basal Metabolism Present. */
    BCS_MEAS_FLAG_MUSCLE_PERCENTAGE    = 0x0010,     /**< Flag bit for Muscle Percentage Present. */
    BCS_MEAS_FLAG_MUSCLE_MASS          = 0x0020,     /**< Flag bit for Muscle Mass Present. */
    BCS_MEAS_FLAG_FAT_FREE_MASS        = 0x0040,     /**< Flag bit for Fat Free Mass Present. */
    BCS_MEAS_FLAG_SOFT_LEAN_MASS       = 0x0080,     /**< Flag bit for Soft Lean Mass Present. */
    BCS_MEAS_FLAG_BODY_WATER_MASS      = 0x0100,     /**< Flag bit for Body Water Mass Present. */
    BCS_MEAS_FLAG_IMPEDANCE            = 0x0200,     /**< Flag bit for Impedance Present. */
    BCS_MEAS_FLAG_WEIGHT               = 0x0400,     /**< Flag bit for Weight Present. */
    BCS_MEAS_FLAG_HEIGHT               = 0x0800,     /**< Flag bit for Height Present. */
    BCS_MEAS_FLAG_MUTI_PACKET          = 0x1000,     /**< Flag bit for Multiple Packet Measurement Present. */
};
/** @} */

/**@brief Body Composition Feature characteristic bit values. */
typedef enum
{
    /* Supported Flags */
    BCS_FEAT_TIME_STAMP        = 0x00000001,  /**< Time Stamp supported */
    BCS_FEAT_MULTI_USER        = 0x00000002,  /**< Multiple Users supported */
    BCS_FEAT_BASAL_METABOLISM  = 0x00000004,  /**< Basal metabolism supported */
    BCS_FEAT_MUSCLE_PERCENTAGE = 0x00000008,  /**< Muscle percentage supported */
    BCS_FEAT_MUSCLE_MASS       = 0x00000010,  /**< Muscle mass supported */
    BCS_FEAT_FAT_FREE_MASS     = 0x00000020,  /**< Fat free mass supported */
    BCS_FEAT_SOFT_LEAN_MASS    = 0x00000040,  /**< Soft lean mass supported */
    BCS_FEAT_BODY_WATER_MASS   = 0x00000080,  /**< Body water mass supported */
    BCS_FEAT_IMPEDANCE         = 0x00000100,  /**< Impedance supported */
    BCS_FEAT_WEIGHT            = 0x00000200,  /**< Weight supported */
    BCS_FEAT_HEIGHT            = 0x00000400,  /**< Height supported */

    /* Mass Resolution */
    BCS_FEAT_MASS_RES_500G     = 0x00000800,  /**< Resolution of 0.5kg or 1lb */
    BCS_FEAT_MASS_RES_200G     = 0x00001000,  /**< Resolution of 0.2kg or 0.5lb */
    BCS_FEAT_MASS_RES_100G     = 0x00001800,  /**< Resolution of 0.1kg or 0.2lb */
    BCS_FEAT_MASS_RES_50G      = 0x00002000,  /**< Resolution of 0.05kg or 0.1lb */
    BCS_FEAT_MASS_RES_20G      = 0x00002800,  /**< Resolution of 0.02kg or 0.05lb */
    BCS_FEAT_MASS_RES_10G      = 0x00003000,  /**< Resolution of 0.01kg or 0.02lb */
    BCS_FEAT_MASS_RES_5G       = 0x00003800,  /**< Resolution of 0.005kg or 0.01lb */

    /* Height Resolution */
    BCS_FEAT_HEIGHT_RES_10MM   = 0x00008000,  /**< Resolution of 0.01m or 1in */
    BCS_FEAT_HEIGHT_RES_5MM    = 0x00010000,  /**< Resolution of 0.005m or 0.5in */
    BCS_FEAT_HEIGHT_RES_1MM    = 0x00018000,  /**< Resolution of 0.001m or 0.1in */

    BCS_FEAT_FULL_BIT          = 0x0001BFFF,
} bcs_feature_t;

/**@brief BCS Weight Measurement resolutions. */
typedef enum
{
    BCS_MASS_RES_500G, /**< Resolution of 0.5kg or 1lb. */
    BCS_MASS_RES_200G, /**< Resolution of 0.2kg or 0.5lb. */
    BCS_MASS_RES_100G, /**< Resolution of 0.1kg or 0.2lb. */
    BCS_MASS_RES_50G,  /**< Resolution of 0.05kg or 0.1lb. */
    BCS_MASS_RES_20G,  /**< Resolution of 0.02kg or 0.05lb. */
    BCS_MASS_RES_10G,  /**< Resolution of 0.01kg or 0.02lb. */
    BCS_MASS_RES_5G,   /**< Resolution of 0.005kg or 0.01lb. */
} bcs_mass_res_t;

/**@brief BCS Height Measurement resolutions. */
typedef enum
{
    BCS_HEIGHT_RES_10MM, /**< Resolution of 0.01m or 1in. */
    BCS_HEIGHT_RES_5MM,  /**< Resolution of 0.005m or 0.5in. */
    BCS_HEIGHT_RES_1MM,  /**< Resolution of 0.001m or 0.1in. */
} bcs_height_res_t;

/**@brief BCS unit types. */
typedef enum
{
    BCS_UNIT_SI,        /**< Weight in kilograms and height in meters */
    BCS_UNIT_IMPERIAL,  /**< Weight in pounds and height in inches */
} bcs_unit_t;

 /**@brief Body Composition Service event type. */
typedef enum
{
    BCS_EVT_INVALID,                    /**< Indicate that invalid event. */
    BCS_EVT_MEAS_INDICATION_ENABLE,     /**< Indicate that body composition measurement indication has been enabled. */
    BCS_EVT_MEAS_INDICATION_DISABLE,    /**< Indicate that body composition measurement indication has been disabled. */
    BCS_EVT_MEAS_INDICATION_CPLT,       /**< Indicate that BC Measurement has been indicated. */
    BCS_EVT_MEAS_READ_CHARACTERISTIC,   /**< The peer reads the characteristic. */
} bcs_evt_type_t;
/** @} */

/**
 * @defgroup BCS_STRUCT Structures
 * @{
 */
/**@brief Body composition Measurement flag data. */
typedef struct
{
    uint8_t  time_stamp_present        :1;  /**< Time Stamp flag. */
    uint8_t  user_id_present           :1;  /**< User ID flag. */
    uint8_t  basal_metabolism_present  :1;  /**< Basal Metabolism flag. */
    uint8_t  muscle_percentage_present :1;  /**< Muscle Percentage flag. */
    uint8_t  muscle_mass_present       :1;  /**< Muscle Mass flag. */
    uint8_t  fat_free_mass_present     :1;  /**< Fat Free Mass flag. */
    uint8_t  soft_lean_mass_present    :1;  /**< Soft Lean Mass flag. */
    uint8_t  body_water_mass_present   :1;  /**< Body Water Mass flag. */
    uint8_t  impedance_present         :1;  /**< Impedance flag. */
    uint8_t  weight_present            :1;  /**< Weight flag. */
    uint8_t  height_present            :1;  /**< Height flag. */  
} bcs_meas_flag_t;

/**@brief Body composition Measurement data. */
typedef struct
{
    uint16_t        body_fat_percentage;    /**< Body Fat Percentage data. */
    prf_date_time_t time_stamp;             /**< Time Stamp data. */
    uint8_t         user_id;                /**< User Index data. */
    uint16_t        basal_metabolism;       /**< Basal Metabolism data. */
    uint16_t        muscle_percentage;      /**< Muscle Percentage data. */
    uint16_t        muscle_mass;            /**< Muscle Mass data. */
    uint16_t        fat_free_mass;          /**< Fat Free Mass data. */
    uint16_t        soft_lean_mass;         /**< Soft Lean Mass data. */
    uint16_t        body_water_mass;        /**< Body Water Mass data. */
    uint16_t        impedance;              /**< Impedance data. */
    uint16_t        weight;                 /**< Weight data. */
    uint16_t        height;                 /**< Height data. */
} bcs_meas_val_t;

/**@brief Body Composition Service event. */
typedef struct
{
    bcs_evt_type_t evt_type;    /**< The BCS event type. */
    uint8_t        conn_idx;    /**< The index of the connection. */
    const uint8_t *p_data;      /**< Pointer to event data. */
    uint16_t       length;      /**< Length of event data. */
} bcs_evt_t;
/** @} */

/**
 * @defgroup BCS_TYPEDEF Typedefs
 * @{
 */
/**@brief Body Composition Service event handler type. */
typedef void (*bcs_evt_handler_t)(bcs_evt_t *p_evt);
/** @} */

/**
 * @defgroup BCS_STRUCT Structures
 * @{
 */
/**@brief Body Composition Service Init variable. */
typedef struct
{
    bcs_evt_handler_t  evt_handler;           /**< Body Composition Service event handler. */
    uint32_t           feature;               /**< Initial value for features. */
    uint8_t            char_mask;             /**< Initial mask of Supported characteristics, and configured with \ref BCS_CHAR_MASK */ 
    bcs_unit_t         bcs_unit;              /**< Initial unit system as SI or Imperial. */
    bcs_meas_flag_t    bcs_meas_flags;        /**< Initial measurement flags. */
    bcs_mass_res_t     bcs_mass_res;          /**< Initial resolution of mass value. */
    bcs_height_res_t   bcs_height_res;        /**< Initial resolution of height value. */
} bcs_init_t;
/** @} */

/**
 * @defgroup BCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Body Composition Service instance and add in the DB.
 *
 * @param[in] p_bcs_init:  Pointer to BC Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t bcs_service_init(bcs_init_t *p_bcs_init);

/**
 ******************************************************************************************
 * @brief Send Body Composition Measurement indication.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_meas:     Pointer to body composition measurement data.
 * @param[in] cache_num:  The number of measurement caches.
 *
 * @return SDK_SUCCESS on success, otherwise an error code.
 ***************************************************************************************** 
 */
sdk_err_t bcs_measurement_send(uint8_t conn_idx, bcs_meas_val_t *p_meas, uint8_t cache_num);

/**
 ******************************************************************************************
 * @brief Get the pointer to the start handle of Body Composition Service.
 * 
 *
 * @return The pointer to the start handle.
 ***************************************************************************************** 
 */
uint16_t * bcs_start_handle_get(void);
/** @} */

#endif
/** @} */
/** @} */

