/**
 ****************************************************************************************
 *
 * @file wss.h
 *
 * @brief Weight Scale Service API.
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_WSS Weight Scale Service (WSS)
 * @{
 * @brief Weight Scale Service module.
 *
 * @details The Weight Scale (WS) Service exposes weight and related data from a weight scale
 *          (Server) intended for consumer healthcare as well as sports/fitness applications.
 *
 *          After \ref wss_init_t variable is intialized, the application must call \ref wss_service_init()
 *          to add Weight Scale Feature and Weight Measurement characteristics to the BLE Stack database 
 *          according to \ref wss_init_t.char_mask.
 */

#ifndef __WSS_H__
#define __WSS_H__

#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup WSS_MACRO Defines
 * @{
 */
#define WSS_CONNECTION_MAX                 10                              /**< Maximum number of Weight Scale Service connections. */
#define WSS_MEAS_VAL_LEN_MAX               15                              /**< Maximum length of Weight Measurment value. */
#define WSS_CACHE_MEAS_NUM_MAX             25                              /**< Maximum number of cache muasurements value for each user. */
#define WSS_FEAT_VAL_LEN_MAX               1                               /**< Maximum length of WS Feature value. */
#define WSS_MEAS_UNSUCCESS                 0xFFFF                          /**< Measurement Unsuccessful. */
/**
 * @defgroup WSS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref wss_init_t.char_mask.
 */
#define WSS_CHAR_FEAT_MANDATORY            0x7F     /**< Bit mask for mandatory characteristic in WSS. */
/** @} */
/** @} */

/**
 * @defgroup WSS_ENUM Enumerations
 * @{
 */
/**@brief Weight Scale Measurement Flags.*/
enum wss_meas_flag_bits
{
    WSS_MEAS_FLAG_UNIT_SI              = 0x00,     /**< Flag bit for SI Measurement Units Present. */
    WSS_MEAS_FLAG_UNIT_IMPERIAL        = 0x01,     /**< Flag bit for Imperial Measurement Units Present. */
    WSS_MEAS_FLAG_DATE_TIME_PRESENT    = 0x02,     /**< Flag bit for Time Stamp Present. */
    WSS_MEAS_FLAG_USER_ID_PRESENT      = 0x04,     /**< Flag bit for User ID Present. */
    WSS_MEAS_FLAG_BMI_HEIGHT_PRESENT   = 0x08,     /**< Flag bit for BMI and Height Present. */
};

/**@brief Weight Scale Feature characteristic bit values.*/
typedef enum
{
    /* Supported Flags */
    WSS_FEAT_TIME_STAMP        = 0x00000001,  /**< Time Stamp supported. */
    WSS_FEAT_MULTI_USER        = 0x00000002,  /**< Multiple Users supported. */
    WSS_FEAT_BMI               = 0x00000004,  /**< BMI supported. */

    /* Weight Measurement Resolution */
    WSS_FEAT_MASS_RES_500G     = 0x00000008,  /**< Resolution of 0.5kg or 1lb. */
    WSS_FEAT_MASS_RES_200G     = 0x00000010,  /**< Resolution of 0.2kg or 0.5lb. */
    WSS_FEAT_MASS_RES_100G     = 0x00000018,  /**< Resolution of 0.1kg or 0.2lb. */
    WSS_FEAT_MASS_RES_50G      = 0x00000020,  /**< Resolution of 0.05kg or 0.1lb. */
    WSS_FEAT_MASS_RES_20G      = 0x00000028,  /**< Resolution of 0.02kg or 0.05lb. */
    WSS_FEAT_MASS_RES_10G      = 0x00000030,  /**< Resolution of 0.01kg or 0.02lb. */
    WSS_FEAT_MASS_RES_5G       = 0x00000038,  /**< Resolution of 0.005kg or 0.01lb. */
    
    /* Height Measurement Resolution */
    WSS_FEAT_HEIGHT_RES_10MM   = 0x00000080,  /**< Resolution of 0.01m or 1in. */
    WSS_FEAT_HEIGHT_RES_5MM    = 0x00000100,  /**< Resolution of 0.005m or 0.5in. */
    WSS_FEAT_HEIGHT_RES_1MM    = 0x00000180,  /**< Resolution of 0.001m or 0.1in. */
    
    WSS_FEAT_FULL_BIT          = 0x000001BF,
} wss_feature_t;

/**@brief WSS Weight Measurement resolutions. */
typedef enum
{
    WSS_MASS_RES_500G, /**< Resolution of 0.5kg or 1lb. */
    WSS_MASS_RES_200G, /**< Resolution of 0.2kg or 0.5lb. */
    WSS_MASS_RES_100G, /**< Resolution of 0.1kg or 0.2lb. */
    WSS_MASS_RES_50G,  /**< Resolution of 0.05kg or 0.1lb. */
    WSS_MASS_RES_20G,  /**< Resolution of 0.02kg or 0.05lb. */
    WSS_MASS_RES_10G,  /**< Resolution of 0.01kg or 0.02lb. */
    WSS_MASS_RES_5G,   /**< Resolution of 0.005kg or 0.01lb. */
} wss_mass_res_t;

/**@brief WSS Height Measurement resolutions. */
typedef enum
{
    WSS_HEIGHT_RES_10MM, /**< Resolution of 0.01m or 1in. */
    WSS_HEIGHT_RES_5MM,  /**< Resolution of 0.005m or 0.5in. */
    WSS_HEIGHT_RES_1MM,  /**< Resolution of 0.001m or 0.1in. */
} wss_height_res_t;

/**@brief WSS unit types. */
typedef enum
{
    WSS_UNIT_SI,        /**< Weight in kilograms and height in meters */
    WSS_UNIT_IMPERIAL,  /**< Weight in pounds and height in inches */
} wss_unit_t;

 /**@brief Weight Scale Service event type. */
typedef enum
{
    WSS_EVT_INVALID,                    /**< Indicate that invalid event. */
    WSS_EVT_MEAS_INDICATION_ENABLE,     /**< Indicate that body composition measurement indication has been enabled. */
    WSS_EVT_MEAS_INDICATION_DISABLE,    /**< Indicate that body composition measurement indication has been disabled. */
    WSS_EVT_MEAS_INDICATION_CPLT,       /**< Indicate that BC Measurement has been indicated. */
    WSS_EVT_MEAS_READ_CHARACTERISTIC,   /**< The peer reads the characteristic. */
} wss_evt_type_t;                       
/** @} */

/**                                     
 * @defgroup WSS_STRUCT Structures
 * @{
 */                                     
/**@brief Weight Scale Measurement data. */
typedef struct
{
    uint16_t         weight;            /**< Weight. */
    prf_date_time_t  time_stamp;        /**< Time stamp. */
    uint8_t          user_id;           /**< User index. */
    uint16_t         bmi;               /**< Bmi. */
    uint16_t         height;            /**< Height. */
} wss_meas_val_t;

/**@brief Weight Scale Service event. */
typedef struct
{
    wss_evt_type_t evt_type;    /**< The WSS event type. */
    uint8_t        conn_idx;    /**< The index of the connection. */ 
    const uint8_t  *p_data;     /**< Pointer to event data. */ 
    uint16_t       length;      /**< Length of event data. */
} wss_evt_t;
/** @} */

/**
 * @defgroup WSS_TYPEDEF Typedefs
 * @{
 */
/**@brief Weight Scale Service event handler type. */
typedef void (*wss_evt_handler_t)(wss_evt_t *p_evt);
/** @} */

/**
 * @defgroup WSS_STRUCT Structures
 * @{
 */
/**@brief Weight Scale Service Init variable. */
typedef struct
{
    wss_evt_handler_t  evt_handler;           /**< Weight Scale Service event handler. */
    uint32_t           feature;               /**< Initial value for features. */
    uint8_t            char_mask;             /**< Initial mask of Supported characteristics, and configured with \ref WSS_CHAR_MASK */ 
    wss_unit_t         wss_unit;              /**< Initial the unit system as SI or Imperial. */
    wss_mass_res_t     wss_mass_res;          /**< Initial resolution of mass value. */
    wss_height_res_t   wss_height_res;        /**< Initial resolution of height value. */
    bool               multi_user_present;    /**< Flag which indicates if multiple user is present. */
    bool               time_stamp_present;    /**< Flag which indicates if time stamp is present. */
    bool               bmi_present;           /**< Flag which indicates if bmi is present. */
} wss_init_t;
/** @} */

/**
 * @defgroup WSS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Weight Scale Service instance and add in the DB.
 *
 * @param[in] p_wss_init:         Pointer to WSS Service initialization variable.
 * @param[in] p_bcs_start_handle: Pointer to the included service(BCS) start handle.
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t wss_service_init(wss_init_t *p_wss_init, uint16_t *p_bcs_start_handle);

/**
 *****************************************************************************************
 * @brief Send Weight Scale Measurement indication..
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_meas:     Pointer to weight scale measurement data.
 * @param[in] cache_num:  The number of measurement caches.
 *
 * @return Result of indicate value.
 *****************************************************************************************
 */
sdk_err_t wss_measurement_send(uint8_t conn_idx, wss_meas_val_t *p_meas, uint8_t cache_num);

/** @} */

#endif
/** @} */
/** @} */

