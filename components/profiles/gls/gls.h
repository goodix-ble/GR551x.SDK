/**
 ****************************************************************************************
 *
 * @file gls.h
 *
 * @brief Glucose Service API.
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
 * @defgroup BLE_SDK_GLS Glucose Service (GLS)
 * @{
 * @brief Glucose Service module.
 *
 * @details The Glucose Service exposes glucose and other data related to a personal glucose
 *          sensor for consumer healthcare applications and is not designed for clinical use.
 *          This module implements the Glucose Service with Glucose Measurement, Glucose Measurement
 *          Context, Glucose Feature and Record Access Control Point characteristics.
 *
 *          After \ref gls_init_t variable is intialized, the application must call \ref gls_service_init()
 *          to add Glucose Service and Glucose Measurement, Glucose Measurement Context, Glucose
 *          Feature and Record Access Control Point characteristics to the BLE Stack database
 *          according to \ref gls_init_t.char_mask.
 */

#ifndef __GLS_H__
#define __GLS_H__

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gr_includes.h"
#include "custom_config.h"
#include "ble_prf_utils.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup GLS_MACRO Defines
 * @{
 */
#define GLS_CONNECTION_MAX                  10                           /**< Maximum number of Glucose Profile connections. */
#define GLS_MEAS_VAL_LEN_MAX                20                           /**< Maximum length of GLS measurement value. */
#define GLS_MEAS_CTX_LEN_MAX                20                           /**< Maximum length of GLS measurement context value. */
#define GLS_REC_ACCESS_CTRL_LEN_MIN         2                            /**< Minimum length of Record Access Control Point packet. */
#define GLS_REC_ACCESS_CTRL_LEN_MAX         21                           /**< Maximum length of Record Access Control Point packet. */


#define GLS_NTF_OF_NULL              0x00       /**< Mask for no notify. */
#define GLS_NTF_OF_MEAS              0x01       /**< Mask for measurement notify. */
#define GLS_NTF_OF_MEAS_CTX          0x10       /**< Mask for measurement context notify. */

#define GLS_ERROR_PROC_IN_PROCESS    0x80       /**< Error code: A previously triggered SC Control Point operation is still in progress. */
#define GLS_ERROR_CCCD_INVALID       0x81       /**< Error code: The Client Characteristic Configuration descriptor is not configured. */

/**
 * @defgroup GLS_CHAR_MASK Characteristics Mask
 * @{
 * @brief Bit masks for the initialization of \ref gls_init_t.char_mask.
 */
#define GLS_CHAR_MANDATORY              0x0f8f     /**< Bit mask for mandatory characteristic in GLS. */
#define GLS_CHAR_MEAS_CTX_SUP           0x0070     /**< Bit mask for Glucose Measurement Context characteristic that is optional. */
#define GLS_CHAR_FULL                   0x0fff     /**< Bit mask of the full characteristic. */
/** @} */

/**
 * @defgroup GLS_FEAT Glucose Feature
 * @{
 * @brief Glucose Service feature.
 */
#define GLS_FEAT_LOW_BATT             (0x01 << 0)     /**< Low Battery Detection During Measurement Supported */
#define GLS_FEAT_MALFUNC              (0x01 << 1)     /**< Sensor Malfunction Detection Supported */
#define GLS_FEAT_SAMPLE_SIZE          (0x01 << 2)     /**< Sensor Sample Size Supported */
#define GLS_FEAT_INSERT_ERR           (0x01 << 3)     /**< Sensor Strip Insertion Error Detection Supported */
#define GLS_FEAT_TYPE_ERR             (0x01 << 4)     /**< Sensor Strip Type Error Detection Supported */
#define GLS_FEAT_RES_HIGH_LOW         (0x01 << 5)     /**< Sensor Result High-Low Detection Supported */
#define GLS_FEAT_TEMP_HIGH_LOW        (0x01 << 6)     /**< Sensor Temperature High-Low Detection Supported */
#define GLS_FEAT_READ_INT             (0x01 << 7)     /**< Sensor Read Interrupt Detection Supported */
#define GLS_FEAT_GENERAL_FAULT        (0x01 << 8)     /**< General Device Fault Supported */
#define GLS_FEAT_TIME_FAULT           (0x01 << 9)     /**< Time Fault Supported */
#define GLS_FEAT_MULTI_BOND           (0x01 << 10)    /**< Multiple Bond Supported */
#define GLS_FEAT_FULL                 (0x07ff)         /**< All feature Supported. */
/** @} */


/**
 * @defgroup GLS_MEAS_FLAG Measurement Flag
 * @{
 * @brief Glucose Measurement Flags.
 */
#define GLS_MEAS_FLAG_TIME_OFFSET       0x01    /**< Time Offset Present */
#define GLS_MEAS_FLAG_CONC_TYPE_LOC     0x02    /**< Glucose Concentration, Type, and Sample Location Present */
#define GLS_MEAS_FLAG_UNITS_KG_L        0x00    /**< Glucose Concentration Units kg/L */
#define GLS_MEAS_FLAG_UNITS_MOL_L       0x04    /**< Glucose Concentration Units mol/L */
#define GLS_MEAS_FLAG_SENSOR_STATUS     0x08    /**< Sensor Status Annunciation Present */
#define GLS_MEAS_FLAG_CONTEXT_INFO      0x10    /**< Context Information Follows */
/** @} */

/**
 * @defgroup GLS_MEAS_CTX_FLAG Measurement Context Flag
 * @{
 * @brief Glucose measurement context flags
 */
#define GLS_MEAS_CTX_FLAG_CARB          (0x01 << 0)    /**< Carbohydrate id and carbohydrate present */
#define GLS_MEAS_CTX_FLAG_MEAL          (0x01 << 1)    /**< Meal present */
#define GLS_MEAS_CTX_FLAG_TESTER        (0x01 << 2)    /**< Tester-health present */
#define GLS_MEAS_CTX_FLAG_EXERCISE      (0x01 << 3)    /**< Exercise duration and exercise intensity present */
#define GLS_MEAS_CTX_FLAG_MED           (0x01 << 4)    /**< Medication ID and medication present */
#define GLS_MEAS_CTX_FLAG_MED_KG        (0x01 << 5)    /**< Medication value units, kilograms */
#define GLS_MEAS_CTX_FLAG_MED_L         (0x01 << 6)    /**< Medication value units, liters */
#define GLS_MEAS_CTX_FLAG_HBA1C         (0x01 << 7)    /**< Hba1c present */
#define GLS_MEAS_CTX_FLAG_EXT           (0x01 << 8)    /**< Extended flags present */
/** @} */

/**
 * @defgroup GLS_MEAS_STATUS Measurement Status
 * @{
 * @brief Glucose measurement status annunciation.
 */
#define GLS_MEAS_STATUS_BATT_LOW       (0x01 << 0)      /**< Device battery low at time of measurement */
#define GLS_MEAS_STATUS_SENSOR_FAULT   (0x01 << 1)      /**< Sensor malfunction or faulting at time of measurement */
#define GLS_MEAS_STATUS_SAMPLE_SIZE    (0x01 << 2)      /**< Sample size for blood or control solution insufficient at time of measurement */
#define GLS_MEAS_STATUS_STRIP_INSERT   (0x01 << 3)      /**< Strip insertion error */
#define GLS_MEAS_STATUS_STRIP_TYPE     (0x01 << 4)      /**< Strip type incorrect for device */
#define GLS_MEAS_STATUS_RESULT_HIGH    (0x01 << 5)      /**< Sensor result higher than the device can process */
#define GLS_MEAS_STATUS_RESULT_LOW     (0x01 << 6)      /**< Sensor result lower than the device can process */
#define GLS_MEAS_STATUS_TEMP_HIGH      (0x01 << 7)      /**< Sensor temperature too high for valid test/result at time of measurement */
#define GLS_MEAS_STATUS_TEMP_LOW       (0x01 << 8)      /**< Sensor temperature too low for valid test/result at time of measurement */
#define GLS_MEAS_STATUS_STRIP_PULL     (0x01 << 9)      /**< Sensor read interrupted because strip was pulled too soon at time of measurement */
#define GLS_MEAS_STATUS_GENERAL_FAULT  (0x01 << 10)     /**< General device fault has occurred in the sensor */
#define GLS_MEAS_STATUS_TIME_FAULT     (0x01 << 11)     /**< Time fault has occurred in the sensor and time may be inaccurate */
/** @} */
/** @} */

/**
 * @defgroup GLS_ENUM Enumerations
 * @{
 */
/**@brief Glucose measurement type */
typedef enum
{
    GLS_MEAS_TYPE_CAP_BLOOD = 0x01,     /**< Capillary whole blood */
    GLS_MEAS_TYPE_CAP_PLASMA,           /**< Capillary plasma */
    GLS_MEAS_TYPE_VEN_BLOOD,            /**< Venous whole blood */
    GLS_MEAS_TYPE_VEN_PLASMA,           /**< Venous plasma */
    GLS_MEAS_TYPE_ART_BLOOD,            /**< Arterial whole blood */
    GLS_MEAS_TYPE_ART_PLASMA,           /**< Arterial plasma */
    GLS_MEAS_TYPE_UNDET_BLOOD,          /**< Undetermined whole blood */
    GLS_MEAS_TYPE_UNDET_PLASMA,         /**< Undetermined plasma */
    GLS_MEAS_TYPE_FLUID,                /**< Interstitial fluid (ISF) */
    GLS_MEAS_TYPE_CONTROL,              /**< Control solution */
} gls_meas_type_t;

/**@brief Glucose measurement location */
typedef enum
{
    GLS_MEAS_LOC_FINGER = 0x01,     /**< Finger */
    GLS_MEAS_LOC_AST,               /**< Alternate Site Test (AST) */
    GLS_MEAS_LOC_EAR,               /**< Earlobe */
    GLS_MEAS_LOC_CONTROL,           /**< Control solution */
    GLS_MEAS_LOC_NOT_AVAIL = 0x0f,  /**< Sample Location value not available */
} gls_meas_loc_t;

/**@brief Glucose measurement context carbohydrate ID */
typedef enum
{
    GLS_MEAS_CTX_CARB_BREAKFAST = 0x01,      /**< Breakfast */
    GLS_MEAS_CTX_CARB_LUNCH,                 /**< Lunch */
    GLS_MEAS_CTX_CARB_DINNER,                /**< Dinner */
    GLS_MEAS_CTX_CARB_SNACK,                 /**< Snack */
    GLS_MEAS_CTX_CARB_DRINK,                 /**< Drink */
    GLS_MEAS_CTX_CARB_SUPPER,                /**< Supper */
    GLS_MEAS_CTX_CARB_BRUNCH,                /**< Brunch */
} gls_meas_ctx_carb_id_t;

/**@brief Glucose measurement context meal */
typedef enum
{
    GLS_MEAS_CTX_MEAL_PREPRANDIAL  = 0x01,   /**< Preprandial (before meal) */
    GLS_MEAS_CTX_MEAL_POSTPRANDIAL,          /**< Postprandial (after meal) */
    GLS_MEAS_CTX_MEAL_FASTING,               /**< Fasting */
    GLS_MEAS_CTX_MEAL_CASUAL,                /**< Casual (snacks, drinks, etc.) */
    GLS_MEAS_CTX_MEAL_BEDTIME,               /**< Bedtime */
} gls_meas_ctx_meal_t;

/**@brief Glucose measurement context tester */
typedef enum
{
    GLS_MEAS_CTX_TESTER_SELF       = 0x01,       /**< Self */
    GLS_MEAS_CTX_TESTER_PRO,                     /**< Health care professional */
    GLS_MEAS_CTX_TESTER_LAB,                     /**< Lab test */
    GLS_MEAS_CTX_TESTER_NOT_AVAIL  = 0x0f,       /**< Tester value not available */
} gls_meas_ctx_tester_t;

/**@brief Glucose measurement context health */
typedef enum
{
    GLS_MEAS_CTX_HEALTH_MINOR      = 0x01,       /**< Minor health issues */
    GLS_MEAS_CTX_HEALTH_MAJOR,                   /**< Major health issues */
    GLS_MEAS_CTX_HEALTH_MENSES,                  /**< During menses */
    GLS_MEAS_CTX_HEALTH_STRESS,                  /**< Under stress */
    GLS_MEAS_CTX_HEALTH_NONE,                    /**< No health issues */
    GLS_MEAS_CTX_HEALTH_NOT_AVAIL  = 0x0f,       /**< Health value not available */
} gls_meas_ctx_health_t;

/**@brief Glucose measurement context medication ID */
typedef enum
{
    GLS_MEAS_CTX_MED_RAPID    = 0x01,    /**< Rapid acting insulin */
    GLS_MEAS_CTX_MED_SHORT,              /**< Short acting insulin */
    GLS_MEAS_CTX_MED_INTERMED,           /**< Intermediate acting insulin */
    GLS_MEAS_CTX_MED_LONG,               /**< Long acting insulin */
    GLS_MEAS_CTX_MED_PREMIX,             /**< Pre-mixed insulin */
} gls_meas_ctx_medic_id_t;

/**@brief Glucose Service event type. */
typedef enum
{
    GLS_EVT_INVALID = 0x00,                 /**< Invalid event. */
    GLS_EVT_MEAS_NOTIFICATION_ENABLED,      /**< Glucose Measurement notification enabled event. */
    GLS_EVT_MEAS_NOTIFICATION_DISABLED,     /**< Glucose Measurement notification disabled event. */
    GLS_EVT_CTX_NOTIFICATION_ENABLED,       /**< Glucose Measurement Context notification enabled event. */
    GLS_EVT_CTX_NOTIFICATION_DISABLED,      /**< Glucose Measurement Context notification disabled event. */
    GLS_EVT_CTRL_INDICATION_ENABLED,        /**< Record Access Control Point indication enabled event. */
    GLS_EVT_CTRL_INDICATION_DISABLED,       /**< Record Access Control Point indication disabled event. */
    GLS_EVT_CTRL_WRITE,                     /**< Record Access Control Point write. */
} gls_evt_type_t;
/** @} */

/**
 * @defgroup GLS_STRUCT Structures
 * @{
 */
/**@brief SFLOAT format (IEEE-11073 16-bit FLOAT, defined as a 16-bit value with 12-bit mantissa and 4-bit exponent). */
typedef struct
{
    int8_t  exponent;            /**< Base 10 exponent, only 4 bits. */
    int16_t mantissa;            /**< Mantissa, only 12 bits. */
} gls_ieee_float16_t;

/**@brief Glucose Service event. */
typedef struct
{
    gls_evt_type_t  evt_type;    /**< The GLS event type. */
    uint8_t         conn_idx;    /**< The index of the connection. */
    const uint8_t  *p_data;      /**< Pointer to event data. */
    uint16_t        length;      /**< Length of event data. */
} gls_evt_t;
/** @} */

/**
 * @defgroup GLS_TYPEDEF Typedefs
 * @{
 */
/**@brief Glucose Service event handler type.*/
typedef void (*gls_evt_handler_t)(gls_evt_t *p_evt);
/** @} */

/**
 * @defgroup GLS_STRUCT Structures
 * @{
 */
/**@brief Glucose Measurement structure. This contains glucose measurement value. */
typedef struct
{
    uint8_t             flags;                       /**< Flags. */
    uint16_t            sequence_number;             /**< Sequence number. */
    prf_date_time_t     base_time;                   /**< Time stamp. */
    int16_t             time_offset;                 /**< Time offset. */
    gls_ieee_float16_t  glucose_concentration;       /**< Glucose concentration. */
    uint8_t             type;                        /**< Type. */
    uint8_t             sample_location;             /**< Sample location. */
    uint16_t            sensor_status_annunciation;  /**< Sensor status annunciation. */
} gls_meas_val_t;

/**@brief Glucose measurement context structure */
typedef struct
{
    uint8_t             flags;                  /**< Flags. */
    uint8_t             extended_flags;         /**< Extended Flags. */
    uint8_t             carbohydrate_id;        /**< Carbohydrate ID. */
    gls_ieee_float16_t  carbohydrate;           /**< Carbohydrate. */
    uint8_t             meal;                   /**< Meal. */
    uint8_t             tester_and_health;      /**< Tester and health. */
    uint16_t            exercise_duration;      /**< Exercise Duration. */
    uint8_t             exercise_intensity;     /**< Exercise Intensity. */
    uint8_t             medication_id;          /**< Medication ID. */
    gls_ieee_float16_t  medication;             /**< Medication. */
    uint16_t            hba1c;                  /**< HbA1c. */
} gls_meas_ctx_t;

/**@brief Glucose measurement record */
typedef struct
{
    gls_meas_val_t  meas_val;      /**< Glucose measurement value. */
    gls_meas_ctx_t  meas_ctx;      /**< Glucose measurement context. */
} gls_rec_t;

/**@brief Glucose Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    gls_evt_handler_t  evt_handler;            /**< Glucose Service event handler. */
    uint16_t           char_mask;              /**< Initial mask of supported characteristics, and configured with \ref GLS_CHAR_MASK. */
    uint16_t           feature;                /**< Initial value for features. */
} gls_init_t;
/** @} */

/**
 * @defgroup GLS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a Glucose Service instance and add in the DB.
 *
 * @param[in] p_gls_init: Pointer to gls_init_t Service initialization variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t gls_service_init(gls_init_t *p_gls_init);

/**
 *****************************************************************************************
 * @brief Record a new glucose measurement.
 *
 * @param[in] p_rec: Pointer to glucose record (measurement plus context).
 *
 * @return If record successfully or not.
 *****************************************************************************************
 */
bool gls_new_meas_record(gls_rec_t *p_rec);

/**
 *****************************************************************************************
 * @brief Send RACP responce if indication has been enabled.
 *
 * @param[in] conn_idx: Connnection index.
 * @param[in] p_data:   Pointer to data.
 * @param[in] length:   Length of data.
 *
 * @return Result of indicate value
 *****************************************************************************************
 */
sdk_err_t gls_racp_rsp_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
/** @} */

#endif
/** @} */
/** @} */

