/**
 ****************************************************************************************
 *
 * @file ble_audio.h
 *
 * @brief BLE Audio API
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
#ifndef __BLE_AUDIO_H__
#define __BLE_AUDIO_H__

#if (CFG_SNIFFER)

   /**
 * @addtogroup BLE
 * @{
 * @brief Definitions and prototypes for the BLE SDK interface.
 */
 
  /**
 * @addtogroup BLE_AUDIO Audio Library
 * @{
 * @brief Definitions and prototypes for the Audio Application interface.
 */

#include "ble_gapc.h"
#include "ble_error.h"

#include <stdint.h>         // Standard Integer
#include <string.h>
#include <stdbool.h>

/**
 * @defgroup BLE_AUDIO_ENUM Enumerations
 * @{
 */
/** @brief Switch role type */
typedef enum
{
    AUDIO_SWITCH_ROLE_SINK_TYPE    = 1 << 0,     /**< Switch role for sink link. */
    AUDIO_SWITCH_ROLE_SNIFFER_TYPE = 1 << 1,     /**< Switch role for sniffer link. */
} audio_switch_role_type_t;

/**
 * @brief Definition of operation code used in the asynchronous operations
 */
 typedef enum
{
    AUDIO_OPCODE_SWITCH_ROLE,       /**< Switch Role. */
    AUDIO_OPCODE_TIME_SYNC,         /**< Time Sync. */
    AUDIO_OPCODE_CREATE_SNIFFER,    /**<Create Sniffer. */
    AUDIO_OPCODE_SYNC_INFO,         /**<Sync info. */
    AUDIO_OPCODE_READ_EVENT_COUNT,  /**<Read event count. */
} audio_op_id_t;

/**@brief Synchronization information type. */
typedef enum
{
    SYNC_INFO_LECB_CONN = 0x00,            /**< Synchronize lecb connection information. */
    SYNC_INFO_LECB_CREDIT,                 /**< Synchronize lecb credit information. */
    SYNC_INFO_SEC,                         /**< Synchronize security information. */
} audio_sync_info_type_t;
/** @} */

/**
 * @defgroup BLE_AUDIO_STRUCT Structures
 * @{
 */
/** @brief Command Complete Event structure. */
typedef struct
{
    audio_op_id_t   op_id;          /**< Operation code. @see audio_op_id_t */
    uint16_t        error_code;     /**< Error code. @see uint16_t */
} audio_op_cmp_evt_t;

/** @brief Switch Role Indication structure. */
typedef struct
{
    uint8_t  switch_type;  /**< Switch role type. */
} audio_switch_role_ind_t;

/** @brief Structure for LE Credit Based Connection Created indication. */
typedef struct
{
    uint8_t  status;       /**< Status. */
    uint16_t le_psm;       /**< Le_psm number. */
    uint16_t local_cid;    /**< The local source channel id. */
    uint16_t peer_credits; /**< It indicates the number of LE-frames that the peer device can receive. */
    uint16_t peer_mtu;     /**< It indicates the maximum SDU size (in octets) that the L2CAP layer entity sending the LE Credit Based Connection Request can receive on this channel.  */
    uint16_t peer_mps;     /**< It indicates the maximum payload size (in octets) that the L2CAP layer entity sending the LE Credit Based Connection Request is capable of receiving on this channe. */
} audio_lecb_conn_ind_t;

/** @brief Structure for event count indication. */
typedef struct
{
    uint16_t event_count;  /**< Event count number. */
} audio_event_count_ind_t;

/** @brief Audio Callback function structure. */
typedef struct
{  
    /**@brief This callback function is called when the cmd has completed.
     * @param[in] conidx Connection index for connection control operations. It is set to zero for non-connection operations.
     * @param[in] param Info of the operation. @see audio_op_cmp_evt_t
     */
    void (*app_audio_opera_cmp_evt_cb)(uint8_t conidx, const audio_op_cmp_evt_t *param);    
      
    /**@brief This callback function is called when receiving Sniffer Created indication.
     * @param[in] conidx Connection index.
     * @param[in] param Connection info. @see gap_conn_cmp_t
     */
    void (*app_audio_sniffer_created_ind_cb)(uint8_t conidx,  const gap_conn_cmp_t *param);

    /**@brief This callback function will be called when switch role has completed.
     * @param[in]  conidx Connection index.
     * @param[in]  param Switch role info. @see audio_switch_role_ind_t
     */
    void (*app_audio_rcv_switch_role_ind_cb)(uint8_t conidx,  const audio_switch_role_ind_t *param); 

    /**@brief This callback function is called when receiving the sync information of LE Credit Based connection.
     * @param[in] conidx Connection index.
     * @param[in] param LE Credit Based Connection information. @see audio_lecb_conn_ind_t
     */
    void (*app_audio_rcv_sync_lecb_ind_cb)(uint8_t conidx, audio_lecb_conn_ind_t *param);

    /**@brief This callback function is called when receiving the event count indication.
     * @param[in] conidx Connection index.
     * @param[in] param Event count info. @see audio_event_count_ind_t
     */
    void (*app_audio_rcv_event_count_ind_cb)(uint8_t conidx, audio_event_count_ind_t *param);

}audio_cb_fun_t;
/** @} */

/**
 * @defgroup BLE_AUDIO_FUNCTION Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Register the callback function.
 * @note When APP is initialized, this function should be called, and the parameter cb must point to a global. 
 *
 * @param[in] cb: Pointer to the Callback function.
 ****************************************************************************************
 */
void ble_audio_callback_register(audio_cb_fun_t* cb);

/**
 ****************************************************************************************
 * @brief Switch role.
 *
 * @param[in] conidx:      Connection index.
 * @param[in] switch_type: Swich role type, @see audio_switch_role_type_t
 *
 * @retval uint16_t @see uint16_t
 ****************************************************************************************
 */   
uint16_t ble_audio_role_switch(uint8_t conidx, uint8_t switch_type);

/**
 ****************************************************************************************
 * @brief Time Sync
 *
 * @param[in] conidx:         Connection index.
 * @param[in] time_sync_en:   Enable time sync.
 * @param[in] sync_pulse_sel: Synchronization pulse selection.
 *
 * @retval uint16_t @see uint16_t
 ****************************************************************************************
 */    
uint16_t ble_audio_time_sync(uint8_t conidx, uint8_t time_sync_en, uint8_t  sync_pulse_sel);


/**
 ****************************************************************************************
 * @brief Create sniffer link.
 *
 * @param[in] source_connidx: Connection index for the source link (between active headset and phone).
 * @param[in] sink_connidx:   Connection index for the sink link (between left headset and right headset).
 *
 * @note This API is asynchronous. 
 * @note app_audio_rcv_sniffer_created_cmp_cb (see @ref audio_cb_fun_t)
 *           will be called once the operation has completed.
 ****************************************************************************************
 */
uint16_t ble_audio_sniffer_create(uint8_t source_connidx, uint8_t sink_connidx);


/**
 ****************************************************************************************
 * @brief Synchronization information for the peer audio device.
 *
 * @param[in] sync_info_type: Synchronization information type.
 * @param[in] source_connidx: Connection index for the source link.
 * @param[in] sink_connidx:   Connection index for the sink link.
 *
 * @retval ::SDK_SUCCESS Synchronization information type is successfully set to the BLE stack.
 * @retval ::SDK_ERR_INVALID_PARAM The parameter is invalid, such as the source_connidx or sink_connidx is invalid value.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_audio_sync_info_send (audio_sync_info_type_t sync_info_type, uint8_t source_connidx, uint8_t sink_connidx);

/**
 ****************************************************************************************
 * @brief Read event count.
 *
 * @param[in] connidx: Connection index for the link.
 *
 * @note This API is asynchronous. 
 *
 * @retval ::SDK_SUCCESS Operation is successfully set to the BLE stack.
 * @retval ::SDK_ERR_INVALID_PARAM The parameter is invalid, such as the connidx invalid value.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_audio_event_count_read(uint8_t connidx);
/** @} */
/** @} */
/** @} */

#endif //#if (CFG_SNIFFER)
#endif //#ifndef __BLE_AUDIO_H__
