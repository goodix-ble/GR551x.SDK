/**
 ****************************************************************************************
 *
 * @file ble.h
 *
 * @brief include all ble sdk header files
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
 * @addtogroup BLE
 * @{
 */

/**
 @addtogroup BLE_COMMEN BLE Common
 @{
 @brief BLE Common interface.
 */

#ifndef __BLE_H__
#define __BLE_H__

#include "ble_att.h"
#include "ble_error.h"
#include "ble_gapc.h"
#include "ble_gapm.h"
#include "ble_gatt.h"
#include "ble_gattc.h"
#include "ble_gatts.h"
#include "ble_l2cap.h"
#include "ble_prf.h"
#include "ble_sec.h"
#include "ble_event.h"

#include <stdio.h> 

/** @addtogroup BLE_COMMEN_ENUM Enumerations
 * @{
 */
/**
 * @brief RF TX mode. 
 */
typedef enum
{
    BLE_RF_TX_MODE_INVALID = 0,
    BLE_RF_TX_MODE_LP_MODE = 1,
    BLE_RF_TX_MODE_ULP_MODE = 2,
} ble_rf_tx_mode_t;

/**
 * @brief The resistance value (ohm) of the RF match circuit. 
 */
typedef enum
{
    BLE_RF_MATCH_CIRCUIT_25OHM = 25,
    BLE_RF_MATCH_CIRCUIT_100OHM = 100,
} ble_rf_match_circuit_t;

/** @} */

/** @addtogroup BLE_COMMEN_STRUCTURES Structures
 * @{
 */
/**@brief The table contains the pointers to four arrays which are used
 * as heap memory by BLE stack in ROM. The size of four arrays depends on
 * the number of connections and the number of attributes of profiles. */
typedef struct 
{
    uint32_t  *env_ret;         /**< Pointer to the array for environment heap */
    uint32_t  *db_ret;          /**< Pointer to the array for ATT DB heap */
    uint32_t  *msg_ret;         /**< Pointer to the array for message heap */
    uint32_t  *non_ret;         /**< Pointer to the array for non-retention heap */
    uint16_t   env_ret_size;    /**< The size of the array for environment heap */
    uint16_t   db_ret_size;     /**< The size of the array for ATT DB heap */
    uint16_t   msg_ret_size;    /**< The size of the array for message heap */
    uint16_t   non_ret_size;    /**< The size of the array for non-retention heap */
    uint8_t   *prf_buf;         /**< Pointer to the array for profile heap */
    uint32_t   buf_size;        /**< The size of the array for profile heap */
    uint8_t   *bm_buf;          /**< Pointer to the array for bond manager heap */
    uint32_t   bm_size;         /**< The size of the array for bond manager heap */
    uint8_t   *conn_buf;        /**< Pointer to the array for connection heap */
    uint32_t   conn_size;       /**< The size of the array for connection heap */
    uint8_t   *scan_dup_filt_list_buf;  /**< Pointer to the array for adv duplicate filter */
    uint32_t   scan_dup_filt_list_size; /**< The size of the array for adv duplicate filter */
}stack_heaps_table_t;

/**@brief The function pointers for HCI UART. */
typedef struct
{
    void (*init)(void);                                                                             /**< Initialize UART. */
    void (*flow_on)(void);                                                                          /**< Flow control on. */
    bool (*flow_off)(void);                                                                         /**< Flow control off. */
    void (*finish_transfers)(void);                                                                 /**< Finish the current transferring. */
    void (*read)(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);   /**< Read data. */
    void (*write)(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);  /**< Write data. */
} hci_uart_call_t;
/** @} */

/** @addtogroup BLE_COMMEN_TYPEDEF Typedefs
 * @{
 */

/**@brief The BLE sync event callback. */
typedef void (*ble_sync_evt_cb_t)(uint32_t sync_cnt, uint16_t sync_period);
/** @} */

/** @addtogroup BLE_COMMEN_FUNCTIONS Functions
 * @{ */
/**
 *****************************************************************************************
 * @brief Initialize BLE Stack.
 *
 * @param[in] evt_handler:    Pointer to ble events handler.
 * @param[in] p_heaps_table:  Pointer to the BLE stack heaps table.
 *****************************************************************************************
 */
uint16_t ble_stack_init(ble_evt_handler_t evt_handler, stack_heaps_table_t *p_heaps_table);

/**
 *****************************************************************************************
 * @brief Initialize only BLE Stack Controller.
 *
 * @param[in] p_heaps_table:  Pointer to the BLE stack heaps table.
 *****************************************************************************************
 */
void ble_stack_controller_init(stack_heaps_table_t *p_heaps_table);

/**
 *****************************************************************************************
 * @brief Register UART instance for HCI.
 *
 * @param[in] id:  Instance index.
 * @param[in] api: Pointer to the struct of function pointers for HCI UART.
 *****************************************************************************************
 */
void ble_hci_uart_register(uint8_t id, hci_uart_call_t *api);

/**
 *****************************************************************************************
 * @brief Register BLE idle time notification callback function.
 *
 * @param[in] callback:  function pointer of BLE idle time notification function.
 * @note        param[in] of callback: hs - the idle time of BLE in half slot (312.5μs).
 *                  Callback will be called by BLE ISR to notify the rest idle time if there are some BLE activities.
 *                  It should be realized as simlpe as you can.
 *                  It's not suitable for ISO activities.
 *****************************************************************************************
 */
void ble_idle_time_notify_cb_register(void (*callback)(uint32_t hs));

/**
 *****************************************************************************************
 * @brief Register BLE activity start notification callback function.
 *
 * @param[in] callback:  function pointer of BLE activity start notification function.
 * @note            param[in] of callback: e_role - the role of activity, gap_activity_role_t for possible roles.
 *                  param[in] of callback: index - The index parameter is interpreted by role.
 *                  If role is GAP_ACTIVITY_ROLE_ADV, it's the index of Advertising.
 *                  If role is GAP_ACTIVITY_ROLE_CON, it's the index of Connection.
 *                  For all other roles, it should be ignored.
 *                  Callback will be called by BLE ISR when the BLE activity starts every time.
 *                  It should be realized as simlpe as you can.
 *                  Notice: You must define the start callback in the RAM space to avoid hardfault.
 *                  It's not suitable for ISO activities.
 *****************************************************************************************
 */
void ble_activity_start_notify_cb_register(void (*callback)(ble_gap_actv_role_t e_role,  uint8_t index));

/**
 *****************************************************************************************
 * @brief Register BLE activity end notification callback function.
 *
 * @param[in] callback:  function pointer of BLE activity end notification function.
 * @note            param[in] of callback: e_role - the role of activity,gap_activity_role_t for possible roles.
 *                  param[in] of callback: index - The index parameter is interpreted by role.
 *                  If role is GAP_ACTIVITY_ROLE_ADV, it's the index of Advertising.
 *                  If role is GAP_ACTIVITY_ROLE_CON, it's the index of Connection.
 *                  For all other roles, it should be ignored.
 *                  Callback will be called by BLE ISR when the BLE activity ends every time.
 *                  It should be realized as simlpe as you can. You'd better to define it in the RAM space
 *                  It's not suitable for ISO activities.
 *****************************************************************************************
 */
void ble_activity_end_notify_cb_register(void (*callback)(ble_gap_actv_role_t e_role,  uint8_t index));

/**
 *****************************************************************************************
 * @brief Create sync source.
 *
 * @param[in] period: Period of sync source.
 *
 * @return  SDK_ERR_DISALLOWED: Create sync source fail.
 *          SDK_SUCCESS:        Create sync source successfully.
 *****************************************************************************************
 */
uint16_t ble_sync_source_create(uint16_t period);

/**
 *****************************************************************************************
 * @brief Register sync event callback.
 *
 * @param[in] sync_evt_cb: Sync event callback.
 *
 * @return  SDK_ERR_POINTER_NULL: Pointer to sync event callback is NULL.
 *          SDK_SUCCESS:          Register successfully.
 *****************************************************************************************
 */
uint16_t ble_sync_evt_cb_register(ble_sync_evt_cb_t sync_evt_cb);

/**
 *****************************************************************************************
 * @brief Distribute sync source.
 *
 * @param[in] conn_idx: The connection index.
 *
 * @return  SDK_ERR_INVALID_CONN_IDX: Invalid connect index.
 *          SDK_ERR_DISALLOWED:       Distribute is disallowed.
 *          SDK_SUCCESS:              Distribute successfully.
 *****************************************************************************************
 */
uint16_t ble_sync_source_distribute(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Destroy sync source.
 *
 * @return SDK_SUCCESS: Destroy successfully.
 *****************************************************************************************
 */
uint16_t ble_sync_source_destroy(void);

/**
 *****************************************************************************************
 * @brief Change the RF TX mode of LP or ULP.
 *
 * @param[in] e_rf_tx_mode:    Refer to @ref ble_rf_tx_mode_t.
 *                                        BLE_RF_TX_MODE_LP_MODE: LP mode.
 *                                        BLE_RF_TX_MODE_ULP_MODE: ULP mode.
 *                                        Others: invalid mode.
 *                               
 * @note  This function should be called before BLE stack init.
 *
 * @return        SDK_SUCCESS: Successfully set Tx mode.   
 *                SDK_ERR_DISALLOWED: Failed to set Tx mode.
 *****************************************************************************************
 */
uint8_t ble_rf_tx_mode_set(ble_rf_tx_mode_t e_rf_tx_mode);

/**
 *****************************************************************************************
 * @brief  Get the RF TX mode of LP or ULP.
 *
 * @return  BLE_RF_TX_MODE_LP_MODE: LP Mode.
 *              BLE_RF_TX_MODE_ULP_MODE: ULP Mode.
 *              Others: Fail.
 *****************************************************************************************
 */
ble_rf_tx_mode_t ble_rf_tx_mode_get(void);
 
/**
 *****************************************************************************************
 * @brief Set the resistance value of the RF match circuit (unit: ohm).
 *
 * @note  This function should be called before BLE stack init.
 *
 * @param[in] e_ohm: The resistance value (ohm) of the RF match circuit according to the board.
 *                                    BLE_RF_MATCH_CIRCUIT_25OHM: 25 ohm.
 *                                    BLE_RF_MATCH_CIRCUIT_100OHM: 100 ohm.
 *                                    Others: invalid.
 *****************************************************************************************
 */
void ble_rf_match_circuit_set(ble_rf_match_circuit_t e_ohm);

/**
 *****************************************************************************************
 * @brief  Get the resistance value of the RF match circuit (unit: ohm).
 *
 * @return  The resistance value (ohm) of the RF match circuit according to the board (ohm).
 *                    BLE_RF_MATCH_CIRCUIT_25OHM: 25 ohm.
 *                    BLE_RF_MATCH_CIRCUIT_100OHM: 100 ohm.
 *                    Others: invalid.
 *****************************************************************************************
 */
ble_rf_match_circuit_t ble_rf_match_circuit_get(void);

/**
 *****************************************************************************************
 * @brief  Generate a signal carrier wave.
 *
 * @param[in] channel: 0~39 channel, 2402~2480 Mhz.
 *
 *****************************************************************************************
 */
void send_signal_carrier_wave(uint8_t channel);

/** @} */
#endif
/** @} */
/** @} */
