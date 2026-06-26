/**
 ****************************************************************************************
 *
 * @file ble_ranging.h
 *
 * @brief BLE RANGING API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2023-2025 GOODIX
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
 * @brief Definitions and prototypes for the BLE SDK interface.
 */

/**
 * @addtogroup BLE_RANGING
 * @{
 * @brief Definitions and prototypes for the ranging interface.
 */

#ifndef __BLE_RANGING_H__
#define __BLE_RANGING_H__

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "custom_config.h"
#include "ble_id.h"
#include "ble_hci_vendor.h"

/**
 @addtogroup BLE_RANGING
 @{
 @brief Definitions and prototypes for the ranging interface.
 */

/** @addtogroup BLE_RANGING_DEFINES Defines
 * @{ */

/** @brief Invalid connection index. */
#define RANGING_INVALID_CONIDX              (0xFF)

/** @brief Invalid connection handle. */
#define RANGING_INVALID_CONHDL              (0xFFFF)

/** @brief Invalid device role of LL layer. */
#define RANGING_INVALID_GAP_LL_ROLE_TYPE    (0xFF)

/** @brief Allocate a block of memory for localization ranging. */
#define RANGING_MEM_ALLOC(length)           ble_ranging_malloc(length)

/** @brief Free a block of memory for localization ranging. */
#define RANGING_MEM_FREE(param_ptr)         ble_ranging_free(param_ptr)

/**
 ****************************************************************************************
 * @brief Convenient wrapper to ble_ranging_ke_msg_alloc().
 *
 * This macro calls ble_ranging_ke_msg_alloc() and cast the returned pointer to the
 * appropriate structure. Can only be used if a parameter structure exists for this message.
 *
 * @param[in] id        Message identifier. The number of messages is limited to 0xFFFF.
 *                      The message ID is divided in two parts:
 *                      bits[15~8]: task index (no more than 255 tasks support).
 *                      bits[7~0]: message index(no more than 255 messages per task).
 * @param[in] param_str Parameter structure type, which shall be a typedef type.
 *
 * @return Pointer to the parameter member of the ke_msg.
 ****************************************************************************************
 */
#define RANGING_KE_MSG_ALLOC(id, param_str) \
        (param_str *) ble_ranging_ke_msg_alloc(id, sizeof(param_str))

typedef enum hci_msg_id hci_msg_id_t;
typedef enum sdk_msg_id sdk_msg_id_t;

/**
 * Message Identifier. The number of messages is limited to 0xFFFF.
 * The message ID is divided in two parts:
 * bits[15~8]: task index (no more than 255 tasks support)
 * bits[7~0]: message index(no more than 255 messages per task)
 */
typedef uint16_t ra_msg_id_t;

/** Task Identifier. Composed by the task type and the task index. */
typedef uint16_t ra_task_id_t;

/**
 * @brief Handler to report vendor-specific HCI event to different modules.
 *
 * @param[in] msg_id ID of the message received.
 * @param[in] opcode Vendor-specific HCI event or command code.
 * @param[in] p_evt  Pointer to the parameters of the message.
 */
typedef int (*ble_ranging_evt_handler_t)(hci_msg_id_t msg_id, uint16_t opcode, void const *p_evt);

/**
 * @brief Handler for the message sent by ble_ranging_ke_msg_send().
 *
 * @param[in] p_msg_param Pointer to the parameter member of the message.
 */
typedef void (*ble_ranging_msg_handler_t)(void const *p_msg_param);

/**
 * @brief Callback for getting the time increment of the current time relative to the baseline time.
 *
 * @param[in] base_time The baseline time. Unit: 0.01ms.
 *                      If base_time is 0, this callback returns the current timestamp.
 *
 * @return Time increment. Unit: 0.01ms.
 */
typedef uint32_t (*ble_ranging_get_time_delta_callback_t)(uint32_t base_time);

#if CFG_SNIFFER_CLK_SYNC_SUPPORT
/**
 * @brief Callback for getting local time tick.
 *
 * @return Timer ticks.
 */
typedef uint32_t (*ble_ranging_get_tick_callback_t)(void);

/**
 * @brief Callback for getting local clock frequency.
 *
 * @return Clock frequency.
 */
typedef uint32_t (*ble_ranging_get_freq_callback_t)(void);
#endif

#if CFG_CS_SUPPORT
/**
 * @brief Callback for dumping CS subevent result.
 *
 * @param[in] role CS role.
 * @param[in] subevt_idx Subevent index in a CS procedure.
 * @param[in] p_subevt Pointer to subevent result.
 *
 * @return Time increment. Unit: 0.01ms.
 */
typedef void (*ble_ranging_dump_cs_subevent_result_callback_t)(uint8_t role, bool is_local, uint8_t subevt_idx, cs_subevent_result_all_t *p_subevt);
#endif

/** @} */

/** @addtogroup BLE_RANGING_ENUMERATIONS Enumerations
 * @{ */

typedef enum
{
#if CFG_CS_SUPPORT
    RA_MODULE_ID_CS_RANGING    = 0x00U,
#endif
    RA_MODULE_ID_RSSI_RANGING  = 0x01U,
#if CFG_SNIFFER_CLK_SYNC_SUPPORT
    RA_MODULE_ID_CLK_SYNC      = 0x02U,
#endif
    RA_MODULE_ID_SNIFFER       = 0x03U,

    RA_MODULE_ID_MAX,
} ble_ranging_module_id_t;

/** @brief Device role of LL layer type. */
typedef enum
{
    RA_GAP_LL_ROLE_CENTRAL    = 0x00U, /**< Central role. */
    RA_GAP_LL_ROLE_PERIPHERAL = 0x01U, /**< Peripheral role. */
} ble_ranging_gap_ll_role_type_t;

/** @} */

/**@addtogroup BLE_RANGING_STRUCTURES Structures
 * @{ */

/**
 * @brief DK device info, which can be expanded.
 */
typedef struct hci_vs_le_set_dk_dev_info_cmd ble_dk_dev_info_t;

#if CFG_SNIFFER_CLK_SYNC_SUPPORT
/** @brief The parameters of clock sync. */
typedef struct hci_vs_le_set_ble_clk_sync_params_cmd ble_clk_sync_set_params_t;

typedef struct hci_vs_le_get_ble_clk_sync_params_cmd_cmp_evt ble_clk_sync_get_params_cmp_t;
#endif

/** @brief The parameters of sniffer management. */
typedef struct hci_vs_le_create_sniffer_link_cmd ble_sniffer_mng_create_sniffer_link_params_t;

typedef struct hci_vs_le_update_sniffer_connection_param_cmd ble_sniffer_mng_upd_conn_params_t;

typedef struct hci_vs_le_update_sniffer_channel_map_param_cmd ble_sniffer_mng_upd_ch_map_t;

typedef struct hci_vs_le_update_sniffer_phy_param_cmd ble_sniffer_mng_upd_phy_t;

typedef struct hci_vs_le_get_link_info_cmp_evt ble_sniffer_mng_get_link_info_cmp_evt_t;

typedef struct hci_vs_le_create_sniffer_cmd_cmp_evt ble_sniffer_mng_create_sniffer_cmp_evt_t;

typedef struct hci_vs_le_sniffer_disconnect_cmp_evt ble_sniffer_mng_disc_sniffer_cmp_evt_t;

typedef struct hci_vs_le_request_ble_sync_evt ble_sniffer_mng_req_sync_clock_evt_t;

typedef struct hci_vs_le_conn_parameter_notification_evt ble_sniffer_mng_upd_conn_param_noti_evt_t;

typedef struct hci_vs_le_channel_map_notification_evt ble_sniffer_mng_upd_channel_map_noti_evt_t;

typedef struct hci_vs_le_phy_notification_evt ble_sniffer_mng_upd_phy_noti_evt_t;

typedef struct le_conn_parameter_s le_conn_param_t;

typedef struct le_channel_map_parameter_s le_ch_map_param_t;

typedef struct sniffer_link_s le_sniffer_link_param_t;

/**
 * @brief RSSI sets ranging status(start or stop) command structure.
 */
typedef struct hci_vs_le_rssi_enable_ranging_cmd ble_dk_rssi_set_ranging_status_t;

/**
 * @brief RSSI sest ranging status(start or stop) complete structure.
 */
typedef struct hci_vs_le_rssi_enable_ranging_cmd_cmp_evt ble_dk_rssi_set_ranging_status_cmp_t;

/**
 * @brief RSSI sets ranging type(RFU or private AA) command structure.
 */
typedef struct hci_vs_le_rssi_update_ranging_type_cmd ble_dk_rssi_set_ranging_type_t;

/**
 * @brief RSSI updates ranging status(start or stop) complete structure.
 */
typedef struct hci_vs_basic_conhdl_cmd_cmp_evt ble_dk_rssi_set_ranging_type_cmp_t;

/**
 * @brief RSSI report info structure.
 */
typedef struct hci_vs_le_rssi_report_info_evt ble_dk_rssi_ranging_report_info_t;

/**
 * @brief Update ranging type indification structure.
 */
typedef struct hci_vs_le_rssi_update_ranging_type_notification_evt ble_dk_rssi_update_ranging_type_ind_t;

/** @brief The parameters of vendor-specific CS. */
typedef struct hci_vs_le_cs_write_sec_info_cmd ble_cs_vs_security_info_t;

typedef struct hci_vs_le_cs_write_remote_fae_table_cmd ble_cs_vs_write_remote_fae_table_t;

typedef struct hci_vs_le_cs_channel_map_ind_cmd ble_cs_vs_update_channel_map_t;

typedef struct hci_vs_le_cs_create_config_cmd ble_cs_vs_create_config_t;

typedef struct hci_vs_le_cs_remove_config_cmd ble_cs_vs_remove_config_t;

typedef struct hci_vs_le_cs_procedure_handover_en_cmd_t ble_cs_vs_procedure_enable_t;

typedef struct hci_vs_le_cs_get_sec_update_info_cmd_t ble_cs_vs_get_update_secruity_info_t;

/**
 * @brief Callback of GAP events.
 */
typedef struct
{
    void (*ble_ranging_conn_cb)(uint8_t conidx, uint16_t interval, uint16_t latency, uint16_t sup_to, uint8_t ll_role);
    void (*ble_ranging_disc_cb)(uint8_t conidx, uint8_t reason);
    void (*ble_ranging_upd_conn_param_cb)(uint8_t conidx, uint8_t status, uint16_t interval, uint16_t latency, uint16_t sup_to);
} ble_ranging_gap_cb_t;

/** @} */

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup BLE_RANGING_FUNCTION Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Register the HCI Event callback functions for each module (such as the RSSI module, Sniffer module, etc.).
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. This API needs to be called during module initialization.
 *
 * @param[in] module_id The identifier of the module.
 * @param[in] evt_handler The HCI Event callback function of the module.
 ****************************************************************************************
 */
void ble_ranging_register_evt_handler(ble_ranging_module_id_t module_id, ble_ranging_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief Set DK device information.
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. This API should be called after the Bluetooth LE protocol stack has been initialized and before the ACL connection is established.
 *
 * @param[in] p_dev_info Decide DK roles or other device information.
 ****************************************************************************************
 */
void ble_set_dk_device_info(const ble_dk_dev_info_t *p_dev_info);

#if CFG_SNIFFER_CLK_SYNC_SUPPORT
/**
 ****************************************************************************************
 * @brief Get native block and BLE clock to sync block between nodes.
 ****************************************************************************************
 */
void ble_clock_sync_get_params(void);

/**
 ****************************************************************************************
 * @brief Set clock parameters to sync BLE block to the other node.
 *
 * @param[in] p_clk_param Clock parameters.
 ****************************************************************************************
 */
void ble_clock_sync_set_params(const ble_clk_sync_set_params_t *p_clk_param);

/**
 ****************************************************************************************
 * @brief Register timestamp callback to controller in order to get local time.
 *
 * @param[in] get_tick_cb Callback to read time tick.
 * @param[in] get_freq_cb Callback to read clock frequency.
 ****************************************************************************************
 */
void ble_clock_sync_register_timestamp_callback(ble_ranging_get_tick_callback_t get_tick_cb,
                                                        ble_ranging_get_freq_callback_t get_freq_cb);
#endif

/**
 ****************************************************************************************
 * @brief Set RSSI ranging status(start or stop).
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. This API should be called after the Sniffer link has been established.
 *
 * @param[in] p_status RSSI ranging status.
 ****************************************************************************************
 */
void ble_rssi_set_ranging_status(const ble_dk_rssi_set_ranging_status_t *p_status);

/**
 ****************************************************************************************
 * @brief Set RSSI ranging type(PDU_RFU_TYPE or PRIVATE_AA_TYPE).
 *
 * @param[in] p_type RSSI ranging type.
 ****************************************************************************************
 */
void ble_rssi_set_ranging_type(const ble_dk_rssi_set_ranging_type_t *p_type);

/**
 ****************************************************************************************
 * @brief Get acl link info.
 *
 * @note 1. This API is only used for DK Central.
 *       2. This API should be called after the ACL connection has been established.
 *       3. After this API is successfully called, the callback function will be triggered by an HCI Event (HCI_VS_LE_GET_LINK_INFO_CMP_EVT_SUBCODE).
 *
 * @param[in] conidx Connection index.
 ****************************************************************************************
 */
void ble_sniffer_mng_get_acl_link_info(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Create sniffer link.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. This API should be called when the DK Anchor receives the creating sniffer link command sent by the DK Central.
 *
 * @param[in] p_params Create sniffer link parameters.
 ****************************************************************************************
 */
void ble_sniffer_mng_create_sniffer_link(ble_sniffer_mng_create_sniffer_link_params_t *p_params);

/**
 ****************************************************************************************
 * @brief Disconnect sniffer link.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. This API should be called when the DK Anchor receives the disconnecting sniffer link command sent by the DK Central.
 *
 * @param[in] sniffer_hdl Sniffer handle.
 ****************************************************************************************
 */
void ble_sniffer_mng_disc_sniffer_link(uint16_t sniffer_hdl);

/**
 ****************************************************************************************
 * @brief Update sniffer link connection parameters.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. This API should be called when the DK Anchor receives the updating connection parameters command sent by the DK Central.
 *
 * @param[in] p_params Update sniffer link connection parameters.
 ****************************************************************************************
 */
void ble_sniffer_mng_upd_conn_params(ble_sniffer_mng_upd_conn_params_t *p_params);

/**
 ****************************************************************************************
 * @brief Update sniffer link channel map.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. This API should be called when the DK Anchor receives the updating channel map command sent by the DK Central.
 *
 * @param[in] p_params Update sniffer link channel map parameters.
 ****************************************************************************************
 */
void ble_sniffer_mng_upd_ch_map(ble_sniffer_mng_upd_ch_map_t *p_params);

/**
 ****************************************************************************************
 * @brief Update sniffer link channel map.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. This API should be called when the DK Anchor receives the updating PHY parameters command sent by the DK Central.
 *
 * @param[in] p_params Update sniffer link phy parameters.
 ****************************************************************************************
 */
void ble_sniffer_mng_upd_phy(ble_sniffer_mng_upd_phy_t *p_params);

/**
 ****************************************************************************************
 * @brief Register sniffer management callback to GAP module.
 *
 * @note 1. This API is only used for DK Central.
 *
 * @param[in] p_params Pointer to callbacks' buffer.
 ****************************************************************************************
 */
void ble_sniffer_mng_register_gap_callback(ble_ranging_gap_cb_t *p_gap_cb);

/**
 ****************************************************************************************
 * @brief Register time delta callback to controller in order to get the time increment.
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. The time precision of the timer is required to be 10 microseconds.
 *
 * @param[in] get_delta_cb Callback to get the time increment of the current time relative to the baseline time.
 ****************************************************************************************
 */
void ble_sniffer_mng_register_time_delta_callback(ble_ranging_get_time_delta_callback_t get_delta_cb);

/**
 ****************************************************************************************
 * @brief Change the Link Layer parameter of a connection.
 *
 * @note 1. This API is only used for DK Central.
 *       2. This API should be called when the DK Central actively updates the connection parameters.
 *
 * @param[in] conidx   ACL connection index.
 * @param[in] interval Connection interval, unit: 1.25 ms, time range: 7.5 ms to 4 s.
 * @param[in] latency  Peripheral latency for the connection in number of connection events.
 * @param[in] timeout  Supervision timeout for the LE link.
 * @param[in] ce_len   The length of connection event needed for this LE connection.
 ****************************************************************************************
 */
void ble_ranging_update_gap_conn_param(uint8_t conidx, uint16_t interval, uint16_t latency, uint16_t timeout, uint16_t ce_len);

/**
 ****************************************************************************************
 * @brief Handle memory allocation for localization ranging.
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. This API should be called before ble_ranging_malloc() is used.
 *       3. The application project needs to define the global macro CFG_LOC_RANGING_SUPPORT as 1.
 *
 * @param[in] p_ranging_heap_buf Pointer to ranging heap buffer.
 * @param[in] ranging_heap_size  Ranging heap size.
 * @return None.
 ****************************************************************************************
 */
void ble_ranging_heap_init(uint8_t *p_ranging_heap_buf, uint32_t ranging_heap_size);

/**
 ****************************************************************************************
 * @brief Allocation of a block of memory for localization ranging.
 *
 * @note 1. Allocate a memory block whose size is size; if no memory is available, return NULL.
 *       2. This API can be used for both DK Central and DK Anchor.
 *       3. This API should be called after ble_ranging_heap_init() is used.
 *
 * @param[in] size Size of the memory area that need to be allocated.
 *
 * @return A pointer to the allocated memory area.
 *
 ****************************************************************************************
 */
void *ble_ranging_malloc(uint32_t size);

/**
 ****************************************************************************************
 * @brief Freeing of a block of memory.
 *
 * @note 1. Free the memory area pointed by p_mem: mark the block as free and insert it in the pool of free block.
 *       2. This API can be used for both DK Central and DK Anchor.
 *       3. This API should be called after ble_ranging_malloc() is used.
 *
 * @param[in] p_mem Pointer to the memory area that need to be freed.
 *
 ****************************************************************************************
 */
void ble_ranging_free(void *p_mem);

/**
 ****************************************************************************************
 * @brief Allocate memory for a message which will be sent to SDK TASK.
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. This API should be called after ble_ranging_heap_init() is used.
 *       3. After sending a message using ble_ranging_ke_msg_send(), the allocated memory will be automatically released by the kernel.
 *       4. If memory allocation fails, an ASSERT will be triggered directly.
 *
 * @param[in] id        Message identifier
 * @param[in] param_len Size of the message parameters to be allocated
 *
 * @return Pointer to the parameter member of the ke_msg. If the parameter
 *         structure is empty, the pointer will point to the end of the message
 *         and should not be used (except to retrieve the message pointer or to
 *         send the message)
 ****************************************************************************************
 */
void *ble_ranging_ke_msg_alloc(sdk_msg_id_t id, uint16_t param_len);

/**
 ****************************************************************************************
 * @brief Send message to SDK TASK.
 *
 * @note 1. This API can be used for both DK Central and DK Anchor.
 *       2. This API should be called after ble_ranging_ke_msg_alloc() is used.
 *       3. After calling this API, you should no longer access the data pointed to p_msg_param,
 *          as the memory allocated by ble_ranging_ke_msg_alloc will be automatically released by the kernel.
 *
 * @param[in] p_msg_param  Pointer to the parameter member of the message that should be sent.
 ****************************************************************************************
 */
void ble_ranging_ke_msg_send(void const *p_msg_param);

/**
 ****************************************************************************************
 * @brief Retrieve connection index from connection handle.
 *
 * @note 1. This API is only used for DK Central.
 *       2. This API should be called after the ACL connection has been established.
 *
 * @param[in] conhdl Connection handle.
 *
 * @return Return found connection index, RANGING_INVALID_CONIDX if not found.
 ****************************************************************************************
 */
uint8_t ble_ranging_get_conidx_by_conhdl(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief Retrieve connection handle from connection index.
 *
 * @note 1. This API is only used for DK Central.
 *       2. This API should be called after the ACL connection has been established.
 *
 * @param[in] conidx Connection index.
 *
 * @return Return found connection handle, RANGING_INVALID_CONHDL if not found.
 ****************************************************************************************
 */
uint16_t ble_ranging_get_conhdl_by_conidx(uint8_t conidx);

#if CFG_CS_VS_SUPPORT
/**
 ****************************************************************************************
 * @brief Set vendor-specific CS node number.
 *
 * @param[in] node_num Number of vendor-specific CS node.
 ****************************************************************************************
 */
void ble_cs_vs_set_node_num(uint8_t node_num);

/**
 ****************************************************************************************
 * @brief Write CS security information.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. The API should be called after CS security information is received from DK Central,
 *          and before CS procedure is enabled on DK Anchor.
 *
 * @param[in] p_params CS security information parameters.
 ****************************************************************************************
 */
void ble_cs_vs_write_security_info(ble_cs_vs_security_info_t *p_params);

/**
 ****************************************************************************************
 * @brief Write remote CS FAE table.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. The API should be called after CS FAE table is received from DK Central,
 *          and before CS procedure is enabled on DK Anchor.
 *
 * @param[in] p_params Remote CS FAE table parameters.
 ****************************************************************************************
 */
void ble_cs_vs_write_remote_fae_tab(ble_cs_vs_write_remote_fae_table_t *p_params);

/**
 ****************************************************************************************
 * @brief Update CS channel classification.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. The API should be called after CS channel classification is received from DK Central.
 *
 * @param[in] p_params CS channel classification parameters.
 ****************************************************************************************
 */
void ble_cs_vs_update_channel_map(ble_cs_vs_update_channel_map_t *p_params);

/**
 ****************************************************************************************
 * @brief Create CS configuration.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. The API should be called after CS configuration is received from DK Central,
 *          and before CS procedure is enabled on DK Anchor.
 *
 * @param[in] p_params CS configuration parameters.
 ****************************************************************************************
 */
void ble_cs_vs_create_config_req(ble_cs_vs_create_config_t *p_params);

/**
 ****************************************************************************************
 * @brief Remove CS configuration.
 *
 * @note 1. This API is only used for DK Anchor.
 *       2. The API should be called after CS configuration has been created on DK Anchor.
 *
 * @param[in] sniffer_hdl Sniffer handle.
 * @param[in] config_id CS configuration ID.
 ****************************************************************************************
 */
void ble_cs_vs_remove_config(uint16_t sniffer_hdl, uint8_t config_id);

/**
 ****************************************************************************************
 * @brief Enable CS procedure based on arrangement.
 *
 * @note 1. This API can be used for DK Anchor and DK Central.
 *       2. The API should be called after standard CS procedure has been enabled on DK Central.
 *       3. For DK Anchor, the API should be called after CS enabling paramters are received from DK Central,
            and CS security information has been written, CS configuration has been created.
 *
 * @param[in] p_params CS procedure parameters to enable.
 ****************************************************************************************
 */
void ble_cs_vs_procedure_enable(ble_cs_vs_procedure_enable_t *p_params);

/**
 ****************************************************************************************
 * @brief Disable CS procedure.
 *
 * @note 1. This API can be used for DK Anchor and DK Central.
 *       2. The API should be called after CS procedure has been enabled.
 *
 * @param[in] handle Connection handle on DK Central or sniffer handle on DK Anchor.
 * @param[in] config_id CS configuration ID.
 ****************************************************************************************
 */
void ble_cs_vs_procedure_disable(uint16_t handle, uint8_t config_id);

/**
 ****************************************************************************************
 * @brief Get CS security information.
 *
 * @note 1. This API is only used for DK Central.
 *       2. The API should be called after CS security has been enabled. And then HCI_VS_LE_CS_SECURITY_UPDATE_CMP_EVT_SUBCODE
 *          will be reported.
 *
 * @param[in] con_hdl Connection handle.
 ****************************************************************************************
 */
void ble_cs_vs_get_security_info(uint16_t con_hdl);


/**
 ****************************************************************************************
 * @brief Notify standard CS module that the vendor CS procedure is finished because the procedure counter is a finite value.
 *
 * @note 1. This API is only used for DK Central.
 *       2. The API should be called after CS procedure is finished when procedure_count reaches the value as procedure configuration.
 *
 * @param[in] conidx Connection index.
 * @param[in] config_id CS config_ID.
 ****************************************************************************************
 */
void ble_cs_vs_notify_procedure_auto_disable_state(uint8_t conidx, uint8_t config_id);
#endif

#ifdef SOC_SGP
/**
 ****************************************************************************************
 * @brief Set whether data transmission is considered when scheduling CS.
 *
 * @param[in] value True or false.
 ****************************************************************************************
 */
void ble_cs_set_sch_with_data_transmission(bool value);
#endif

/**
  * @brief Allocate cache memory for Channel Sounding subevent results.
  * @note This function automatically selects between calibration buffers and normal RAS buffer pool.
  *       For calibration configurations, dedicated buffers are used. For normal ranging, the RAS
  *       buffer pool allocation is used with length validation.
  *
  * @param[in] conidx      Connection index.
  * @param[in] config_id   CS configuration ID.
  * @param[in] is_local    Local (1) or remote (0) subevent results.
  * @param[in] pro_cnt     Procedure counter.
  * @param[in] len         Required buffer length in bytes.
  *
  * @return Pointer to allocated buffer, or NULL if allocation failed.
  */
void *cs_malloc_cache_mem(uint8_t conidx, uint8_t config_id, uint8_t is_local, uint16_t pro_cnt, uint32_t len);

/**
  * @brief Free cache memory allocated by cs_malloc_cache_mem.
  * @note Calibration buffers are not freed and remain persistent. Only normal RAS buffer pool
  *       allocations are released.
  *
  * @param[in] p_mem       Pointer to memory to free.
  *
  * @return None.
  */
void cs_free_cache_mem(void *p_mem);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
/** @} */

/** @} */
/** @} */

