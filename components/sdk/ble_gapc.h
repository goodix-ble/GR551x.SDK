/**
 ****************************************************************************************
 *
 * @file ble_gapc.h
 *
 * @brief BLE GAPC API
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
 * @brief Definitions and prototypes for the BLE SDK interface.
 */
 
  /**
 * @addtogroup BLE_GAP Generic Access Profile (GAP)
 * @{
 * @brief Definitions and prototypes for the GAP interface.
 */
 
 /**
 * @defgroup BLE_GAPC Generic Access Profile (GAP) Connection Control
 * @{
 * @brief Definitions and prototypes for the GAP Connection Control interface.
 */
#ifndef __BLE_GAPC_H__
#define __BLE_GAPC_H__

#include "ble_error.h"
#include "gr55xx_sys_cfg.h"
#include <stdint.h>         // Standard Integer
#include <string.h>
#include <stdbool.h>

/**
 * @defgroup  BLE_GAPC_DEFINES Defines
 * @{
 */
#define BLE_GAP_CHNL_MAP_LEN         0x05       /**< The length of channel map. */
#define BLE_GAP_FEATS_LEN            0x08       /**< The length of features. */
#define BLE_GAP_ADDR_LEN             0x06       /**< The length of address. */
#define BLE_GAP_INVALID_CONN_INDEX   0xFF       /**< Invalid connection index. */

/**@defgroup BLE_GAP_ADDR_TYPES GAP Address types
 * @{ */
#define BLE_GAP_ADDR_TYPE_PUBLIC            0x00        /**< Public (identity) address.*/
#define BLE_GAP_ADDR_TYPE_RANDOM_STATIC     0x01        /**< Random static (identity) address. */
/**@} */

/**@defgroup BLE_GAP_PHY_OPTIONS GAP PHY OPTIONS (bitmask)
 * @{ */
#define BLE_GAP_PHY_OPT_NO_CODING    0x00       /**< The Host has no preferred coding when transmitting on the LE Coded PHY. */
#define BLE_GAP_PHY_OPT_S2_CODING    0x01       /**< The Host prefers that S=2 coding be used when transmitting on the LE Coded PHY. */
#define BLE_GAP_PHY_OPT_S8_CODING    0x02       /**< The Host prefers that S=8 coding be used when transmitting on the LE Coded PHY. */
/**@} */

/** @} */

/**
 * @defgroup BLE_GAPC_ENUM Enumerations
 * @{
 */
/** @brief The operation code used to get connection info */
typedef enum  
{
    BLE_GAP_GET_CON_RSSI = 0,        /**< Get connection RSSI info. */
    BLE_GAP_GET_CON_CHANNEL_MAP,     /**< Get connection channel map. */
    BLE_GAP_GET_PHY,                 /**< Get connection PHY. */
    BLE_GAP_GET_CHAN_SEL_ALGO        /**< Get selection algorithm for connection channel. */
} ble_gap_get_conn_info_op_t;

/**@brief The operation code used to get peer device info. */
typedef enum 
{
    BLE_GAP_GET_PEER_VERSION = 0,    /**< Get peer device version info. */
    BLE_GAP_GET_PEER_FEATURES        /**< Get peer device features info. */
} ble_gap_get_peer_info_op_t;

/** @brief Device role of LL layer type */
typedef enum
{
    BLE_GAP_LL_ROLE_MASTER = 0,                  /**< Master role. */
    BLE_GAP_LL_ROLE_SLAVE  = 1,                  /**< Slave role. */
} ble_gap_ll_role_type_t;

/**
 * @brief Operation code used to set param(s).
 */
typedef enum
{
    BLE_GAP_OPCODE_CHNL_MAP_SET,            /**< Set Channel Map. */
    BLE_GAP_OPCODE_WHITELIST_SET,           /**< Set white list. */
    BLE_GAP_OPCODE_PER_ADV_LIST_SET,        /**< Set periodic advertising list. */
    BLE_GAP_OPCODE_PRIVACY_MODE_SET,        /**< Set privacy mode for peer device. */
} ble_gap_param_set_op_id_t;

/**
 * @brief Operation code used for LEPSM manager.
 */
typedef enum
{
    BLE_GAP_OPCODE_LEPSM_REGISTER,      /**< LEPSM register operation. */
    BLE_GAP_OPCODE_LEPSM_UNREGISTER,    /**< LEPSM unregister operation. */
} ble_gap_psm_manager_op_id_t;

/**
 * @brief The specified reason for terminating a connection.
 */
typedef enum
{
    BLE_GAP_HCI_AUTHENTICATION_FAILURE                          = 0x05, /**< Authentication Failure. */
    BLE_GAP_HCI_REMOTE_USER_TERMINATED_CONNECTION               = 0x13, /**< Remote User Terminated Connection. */
    BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES     = 0x14, /**< Remote Device Terminated Connection due to Low Resources . */
    BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF         = 0x15, /**< Remote Device Terminated Connection due to Power Off. */
    BLE_GAP_HCI_UNSUPPORTED_REMOTE_FEATURE                      = 0x1A, /**< Unsupported Remote Feature. */
    BLE_GAP_HCI_PAIRING_WITH_UNIT_KEY_UNSUPPORTED               = 0X29, /**< Pairing With Unit Key Not Supported. */
    BLE_GAP_HCI_CONN_INTERVAL_UNACCEPTABLE                      = 0x3B, /**< Unacceptable Connection Parameters. */
} ble_gap_disconn_reason_t;

/** @} */

/**
 * @defgroup BLE_GAPC_STRUCT Structures
 * @{
 */

/** @brief The struct of device version. */
typedef struct
{
    uint8_t  hci_ver;       /**< HCI version. */
    uint8_t  lmp_ver;       /**< LMP version. */
    uint8_t  host_ver;      /**< Host version. */
    uint16_t hci_subver;    /**< HCI subversion. */
    uint16_t lmp_subver;    /**< LMP subversion. */
    uint16_t host_subver;   /**< Host subversion. */
    uint16_t manuf_name;    /**< Manufacturer name. */
} ble_gap_dev_version_ind_t;

/** @brief The struct of address. */
typedef struct
{
    uint8_t  addr[BLE_GAP_ADDR_LEN];    /**< 6-byte array address value. */
} ble_gap_addr_t;

/** @brief The struct of broadcast address with broadcast type. */
typedef struct
{
    ble_gap_addr_t gap_addr;     /**< Device BD Address. */
    uint8_t        addr_type;    /**< Address type of the device: 0=public/1=random. please @ref BLE_GAP_ADDR_TYPES. */
} ble_gap_bdaddr_t;

/** @brief Get broadcast address struct. */
typedef struct
{
    uint8_t          index;         /**< Advertsing index. The valid range is: 0 - 4. */
    ble_gap_bdaddr_t bd_addr;       /**< BD address. */
} ble_gap_get_bd_addr_t;

/** @brief TX power info struct. */
typedef struct
{
    int8_t     power_lvl;       /**< Advertising channel TX power level. Range: -20 to 10. Unit: dBm. Accuracy: +/-4dB. */
} ble_gap_dev_adv_tx_power_t;

/** @brief TX power info struct. */
typedef struct
{
    int8_t min_tx_pwr;      /**< MIN of TX power. Size: 1 octet (signed integer). Range: -127  to +126. Unit: dBm. */
    int8_t max_tx_pwr;      /**< MAX of TX power. Size: 1 octet (signed integer). Range: -127 to +126. Unit: dBm. */
} ble_gap_dev_tx_power_t;

/** @brief Max data length info struct. */
typedef struct
{
    uint16_t suppted_max_tx_octets;     /**< Maximum number of payload octets that the local Controller supports for transmission of a single Link Layer packet on a data connection.
                                             Range: 0x001B-0x00FB (all other values reserved for future use). */
    uint16_t suppted_max_tx_time;       /**< Maximum time, in microseconds, that the local Controller supports for transmission of a single Link Layer packet on a data connection.
                                             Range: 0x0148-0x4290 (all other values reserved for future use). */
    uint16_t suppted_max_rx_octets;     /**< Maximum number of payload octets that the local Controller supports for reception of a single Link Layer packet on a data connection.
                                             Range: 0x001B-0x00FB (all other values reserved for future use). */
    uint16_t suppted_max_rx_time;       /**< Maximum time, in microseconds, that the local Controller supports for reception of a single Link Layer packet on a data connection.
                                             Range: 0x0148-0x4290 (all other values reserved for future use). */
} ble_gap_max_data_len_t;

/** @brief Suggested default data length info. */
typedef struct
{
    uint16_t suggted_max_tx_octets; /**< The Host's suggested value for the Controller's maximum transmitted number of payload octets to be used for new connections.
                                         Range: 0x001B-0x00FB (all other values reserved for future use), default: 0x001B */
    uint16_t suggted_max_tx_time;   /**< The Host's suggested value for the Controller's maximum packet transmission time to be used for new connections.
                                         Range: 0x0148-0x4290 (all other values reserved for future use), default: 0x0148*/
} ble_gap_sugg_dflt_data_len_t;

/** @brief Number of available advertising sets info. */
typedef struct
{
    uint8_t nb_adv_sets;        /**< Number of available advertising sets. */
} ble_gap_nb_adv_sets_t;

/** @brief Maximum advertising data length info. */
typedef struct
{
    uint16_t length;            /**< Maximum advertising data length supported by controller. */
} ble_gap_max_adv_data_len_ind_t;

/** @brief RF path compensation values info. */
typedef struct
{
    uint16_t tx_path_comp;      /**< RF TX path compensation. */
    uint16_t rx_path_comp;      /**< RF RX path compensation. */
} ble_gap_dev_rf_path_comp_ind_t;

/** @brief Device info. */
typedef union
{
    ble_gap_dev_version_ind_t       dev_version;            /**< Version info. */
    ble_gap_get_bd_addr_t           get_bd_addr;            /**< Device BD address info. */   
    ble_gap_dev_adv_tx_power_t      adv_tx_power;           /**< Advertising TX power info. */
    ble_gap_sugg_dflt_data_len_t    sugg_dflt_data_len;     /**< Suggested default data length info. */
    ble_gap_max_data_len_t          max_data_len;           /**< Suggested  MAX data length info. */
    ble_gap_nb_adv_sets_t           nb_adv_sets;            /**< Number of available advertising sets. */
    ble_gap_max_adv_data_len_ind_t  max_adv_data_len;       /**< Maximum advertising data length info. */
    ble_gap_dev_tx_power_t          dev_tx_power;           /**< Device TX power info. */
    ble_gap_dev_rf_path_comp_ind_t  dev_rf_path_comp;       /**< RF path compensation values. */
} ble_gap_dev_info_t;

/** @brief The parameter of connection. */
typedef  struct
{
     uint16_t interval_min;     /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                     Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;     /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                     Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency;    /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;      /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_conn_param_t;

/** @brief The parameter of update connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. range: 0x000A to 0x0C80, unit: 10 ms, Time range: 100 ms to 32 s. */
     uint16_t ce_len;        /**< The length of connection event needed for this LE connection. Range: 0x0002 to 0xFFFF, unit: 0.625 ms, time Range: 1.25 ms to 40.9 s.
                                  recommended value: 0x0002 for 1M phy, 0x0006 for coded phy */
} ble_gap_conn_update_param_t;

/** @brief  Channel map structure. */
typedef struct
{
    uint8_t map[BLE_GAP_CHNL_MAP_LEN]; /**< This parameter contains 37 1-bit fields. The nth bit (n is in the range of 0 to 36) contains the value for the link layer channel index n.
                                        Channel n is unused = 0, channel n is used = 1. The most significant bits are reserved for future use.*/
} ble_gap_chnl_map_t;

/** @brief PHY info. */
typedef struct
{
    uint8_t tx_phy;         /**< LE PHY for data transmission. @ref BLE_GAP_PHY_OPTIONS. */
    uint8_t rx_phy;         /**< LE PHY for data reception. @ref BLE_GAP_PHY_OPTIONS. */
} ble_gap_le_phy_ind_t;

/** @brief Connection info. */
typedef union
{
    int8_t               rssi;              /**< RSSI. */
    ble_gap_chnl_map_t   chnl_map;          /**< channel map. */
    ble_gap_le_phy_ind_t phy;               /**< PHY indicaiton. */
    uint8_t              chan_sel_algo;     /**< Chanel Selection algorithm, 0x00: LE Channel Selection Algorithm #1 is used.
                                             0x01: LE Channel Selection Algorithm #2 is used.\n 0x02-0xFF: reserved. */
} ble_gap_conn_info_t;

/** @brief Peer version info. */
typedef struct 
{
    uint16_t compid;        /**<Manufacturer name. */
    uint16_t lmp_subvers;   /**< LMP subversion. */
    uint8_t  lmp_vers;      /**< LMP version. */
} ble_gap_peer_version_ind_t;

/** @brief LE features info. */
typedef struct
{
    uint8_t features[BLE_GAP_FEATS_LEN]; /**< 8-byte array for LE features\n 
                                          Feature Setting field's bit mapping to Controller Features (0: not support, 1: support) \n
                                                          |Bit position       | Link Layer Feature|
                                                          |-------------|-----------------|
                                                          |0                    | LE Encryption|
                                                          |1                    |Connection Parameters Request Procedure| 
                                                          |2                    |Extended Reject Indication|
                                                          |3                    | Slave-initiated Features Exchange | 
                                                          |4                    |LE Ping | 
                                                          |5                    |LE Data Packet Length Extension | 
                                                          |6                    |LL Privacy |  
                                                          |7                    |Extended Scanner Filter Policies | 
                                                          |8                    |LE 2M PHY|  
                                                          |9                    | Stable Modulation Index - Transmitter | 
                                                          |10                   | Stable Modulation Index - Receiver |
                                                          |11                   |LE Coded PHY | 
                                                          |12                   |LE Extended Advertising| 
                                                          |13                   | LE Periodic Advertising| 
                                                          |14                   | Channel Selection Algorithm #2| 
                                                          |15                    |LE Power Class 1|
                                                          |16                  |Minimum Number of Used Channels Procedure|  
                                                          |All other values |Reserved for Future Use|*/
} ble_gap_peer_features_ind_t;

/** @brief LE peer info. */
typedef union
{
    ble_gap_peer_version_ind_t  peer_version;   /**< Version info. */
    ble_gap_peer_features_ind_t peer_features;  /**< Features info. */
} ble_gap_peer_info_t;

/**@brief The Structure for BLE Connection Arrangement. */
typedef struct
{
    uint16_t    conn_idx;     /**< Connection Index. */
    uint32_t    interval;     /**< Connection Interval (in 625us). */
    uint32_t    offset;       /**< Connection Offset (in 625us). */
    uint32_t    duration;     /**< Connection Duration (in 625us). */
} ble_gap_con_plan_tag_t;

/** @brief Set preference slave event duration */
typedef struct
{
    uint16_t    duration;       /**< Preferred event duration. */
    uint8_t     single_tx;      /**< Slave transmits a single packet per connection event (False/True). */
} ble_gap_set_pref_slave_evt_dur_param_t;

/**@brief PHY update event for @ref BLE_GAPC_EVT_PHY_UPDATED. */
typedef struct
{
    uint8_t     tx_phy;         /**< LE PHY for data transmission. @ref BLE_GAP_PHY_OPTIONS. */
    uint8_t     rx_phy;         /**< LE PHY for data reception. @ref BLE_GAP_PHY_OPTIONS. */
} ble_gap_evt_phy_update_t;

/** @brief Get device info event  for BLE_GAPC_EVT_DEV_INFO_GOT. */
typedef struct
{
    uint8_t               operation;      /**< Operation code. @ref ble_gap_dev_info_get_type_t. */
    ble_gap_dev_info_t    dev_info;       /**< Device info. */
 } ble_gap_evt_dev_info_get_t;

/** @brief  Connection complete event for @ref BLE_GAPC_EVT_CONNECTED. */
typedef struct
{
    uint16_t                conn_index;             /**< Connection index. Range: 0x0000-0x0EFF (all other values reserved for future use). */
    uint16_t                conn_interval;          /**< Connection interval. Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s. */
    uint16_t                slave_latency;          /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t                sup_timeout;            /**< Connection supervision timeout. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
    uint8_t                 clk_accuracy;           /**< Clock accuracy (0x00: 500 ppm, 0x01: 250 ppm, 0x02: 150 ppm, 0x03: 100 ppm, 0x04: 75 ppm, 0x05:50 ppm, 0x06:30 ppm, 0x07:20 ppm, others: reserved for future use). */
    uint8_t                 peer_addr_type;         /**< Peer address type(0x00: Public Device Address, 0x01 : Random Device Address, others: reserved for future use). */
    ble_gap_addr_t          peer_addr;              /**< Peer BT address. */
    ble_gap_ll_role_type_t  ll_role;                /**< Device Role of LL Layer. */
} ble_gap_evt_connected_t;

/**@brief Disconnection event for @ref BLE_GAPC_EVT_DISCONNECTED. */
typedef struct
{
    uint8_t reason;         /**< HCI error code, see BLE_HCI_STATUS_CODES. */
} ble_gap_evt_disconnected_t;

/** @brief  Name of peer device indication event for @ref BLE_GAPC_EVT_PEER_NAME_GOT. */
typedef struct
{
    ble_gap_addr_t  peer_addr;              /**< Peer device bd address. */
    uint8_t         addr_type;              /**< Peer device address type. */
    uint8_t         name_len;               /**< Peer device name length. */
    uint8_t        *name;                   /**< Peer device name. */
} ble_gap_evt_peer_name_get_t;

/** @brief Get peer info event for @ref BLE_GAPC_EVT_PEER_INFO_GOT. */
typedef struct
{
    uint8_t             opcode;         /**< Operation code. See @ref ble_gap_get_peer_info_op_t. */
    ble_gap_peer_info_t peer_info;      /**< Peer info. */
} ble_gap_evt_peer_info_t;

/** @brief Connection parameter updated event for @ref BLE_GAPC_EVT_CONN_PARAM_UPDATED. */
typedef struct
{
    uint16_t conn_interval;             /**< Connection interval. Range: 0x0006 to 0x0C80. Unit: 1.25 ms. Time range: 7.5 ms to 4 s. */
    uint16_t slave_latency;             /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t sup_timeout;               /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_evt_conn_param_updated_t;

/** @brief Connection parameter update request event for @ref BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_evt_conn_param_update_req_t;

/** @brief Get Connection info event for @ref BLE_GAPC_EVT_CONN_INFO_GOT. */
typedef struct
{
    uint8_t         opcode;     /**< Operation code. See @ref ble_gap_get_conn_info_op_t. */
    ble_gap_conn_info_t info;       /**< Connection info. */
} ble_gap_evt_conn_info_t;

/** @brief Data Length Updated event for @ref BLE_GAPC_EVT_DATA_LENGTH_UPDATED. */
typedef struct
{
    uint16_t max_tx_octets; /**<  The maximum number of payload octets in TX. */
    uint16_t max_tx_time;   /**<  The maximum time that the local Controller will take to TX. */
    uint16_t max_rx_octets; /**<  The maximum number of payload octets in RX. */
    uint16_t max_rx_time;   /**<  The maximum time that the local Controller will take to RX. */
} ble_gap_evt_data_length_t;

/**@brief BLE GAPC event structure. */
typedef struct
{
    uint8_t  index;                                                     /**< Index of connection. */
    union                                                               /**< union alternative identified by evt_id in enclosing struct. */
    {
        ble_gap_evt_phy_update_t                phy_update;             /**< PHY update parameters. */
        ble_gap_evt_connected_t                 connected;              /**< Connection parameters. */
        ble_gap_evt_disconnected_t              disconnected;           /**< Disconnection parameters. See @ref BLE_STACK_ERROR_CODES. */
        ble_gap_evt_peer_name_get_t             peer_name;              /**< Peer device name indication parameters. */
        ble_gap_evt_peer_info_t                 peer_info;              /**< Peer info indication parameters. */
        ble_gap_evt_conn_param_updated_t        conn_param_updated;     /**< Connection parameter updated parameters. */
        ble_gap_evt_conn_param_update_req_t     conn_param_update_req;  /**< Connection parameter update request parameters. */
        ble_gap_evt_conn_info_t                 conn_info;              /**< Connection info parameters. */
        ble_gap_evt_data_length_t               data_length;            /**< Data Length Update parameter. */
    } params;                                                           /**< Event Parameters. */
} ble_gapc_evt_t;

/** @} */

/**
 * @defgroup BLE_GAPC_FUNCTION Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Terminate an existing connection.
 *
 * @param[in] conn_idx: The index of connection.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief Terminate an existing connection with a specified reason.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] reason:   The specified reason.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect_with_reason(uint8_t conn_idx, ble_gap_disconn_reason_t reason);

/**
 ****************************************************************************************
 * @brief Change the Link Layer connection parameters of a connection. 
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] p_conn_param: Pointer to the new connection param.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update (uint8_t conn_idx, const ble_gap_conn_update_param_t *p_conn_param);

/**
 *****************************************************************************************
 * @brief Set connection's Latency.
 * @note  The latency shall be set to X value by LLCP firstly, then uses this API to change the latency in [0, X].
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] latency:      The latency of connection.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_latency_set(uint8_t conn_idx, uint16_t latency);

/**
 *****************************************************************************************
 * @brief Consult BLE connection activity plan situation function.
 * @note  This function should be called when connection established and no periodic advertising exists.
 *
 * @param[out] p_act_num:       Pointer to the number of existing connection activities.
 * @param[out] p_conn_plan_arr: Pointer to the global array that stores planned connection activities.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_con_plan_consult(uint8_t *p_act_num, ble_gap_con_plan_tag_t **p_conn_plan_arr);

/**
 ****************************************************************************************
 * @brief Connection param update reply to peer device.
 *
 * @param[in] conn_idx:      The index of connection.
 * @param[in] accept:        True to accept connection parameters, false to reject.
 *
 * @retval ::SDK_SUCCESS: Operation is success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update_reply(uint8_t conn_idx, bool accept);

/**
 ****************************************************************************************
 * @brief The suggested maximum transmission packet size and maximum packet transmission time to be used for a given connection.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_octects: Preferred maximum number of payload octets that the local Controller should include in a single Link Layer packet on this connection.
 *                        Range 0x001B-0x00FB (all other values reserved for future use).
 * @param[in] tx_time:    Preferred maximum number of microseconds that the local Controller should use to transmit a single Link Layer packet on this connection.
 *                        Range 0x0148-0x4290 (all other values reserved for future use).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_data_length_update(uint8_t conn_idx, uint16_t tx_octects , uint16_t tx_time);

/**
 ****************************************************************************************
 * @brief Set the PHY preferences for the connection identified by the connection index.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_phys:    The transmitter PHYs that the Host prefers the Controller to use (see @ref BLE_GAP_PHY_OPTIONS).
 * @param[in] rx_phys:    A bit field that indicates the receiver PHYs that the Host prefers the Controller to use (see @ref BLE_GAP_PHY_OPTIONS).
 * @param[in] phy_opt:    A bit field that allows the Host to specify options for PHYs (see @ref BLE_GAP_PHY_OPTIONS).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_phy_update(uint8_t conn_idx, uint8_t tx_phys , uint8_t rx_phys, uint8_t phy_opt);

/**
 ****************************************************************************************
 * @brief Get the information of the connection.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref ble_gap_get_conn_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_info_get(uint8_t conn_idx, ble_gap_get_conn_info_op_t opcode);

/**
 ****************************************************************************************
 * @brief Get the information of the peer device.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref ble_gap_get_peer_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_peer_info_get(uint8_t conn_idx, ble_gap_get_peer_info_op_t opcode);

/** @} */

#endif
/** @} */
/** @} */
/** @} */
