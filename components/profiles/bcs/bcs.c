/**
 *****************************************************************************************
 *
 * @file bcs.c
 *
 * @brief The implementation of Body Composition Service.
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

/* 
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "bcs.h"
#include "wss_db.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "app_log.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/** Body Composition Service Attributes Indexes. */
enum
{
    BCS_IDX_SVC,

    BCS_IDX_BC_FEAT_CHAR,
    BCS_IDX_BC_FEAT_VAL,

    BCS_IDX_BC_MEAS_CHAR,
    BCS_IDX_BC_MEAS_VAL,
    BCS_IDX_BC_MEAS_IND_CFG,

    BCS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Body Composition Service Measurement packet. */
typedef struct
{
    uint16_t size;                          /**< Measurement packet size. */
    uint8_t  value[BCS_MEAS_VAL_LEN_MAX];   /**< Measurement packet Value. */
} meas_packet_t;

/**@brief Body Composition Service Measurement data stream. */
typedef struct
{
    meas_packet_t meas_packets[BCS_CACHE_MEAS_NUM_MAX * NUM_PACKETS];  /**< BCS Measurement packet Value. */
    uint8_t       packet_num;                                          /**< BCS Measurement packet number. */
    uint8_t       packet_to_be_send_num;                               /**< Number of BCS Measurement packet to be send. */
} bcs_meas_data_stream_t;

/**@brief Body Composition Service environment variable. */
struct bcs_env_t
{
    bcs_init_t              bcs_init;                            /**< Body Composition Service Init Value. */    
    uint16_t                start_hdl;                           /**< Body Composition Service start handle. */
    uint16_t                meas_ind_cfg[BCS_CONNECTION_MAX];    /**< The configuration of BC Measurement Indication which is configured by the peer devices. */
    ble_gatts_create_db_t   bcs_gatts_db;                        /**< Body Composition Service attributs database. */
};

/*
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */
static uint16_t   bcs_indicate_meas_value_chunk(uint8_t conn_idx);
static void       bcs_meas_value_encoded(const bcs_meas_val_t *p_meas, uint16_t max_payload, uint8_t cache_num);
static uint16_t   packet_field_add(uint16_t flag, uint16_t value, uint16_t *p_flags, uint8_t **pp_field);
static bool       subsequent_packet_switched(uint16_t max_payload, uint8_t field_size, meas_packet_t *p_packets);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct bcs_env_t       s_bcs_env;
static bcs_meas_data_stream_t s_meas_packets;
static const uint8_t          s_bcs_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_BODY_COMPOSITION);

/**@brief Full BCS attributes descriptor which is used to add attributes into the ATT database.*/ 
static const ble_gatts_attm_desc_t bcs_attr_tab[BCS_IDX_NB] =
{
    // Body Composition Service Declaration
    [BCS_IDX_SVC]             = {BLE_ATT_DECL_SECONDARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // BC Feature Characteristic - Declaration
    [BCS_IDX_BC_FEAT_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // BC Feature Characteristic - Value
    [BCS_IDX_BC_FEAT_VAL]     = {BLE_ATT_CHAR_BODY_COMPOSITION_FEATURE,
                                 BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH),
                                 BLE_GATTS_ATT_VAL_LOC_USER,
                                 BCS_FEAT_VAL_LEN_MAX},

    // BC Measurement Characteristic - Declaration
    [BCS_IDX_BC_MEAS_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // BC Measurement Characteristic - Value
    [BCS_IDX_BC_MEAS_VAL]     = {BLE_ATT_CHAR_BODY_COMPOSITION_MEASUREMENT,
                                 BLE_GATTS_INDICATE_PERM(BLE_GATTS_UNAUTH),
                                 BLE_GATTS_ATT_VAL_LOC_USER,
                                 BCS_MEAS_VAL_LEN_MAX},
    // BC Measurement Characteristic - Client Characteristic Configuration Descriptor
    [BCS_IDX_BC_MEAS_IND_CFG] = {BLE_ATT_DESC_CLIENT_CHAR_CFG,
                                 BLE_GATTS_READ_PERM(BLE_GATTS_UNAUTH) | BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_UNAUTH),
                                 0,
                                 0},
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the read request.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *
 * @return If the request was consumed or not.
 *****************************************************************************************
 */
static void bcs_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t cfm;
    uint8_t handle = p_param->handle;
    uint8_t tab_index = prf_find_idx_by_handle(handle,
                                               s_bcs_env.start_hdl,
                                               BCS_IDX_NB,
                                               (uint8_t *)&(s_bcs_env.bcs_init.char_mask));

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;
    switch (tab_index)
    {
        case BCS_IDX_BC_FEAT_VAL:
            cfm.length = sizeof(uint32_t);
            cfm.value = (uint8_t *)&s_bcs_env.bcs_init.feature;
            break;

        case BCS_IDX_BC_MEAS_IND_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)&s_bcs_env.meas_ind_cfg[conn_idx];
            break;

        default:
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the write request.
 *
 * @return If the request was consumed or not.
 *****************************************************************************************
 */
static void bcs_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    uint16_t              handle     = p_param->handle;
    uint16_t              tab_index  = 0;
    uint16_t              cccd_value = 0;
    bcs_evt_t             event;
    ble_gatts_write_cfm_t cfm;

    tab_index = prf_find_idx_by_handle(handle,
                                       s_bcs_env.start_hdl,
                                       BCS_IDX_NB,
                                       (uint8_t *)&s_bcs_env.bcs_init.char_mask);
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    event.evt_type = BCS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case BCS_IDX_BC_MEAS_IND_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               BCS_EVT_MEAS_INDICATION_ENABLE :\
                               BCS_EVT_MEAS_INDICATION_DISABLE);
            s_bcs_env.meas_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && BCS_EVT_INVALID != event.evt_type && s_bcs_env.bcs_init.evt_handler)
    {
        s_bcs_env.bcs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the cccd recover request.
 *
 * @param[in]: conn_idx:   Connection index
 * @param[in]: handle:     The handle of cccd attribute.
 * @param[in]: cccd_value: The value of cccd attribute.
 *****************************************************************************************
 */
static void bcs_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint16_t         tab_index  = 0;
    bcs_evt_t        event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index  = prf_find_idx_by_handle(handle,
                                        s_bcs_env.start_hdl,
                                        BCS_IDX_NB,
                                        (uint8_t *)&s_bcs_env.bcs_init.char_mask);

    event.evt_type = BCS_EVT_INVALID;
    event.conn_idx = conn_idx;

    switch (tab_index)
    {
        case BCS_IDX_BC_MEAS_IND_CFG:
            event.evt_type = ((PRF_CLI_START_IND == cccd_value) ?\
                               BCS_EVT_MEAS_INDICATION_ENABLE :\
                               BCS_EVT_MEAS_INDICATION_DISABLE);
            s_bcs_env.meas_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (BCS_EVT_INVALID != event.evt_type && s_bcs_env.bcs_init.evt_handler)
    {
        s_bcs_env.bcs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx:   Connection index
 * @param[in] status:     Complete event status.
 * @param[in] p_ntf_ind:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void bcs_ntf_ind_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    bcs_evt_t  event;

    event.evt_type = BCS_EVT_INVALID;
    event.conn_idx = conn_idx;

    if (s_bcs_env.bcs_init.evt_handler && SDK_SUCCESS == status)
    {
        if (BLE_GATT_INDICATION == p_ntf_ind->type)
        {
            bcs_indicate_meas_value_chunk(conn_idx);
            event.evt_type = BCS_EVT_MEAS_INDICATION_CPLT;
        }
        s_bcs_env.bcs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Initialize the packet with mandatory data.
 *
 * @param[in]  p_meas:   Pointer to the measurement data.
 * @param[in]  p_pkt:    Pointer to the encoded packet.
 * @param[in]  pp_flags: Point to pointer of flags in encoded packet.
 * @param[in]  pp_field: Point to pointer of field in encoded packet.
 *
 * @return The size of mandatory data in encoded packet.
 *****************************************************************************************
 */
static uint16_t meas_packet_init(const bcs_meas_val_t *p_meas, meas_packet_t *p_pkt,
                                 uint16_t **pp_flags, uint8_t **pp_field)
{
    *pp_flags = (uint16_t *)p_pkt->value;
    /* the first 2 octets are reserved for flags. */
    *pp_field = &p_pkt->value[2];

    uint16_t *p_flags = *pp_flags;
    *p_flags |= (s_bcs_env.bcs_init.bcs_unit == BCS_UNIT_SI) ?
                 BCS_MEAS_FLAG_UNIT_SI : BCS_MEAS_FLAG_UNIT_IMPERIAL;

    uint16_t *p_field = (uint16_t *)*pp_field;
    *p_field = p_meas->body_fat_percentage;

    *pp_field += sizeof(p_meas->body_fat_percentage);

    return (sizeof(*p_flags) + sizeof(p_meas->body_fat_percentage));
}

/**
 *****************************************************************************************
 * @brief Add a field into a packet.
 *
 * @param[in]  flag:     The flag of value to be added.
 * @param[in]  value:    16bits value to be added.
 * @param[in]  p_flags:  Pointer to the flags in encoded packet.
 * @param[in]  pp_field: Point to pointer of the filed in encoded packet.
 *
 * @return The size of value being added.
 *****************************************************************************************
 */
static uint16_t packet_field_add(uint16_t flag, uint16_t value,
                                 uint16_t *p_flags, uint8_t **pp_field)
{
    *p_flags |= flag;

    uint16_t *p_field = (uint16_t *)*pp_field;
    *p_field = value;

    *pp_field += sizeof(value);

    return sizeof(value);
}

/**
 *****************************************************************************************
 * @brief Check if switch to subsequent packet.
 *
 * @param[in] max_payload: The max size of payload which is carried on ATT,
 *                         equals (MTU - MEASUREMENT_VAL_MAX_LEN).
 * @param[in] field_size:  The size of field to be added.
 * @param[in] p_packets:   Pointer to array of the packets where the encoded
 *                         data will be written.
 *
 * @return true for switching to subsequent packet.
 *****************************************************************************************
 */
static bool subsequent_packet_switched(uint16_t max_payload, uint8_t field_size,
                                       meas_packet_t *p_packets)
{
    if (p_packets[MEAS_PACKET_SUB].size)
    {
        return false;
    }

    meas_packet_t *p_first_pkt = &p_packets[MEAS_PACKET_FIRST];

    if (p_first_pkt->size + field_size > max_payload) 
    {
        // The Multiple packet bit is setting for the first packet.
        uint16_t *p_flags = (uint16_t *)p_first_pkt->value;
        *p_flags |= BCS_MEAS_FLAG_MUTI_PACKET;

        meas_packet_t *p_sub_pkt = &p_packets[MEAS_PACKET_SUB];

        // The Multiple packet bit is setting for the second packet.
        p_flags = (uint16_t *)p_sub_pkt->value;
        *p_flags |= BCS_MEAS_FLAG_MUTI_PACKET;

        return true;
    } 
    else 
    {
        return false;
    }
}

/**
 *****************************************************************************************
 * @brief Handle Body Composition Measurement data indication.
 *
 * @param[in] conn_idx: Connection index.
 *
 * @return Result of handle.
 *****************************************************************************************
 */
static uint16_t bcs_indicate_meas_value_chunk(uint8_t conn_idx)
{
    ble_gatts_noti_ind_t bcs_meas_ind;
    sdk_err_t            error_code;

    if (0 == s_meas_packets.packet_to_be_send_num)
    {
        s_meas_packets.packet_num            = 0;
        s_meas_packets.packet_to_be_send_num = 0;
        memset(s_meas_packets.meas_packets, 0x00, sizeof(s_meas_packets.meas_packets));

        return SDK_SUCCESS;
    }

    if (PRF_CLI_START_IND == s_bcs_env.meas_ind_cfg[conn_idx])
    {
        bcs_meas_ind.type   = BLE_GATT_INDICATION;
        bcs_meas_ind.handle = prf_find_handle_by_idx(BCS_IDX_BC_MEAS_VAL,
                                                     s_bcs_env.start_hdl,
                                                     (uint8_t *)&s_bcs_env.bcs_init.char_mask);
        bcs_meas_ind.length = s_meas_packets.meas_packets[s_meas_packets.packet_num - s_meas_packets.packet_to_be_send_num].size;
        bcs_meas_ind.value  = s_meas_packets.meas_packets[s_meas_packets.packet_num - s_meas_packets.packet_to_be_send_num].value;

        error_code = ble_gatts_noti_ind(conn_idx, &bcs_meas_ind);
    }

    if (SDK_SUCCESS == error_code)
    {
        s_meas_packets.packet_to_be_send_num--;
    }
    return error_code;
}

/**
 *****************************************************************************************
 * @brief Encode a Body Composition measurement data.
 *
 * @param[in]  p_meas:      Pointer to the measurement data which to be encoded.
 * @param[in]  max_payload: The max size of payload which is carried on ATT,
 *                          equals (MTU - MEASUREMENT_VAL_MAX_LEN).
 * @param[in]  cache_num:   The number of measurment caches.
 *****************************************************************************************
 */
static void bcs_meas_value_encoded(const bcs_meas_val_t *p_meas, uint16_t max_payload, uint8_t cache_num)
{
    uint8_t packet_num = 0;

    for (uint8_t i = 0; i < cache_num; i++)
    {
        meas_packet_t *p_pkt = &s_meas_packets.meas_packets[packet_num];
        uint16_t      *p_flags;
        uint8_t       *p_field;

        /** Put mandatory fields into packet. */
        meas_packet_init(&p_meas[i], p_pkt, &p_flags, &p_field);

        /** The following are the optional fields. */
        // Time Stamp Field.
        if ((s_bcs_env.bcs_init.feature & BCS_FEAT_TIME_STAMP) && \
             s_bcs_env.bcs_init.bcs_meas_flags.time_stamp_present) 
        {
            *p_flags |= BCS_MEAS_FLAG_DATE_TIME_PRESENT;
            p_field += prf_pack_date_time(p_field, &p_meas[i].time_stamp);
        }

        // User ID Field.
        if ((s_bcs_env.bcs_init.feature & BCS_FEAT_MULTI_USER) && \
             s_bcs_env.bcs_init.bcs_meas_flags.user_id_present)
        {
            *p_flags |= BCS_MEAS_FLAG_USER_ID_PRESENT;
            *p_field++ = p_meas[i].user_id;
        }

        if (BCS_MEAS_UNSUCCESS != p_meas[i].body_fat_percentage)
        {
            // Basal Metabolism Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_BASAL_METABOLISM) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.basal_metabolism_present) 
            {
                packet_field_add(BCS_MEAS_FLAG_BASAL_METABOLISM, p_meas[i].basal_metabolism, p_flags, &p_field);
            }

            // Muscle Percentage Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_MUSCLE_PERCENTAGE) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.muscle_percentage_present) 
            {
                packet_field_add(BCS_MEAS_FLAG_MUSCLE_PERCENTAGE, p_meas[i].muscle_percentage, p_flags, &p_field);
            }

            // Muscle Mass Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_MUSCLE_MASS) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.muscle_mass_present) 
            {
                packet_field_add(BCS_MEAS_FLAG_MUSCLE_MASS, p_meas[i].muscle_mass, p_flags, &p_field);
            }

            // Fat Free Mass Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_FAT_FREE_MASS) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.fat_free_mass_present) 
            {
                packet_field_add(BCS_MEAS_FLAG_FAT_FREE_MASS, p_meas[i].fat_free_mass, p_flags, &p_field);
            }

            p_pkt->size = p_field - p_pkt->value;

            /** If ATT_MTU=23 (max payload = 20), all above fields can be sent in one
             *  packet, but for the remaining optional fields below the current packet
             *  size shall be checked and the subsequent packet shall be used if needed.
             */
            // Soft Lean Mass Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_SOFT_LEAN_MASS) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.soft_lean_mass_present) 
            {
                if  (subsequent_packet_switched(max_payload, 2, &s_meas_packets.meas_packets[packet_num]) == true) 
                {
                    p_pkt = &s_meas_packets.meas_packets[packet_num + 1];
                    p_pkt->size = meas_packet_init(&p_meas[i], p_pkt, &p_flags, &p_field);
                }

                p_pkt->size += packet_field_add(BCS_MEAS_FLAG_SOFT_LEAN_MASS, p_meas[i].soft_lean_mass, p_flags, &p_field);
            }

            // Body Water Mass Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_BODY_WATER_MASS) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.body_water_mass_present) 
            {
                if  (subsequent_packet_switched(max_payload, 2, &s_meas_packets.meas_packets[packet_num]) == true) 
                {
                    p_pkt = &s_meas_packets.meas_packets[packet_num + 1];
                    p_pkt->size = meas_packet_init(&p_meas[i], p_pkt, &p_flags, &p_field);
                }

                p_pkt->size += packet_field_add(BCS_MEAS_FLAG_BODY_WATER_MASS, p_meas[i].body_water_mass, p_flags, &p_field);
            }

            // Impedance Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_IMPEDANCE) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.impedance_present) 
            {
                if  (subsequent_packet_switched(max_payload, 2, &s_meas_packets.meas_packets[packet_num]) == true) {
                    p_pkt = &s_meas_packets.meas_packets[packet_num + 1];
                    p_pkt->size = meas_packet_init(&p_meas[i], p_pkt, &p_flags, &p_field);
                }

                p_pkt->size += packet_field_add(BCS_MEAS_FLAG_IMPEDANCE, p_meas[i].impedance, p_flags, &p_field);
            }

            // Weight Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_WEIGHT) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.weight_present) 
            {
                if  (subsequent_packet_switched(max_payload, 2, &s_meas_packets.meas_packets[packet_num]) == true) 
                {
                    p_pkt = &s_meas_packets.meas_packets[packet_num + 1];
                    p_pkt->size = meas_packet_init(&p_meas[i], p_pkt, &p_flags, &p_field);
                }

                p_pkt->size += packet_field_add(BCS_MEAS_FLAG_WEIGHT, p_meas[i].weight, p_flags, &p_field);
            }

            // Height Field.
            if ((s_bcs_env.bcs_init.feature & BCS_FEAT_HEIGHT) && \
                 s_bcs_env.bcs_init.bcs_meas_flags.height_present) 
            {
                if  (subsequent_packet_switched(max_payload, 2, &s_meas_packets.meas_packets[packet_num]) == true) 
                {
                    p_pkt = &s_meas_packets.meas_packets[packet_num + 1];
                    p_pkt->size = meas_packet_init(&p_meas[i], p_pkt, &p_flags, &p_field);
                }

                p_pkt->size += packet_field_add(BCS_MEAS_FLAG_HEIGHT, p_meas[i].height, p_flags, &p_field);
            }
        }

        packet_num += 2;
    }
    s_meas_packets.packet_num = packet_num;
    s_meas_packets.packet_to_be_send_num = packet_num;
}

static void bcs_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            bcs_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            bcs_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            bcs_ntf_ind_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            bcs_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t bcs_measurement_send(uint8_t conn_idx, bcs_meas_val_t *p_bcs_meas_val, uint8_t cache_num)
{
    if (NULL == p_bcs_meas_val || BCS_CACHE_MEAS_NUM_MAX < cache_num)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    sdk_err_t        error_code = SDK_ERR_IND_DISABLED;

    memset(s_meas_packets.meas_packets, 0x00, sizeof(s_meas_packets.meas_packets));
    s_meas_packets.packet_num            = 0;
    s_meas_packets.packet_to_be_send_num = 0;

    if (PRF_CLI_START_IND == s_bcs_env.meas_ind_cfg[conn_idx])
    {
        bcs_meas_value_encoded(p_bcs_meas_val, BLE_ATT_MTU_DEFAULT - INDI_PAYLOAD_HEADER_LEN, cache_num);
        error_code = bcs_indicate_meas_value_chunk(conn_idx);
    }

    return error_code;
}

sdk_err_t bcs_service_init(bcs_init_t *p_bcs_init)
{
    if (NULL == p_bcs_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_bcs_env.bcs_init, p_bcs_init, sizeof(bcs_init_t));

    s_bcs_env.start_hdl = PRF_INVALID_HANDLE;
    s_bcs_env.bcs_gatts_db.shdl                 = &s_bcs_env.start_hdl;
    s_bcs_env.bcs_gatts_db.uuid                 = s_bcs_svc_uuid;
    s_bcs_env.bcs_gatts_db.attr_tab_cfg         = (uint8_t *)&(s_bcs_env.bcs_init.char_mask);
    s_bcs_env.bcs_gatts_db.max_nb_attr          = BCS_IDX_NB;
    s_bcs_env.bcs_gatts_db.srvc_perm            = BLE_GATTS_SRVC_SECONDARY_SET; 
    s_bcs_env.bcs_gatts_db.attr_tab_type        = BLE_GATTS_SERVICE_TABLE_TYPE_16;
    s_bcs_env.bcs_gatts_db.attr_tab.attr_tab_16 = bcs_attr_tab;

    return ble_gatts_prf_add(&s_bcs_env.bcs_gatts_db, bcs_ble_evt_handler);
}

uint16_t *bcs_start_handle_get(void)
{
    return &s_bcs_env.start_hdl;
}

