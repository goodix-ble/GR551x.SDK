#ifndef __PATCH_TAB_H_
#define __PATCH_TAB_H_

typedef uint16_t ke_task_id_t;
typedef uint16_t ke_msg_id_t;

typedef int (*ke_msg_func_t)(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

typedef int (*llm_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);

typedef int (*gapm_hci_evt_hdl_func_t)(uint16_t opcode, void const *param);

typedef struct
{
    ke_msg_func_t ori_func_addr;
    ke_msg_func_t new_func_addr;
} msg_tab_item_t;

typedef struct
{
    llm_hci_cmd_hdl_func_t ori_func_addr;
    llm_hci_cmd_hdl_func_t new_func_addr;
} hci_cmd_tab_item_t;

typedef struct
{
    gapm_hci_evt_hdl_func_t ori_func_addr;
    gapm_hci_evt_hdl_func_t new_func_addr;
} gapm_hci_evt_tab_item_t;

// ble sdk task
extern int host_to_sdk_msg_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// ble sdk gapm for common
extern int gap_activity_stopped_ind_handler_patch(ke_msg_id_t const msgid, void const *p_param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_cmp_evt_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gap_dev_bdaddr_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// llm task for common
extern int llm_hci_command_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id,ke_task_id_t const src_id);
extern int llc_hci_command_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id,ke_task_id_t const src_id);

#if CFG_MUL_LINK_WITH_SAME_DEV
extern int lld_adv_end_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int hci_le_set_ext_adv_en_cmd_handler_patch(void const *param, uint16_t opcode);
#endif

// hci cmd for common
extern int hci_le_add_dev_to_wlst_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_rmv_dev_from_wlst_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_clear_wlst_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_clear_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_add_dev_to_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_priv_mode_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_addr_resol_en_cmd_handler_patch(void *param, uint16_t opcode);

extern int hci_dbg_ble_reg_wr_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_dbg_ble_reg_rd_cmd_handler_patch(void const *param, uint16_t opcode);

#if CFG_MAX_CONNECTIONS
// gattc task
extern int l2cc_pdu_recv_ind_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gattc_cmp_evt_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
// llc task
extern int llc_loc_llcp_rsp_to_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int llc_rem_llcp_rsp_to_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id,ke_task_id_t const src_id);
extern int lld_acl_rx_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int lld_con_offset_upd_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int lld_con_param_upd_cfm_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int lld_llcp_rx_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int lld_llcp_tx_cfm_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int lld_ch_map_upd_cfm_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int lld_phy_upd_cfm_handler_patch(ke_msg_id_t const msgid, void const*param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int llc_auth_payl_nearly_to_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int llc_auth_payl_real_to_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int llc_op_dl_upd_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// l2cc task
extern int hci_nb_cmp_pkts_evt_handler_patch(ke_msg_id_t const msgid, void const *event,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// gapc task
extern int gapc_hci_handler_patch(ke_msg_id_t const msgid, void const* event,
    ke_task_id_t dest_id, ke_task_id_t src_id);

#if CFG_MUL_LINK_WITH_SAME_DEV
int gapc_bond_cfm_handler_patch(ke_msg_id_t const msgid, void *cfm,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

// ble sdk task for gapc
extern int gap_connection_req_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gap_disconnect_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gap_cmp_evt_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int sec_rcv_sec_req_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int sec_rcv_encrypt_req_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gap_set_dev_info_req_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

#if CFG_MUL_LINK_WITH_SAME_DEV
extern int sec_rcv_bond_req_ind_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

// ble sdk task for gatt
extern int ble_sdk_gatts_svc_changed_cfg_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int ble_sdk_gattc_event_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int ble_sdk_gatt_mtu_changed_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

#if CFG_MAX_SCAN
// gapm task for scan
extern int gapm_activity_create_cmd_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// hci cmd for scan
extern int hci_le_ext_create_con_cmd_handler_patch(void *param, uint16_t opcode);
extern int hci_le_set_ext_scan_param_cmd_handler_patch(void const *param, uint16_t opcode);
extern int hci_le_set_ext_scan_en_cmd_handler_patch(void const *param, uint16_t opcode);

// gapm hci evt for scan
extern int hci_le_cmd_cmp_evt_init_handler_patch(uint16_t opcode, void const *p_event);
#endif

// gapm hci evt for scan or support 31 bytes adv data
#if (CFG_MAX_SCAN) || (CFG_MAX_ADVS && CFG_MAX_ADV_DATA_LEN_SUPPORT) || (CFG_MUL_LINK_WITH_SAME_DEV)
extern int gapm_hci_handler_patch(ke_msg_id_t const msgid, void const* event,
    ke_task_id_t dest_id, ke_task_id_t opcode);
#endif

// gapm task for support 31 bytes adv data
#if (CFG_MAX_ADVS && CFG_MAX_ADV_DATA_LEN_SUPPORT)
extern int gapm_set_adv_data_cmd_handler_patch(ke_msg_id_t const msgid, void const *p_param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int hci_le_cmd_cmp_evt_adv_handler_patch(uint16_t opcode, void const *p_event);
#endif

//gapm scan
#if (CFG_MAX_SCAN)
extern int hci_le_cmd_cmp_evt_scan_handler_patch(uint16_t opcode, void const *p_event);
#endif

#if CFG_MUL_LINK_WITH_SAME_DEV
extern int hci_le_adv_set_term_evt_handler_patch(uint16_t opcode, void const *p_event);
#endif

#if CFG_CAR_KEY_SUPPORT
// llm task for car key
extern int llm_pub_key_gen_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// llm hci cmd handler for car key
extern int hci_le_rd_local_p256_public_key_cmd_handler_patch(void const *param, uint16_t opcode);
#endif

// gapm cfg
extern int gapm_set_dev_config_cmd_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

//llm init
extern int lld_init_end_ind_handler_patch(ke_msg_id_t const msgid, void const*param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int llm_ch_map_to_handler_patch(ke_msg_id_t const msgid, void const*param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

// ble sdk gattc
extern int ble_sdk_gattc_prf_cmp_evt_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int ble_sdk_gattc_prf_event_ind_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int ble_sdk_gattc_prf_sdp_srvc_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

msg_tab_item_t msg_tab[] =
{
    // ble sdk gapm for common
    {(ke_msg_func_t)0x00074235, (ke_msg_func_t)gapm_cmp_evt_handler_patch},
    {(ke_msg_func_t)0x000709e1, (ke_msg_func_t)gap_activity_stopped_ind_handler_patch},
    {(ke_msg_func_t)0x00071f6d, (ke_msg_func_t)gap_dev_bdaddr_ind_handler_patch},

    // llm task for common
    {(ke_msg_func_t)0x00051d09, (ke_msg_func_t)llm_hci_command_handler_patch},

    #if CFG_MUL_LINK_WITH_SAME_DEV
    // llm task for slave
    {(ke_msg_func_t)0x0003b639, (ke_msg_func_t)lld_adv_end_ind_handler_patch},
    #endif

    // llc task for common
    {(ke_msg_func_t)0x00023231, (ke_msg_func_t)llc_hci_command_handler_patch},
    // ble sdk task for common
    {(ke_msg_func_t)0x000761a9, (ke_msg_func_t)host_to_sdk_msg_handler_patch},

    #if (CFG_MAX_ADVS && CFG_MAX_ADV_DATA_LEN_SUPPORT)
    {(ke_msg_func_t)0x00013f09, (ke_msg_func_t)gapm_set_adv_data_cmd_handler_patch},
    #endif

    #if CFG_MAX_CONNECTIONS
    // gattc task for conn
    {(ke_msg_func_t)0x00032e7d, (ke_msg_func_t)l2cc_pdu_recv_ind_handler_patch},
    {(ke_msg_func_t)0x0001493d, (ke_msg_func_t)gattc_cmp_evt_handler_patch},
    // llc task for conn
    {(ke_msg_func_t)0x000391b5, (ke_msg_func_t)llc_loc_llcp_rsp_to_handler_patch},
    {(ke_msg_func_t)0x0003a795, (ke_msg_func_t)llc_rem_llcp_rsp_to_handler_patch},
    {(ke_msg_func_t)0x00049a6d, (ke_msg_func_t)lld_llcp_rx_ind_handler_patch},
    {(ke_msg_func_t)0x00049c95, (ke_msg_func_t)lld_llcp_tx_cfm_handler_patch},
    {(ke_msg_func_t)0x0003aed9, (ke_msg_func_t)lld_acl_rx_ind_handler_patch},
    {(ke_msg_func_t)0x00045e6d, (ke_msg_func_t)lld_con_param_upd_cfm_handler_patch},
    {(ke_msg_func_t)0x000418c9, (ke_msg_func_t)lld_ch_map_upd_cfm_handler_patch},
    {(ke_msg_func_t)0x0004b67d, (ke_msg_func_t)lld_phy_upd_cfm_handler_patch},
    {(ke_msg_func_t)0x00045da1, (ke_msg_func_t)lld_con_offset_upd_ind_handler_patch},
    {(ke_msg_func_t)0x0003679d, (ke_msg_func_t)llc_auth_payl_nearly_to_handler_patch},
    {(ke_msg_func_t)0x00036801, (ke_msg_func_t)llc_auth_payl_real_to_handler_patch},
    {(ke_msg_func_t)0x000398a1, (ke_msg_func_t)llc_op_dl_upd_ind_handler_patch},
    //llm init for conn
    {(ke_msg_func_t)0x00047ca1, (ke_msg_func_t)lld_init_end_ind_handler_patch},
    {(ke_msg_func_t)0x0005199f, (ke_msg_func_t)llm_ch_map_to_handler_patch},
    // l2cc task for conn
    {(ke_msg_func_t)0x0002b281, (ke_msg_func_t)hci_nb_cmp_pkts_evt_handler_patch},
    // gapc task for conn
    {(ke_msg_func_t)0x0000f751, (ke_msg_func_t)gapc_hci_handler_patch},
    // ble sdk gapc for conn
    {(ke_msg_func_t)0x00071781, (ke_msg_func_t)gap_connection_req_ind_handler_patch},
    {(ke_msg_func_t)0x00072229, (ke_msg_func_t)gap_disconnect_ind_handler_patch},
    {(ke_msg_func_t)0x0007ac89, (ke_msg_func_t)sec_rcv_sec_req_ind_handler_patch},
    {(ke_msg_func_t)0x00071519, (ke_msg_func_t)gap_cmp_evt_handler_patch},
    {(ke_msg_func_t)0x0007ab15, (ke_msg_func_t)sec_rcv_encrypt_req_ind_handler_patch},
    {(ke_msg_func_t)0x00073655, (ke_msg_func_t)gap_set_dev_info_req_ind_handler_patch},

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(ke_msg_func_t)0x0007a881, (ke_msg_func_t)sec_rcv_bond_req_ind_handler_patch},
    #endif

    // ble sdk gatt for conn
    {(ke_msg_func_t)0x0006d0d5, (ke_msg_func_t)ble_sdk_gattc_event_ind_handler_patch},
    {(ke_msg_func_t)0x0006db75, (ke_msg_func_t)ble_sdk_gatts_svc_changed_cfg_ind_handler_patch},
    {(ke_msg_func_t)0x0006cd41, (ke_msg_func_t)ble_sdk_gatt_mtu_changed_ind_handler_patch},
    #endif

    #if CFG_MAX_SCAN
    // gapm task for scan
    {(ke_msg_func_t)0x00010d2d, (ke_msg_func_t)gapm_activity_create_cmd_handler_patch},
    #endif

    #if (CFG_MAX_SCAN) || (CFG_MAX_ADVS && CFG_MAX_ADV_DATA_LEN_SUPPORT) || (CFG_MUL_LINK_WITH_SAME_DEV)
    {(ke_msg_func_t)0x00012751, (ke_msg_func_t)gapm_hci_handler_patch},
    #endif

    {(ke_msg_func_t)0x00013fe9, (ke_msg_func_t)gapm_set_dev_config_cmd_handler_patch},

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(ke_msg_func_t)0x0000e99d, (ke_msg_func_t)gapc_bond_cfm_handler_patch},
    #endif

    #if CFG_CAR_KEY_SUPPORT
    // llm task for car key
    {(ke_msg_func_t)0x00052309, (ke_msg_func_t)llm_pub_key_gen_ind_handler_patch},
    #endif
};

hci_cmd_tab_item_t hci_cmd_tab[] =
{
    // hci cmd for common
    {(llm_hci_cmd_hdl_func_t)0x00024711, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_wlst_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x000281f5, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_wlst_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00024fa1, (llm_hci_cmd_hdl_func_t)hci_le_clear_wlst_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x000283b5, (llm_hci_cmd_hdl_func_t)hci_le_set_addr_resol_en_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00024f75, (llm_hci_cmd_hdl_func_t)hci_le_clear_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00024681, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x0002a99d, (llm_hci_cmd_hdl_func_t)hci_le_set_priv_mode_cmd_handler_patch},

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(llm_hci_cmd_hdl_func_t)0x00029731, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_adv_en_cmd_handler_patch},
    #endif

    #if CFG_MAX_SCAN
    {(llm_hci_cmd_hdl_func_t)0x0002677d, (llm_hci_cmd_hdl_func_t)hci_le_ext_create_con_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x0002a081, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_scan_param_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00029e5d, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_scan_en_cmd_handler_patch},
    #endif

    #if DTM_TEST_ENABLE
    {(llm_hci_cmd_hdl_func_t)0x000233f7, (llm_hci_cmd_hdl_func_t)hci_dbg_ble_reg_wr_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x000233c9, (llm_hci_cmd_hdl_func_t)hci_dbg_ble_reg_rd_cmd_handler_patch},
    #endif

    #if CFG_CAR_KEY_SUPPORT
    // llm hci cmd for car key
    {(llm_hci_cmd_hdl_func_t)0x00027455, (llm_hci_cmd_hdl_func_t)hci_le_rd_local_p256_public_key_cmd_handler_patch},
    #endif
};

gapm_hci_evt_tab_item_t gapm_hci_evt_tab[] =
{
    {NULL, NULL},

    #if CFG_MAX_SCAN
    {(gapm_hci_evt_hdl_func_t)0x00025121, (gapm_hci_evt_hdl_func_t)hci_le_cmd_cmp_evt_init_handler_patch},
    #endif

    #if (CFG_MAX_ADVS && CFG_MAX_ADV_DATA_LEN_SUPPORT)
    {(gapm_hci_evt_hdl_func_t)0x00025031, (gapm_hci_evt_hdl_func_t)hci_le_cmd_cmp_evt_adv_handler_patch},
    #endif

    #if (CFG_MAX_SCAN)
    {(gapm_hci_evt_hdl_func_t)0x00025229, (gapm_hci_evt_hdl_func_t)hci_le_cmd_cmp_evt_scan_handler_patch},
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(gapm_hci_evt_hdl_func_t)0x000248cd, (gapm_hci_evt_hdl_func_t)hci_le_adv_set_term_evt_handler_patch},
    #endif
};

#endif
