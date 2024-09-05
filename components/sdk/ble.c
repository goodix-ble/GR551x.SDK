#include "custom_config.h"
#include "ble.h"
#include "ble_event.h"
#include "patch_tab.h"

extern void ble_con_env_init(void);
extern void ble_adv_env_init(void);
extern void ble_per_adv_env_init(void);
extern void ble_scan_env_init(void);
extern void ble_sync_env_init(void);
extern void ble_adv_param_init(uint8_t max_conn);
extern void ble_mul_link_with_same_device(void);
extern void ble_enable_bt_bredr(void);

extern void reg_hci_cmd_patch_tab(hci_cmd_tab_item_t *hci_cmd_tab, uint16_t hci_cmd_cnt);
extern void reg_msg_patch_tab(msg_tab_item_t *msg_tab, uint16_t msg_cnt);
extern void reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab_item_t *gapm_hci_evt_tab, uint16_t gapm_hci_evt_cnt);
extern void register_rwip_reset(void (*callback)(void));
extern void rwip_reset_patch(void);
extern void register_rwip_init(void (*callback)(uint32_t));
extern void rwip_init_patch(uint32_t error);

extern uint16_t ble_stack_enable(ble_evt_handler_t evt_handler, stack_heaps_table_t *p_heaps_table);

void ble_sdk_patch_env_init(void)
{
    #if (CFG_MAX_CONNECTIONS < 3) && (CFG_MAX_ADVS < 2) && (CFG_MESH_SUPPORT < 1)
    register_rwip_reset(rwip_reset_patch);
    register_rwip_init(rwip_init_patch);
    #endif

    // register the msg handler for patch
    uint16_t msg_cnt = sizeof(msg_tab) / sizeof(msg_tab_item_t);
    reg_msg_patch_tab(msg_tab, msg_cnt);

    // register the hci cmd handler for patch
    uint16_t hci_cmd_cnt = sizeof(hci_cmd_tab) / sizeof(hci_cmd_tab_item_t);
    reg_hci_cmd_patch_tab(hci_cmd_tab, hci_cmd_cnt);

    // register the gapm hci evt handler for patch
    uint16_t hci_evt_cnt = sizeof(gapm_hci_evt_tab) / sizeof(gapm_hci_evt_tab_item_t);
    reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab, hci_evt_cnt);

    #if CFG_MAX_CONNECTIONS
    ble_con_env_init();
    ble_adv_param_init(CFG_MAX_CONNECTIONS);
    #endif

    #if CFG_MAX_SCAN
    ble_scan_env_init();
    #endif

    #if CFG_MAX_ADVS
    ble_adv_env_init();
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    ble_mul_link_with_same_device();
    #endif

    #if CFG_BT_BREDR
    ble_enable_bt_bredr();
    #endif

    #if CFG_CAR_KEY_SUPPORT
    ble_car_key_env_init();
    #endif
}

uint16_t ble_stack_init(ble_evt_handler_t evt_handler, stack_heaps_table_t *p_heaps_table)
{
    ble_sdk_patch_env_init();
    return ble_stack_enable(evt_handler, p_heaps_table);
}
