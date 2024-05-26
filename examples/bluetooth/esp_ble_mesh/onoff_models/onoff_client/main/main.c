/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"
#include "esp_timer.h"

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5
int c=0;
int c1=0;
uint8_t data[8];
int len=2;//Change size to number of expected status confirmation messages
int64_t time_arr[2];
int64_t status_cont_time;
int64_t sync_time;
int pt=0;
bool first=true;
static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static struct example_info_store {
    uint16_t net_idx;   /* NetKey Index */
    uint16_t app_idx;   /* AppKey Index */
    uint8_t  onoff;     /* Remote OnOff */
    uint8_t  onoff1;     /* Remote OnOff */
    uint8_t  onoff2;     /* Remote OnOff */
    uint8_t  tid;       /* Message TID */
} __attribute__((packed)) store = {
    .net_idx = ESP_BLE_MESH_KEY_UNUSED,
    .app_idx = ESP_BLE_MESH_KEY_UNUSED,
    .onoff = LED_OFF,
    .onoff1 = LED_OFF,
    .onoff2 = LED_OFF,
    .tid = 0x0,
};
float m=2;//Number of lights
float n=1;//Number of relays
float sent=0;
float recv=0;
float pdr;
static TimerHandle_t delay_timer;
static TimerHandle_t delay_timer1;
static TimerHandle_t delay_timer2;
static TimerHandle_t delay_timer3;

static nvs_handle_t NVS_HANDLE;
static const char * NVS_KEY = "onoff_client";

static esp_ble_mesh_client_t onoff_client;

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 1, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_size = 4,
    .input_actions = ESP_BLE_MESH_PUSH,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

static void mesh_example_info_store(void)
{
    ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
}

static void mesh_example_info_restore(void)
{
    esp_err_t err = ESP_OK;
    bool exist = false;

    err = ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), &exist);
    if (err != ESP_OK) {
        return;
    }

    if (exist) {
        ESP_LOGI(TAG, "Restore, net_idx 0x%04x, app_idx 0x%04x, onoff %u, tid 0x%02x",
            store.net_idx, store.app_idx, store.onoff, store.tid);
    }
}

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
    store.net_idx = net_idx;
    /* mesh_example_info_store() shall not be invoked here, because if the device
     * is restarted and goes into a provisioned state, then the following events
     * will come:
     * 1st: ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT
     * 2nd: ESP_BLE_MESH_PROV_REGISTER_COMP_EVT
     * So the store.net_idx will be updated here, and if we store the mesh example
     * info here, the wrong app_idx (initialized with 0xFFFF) will be stored in nvs
     * just before restoring it.
     */
}
static void change_relay_mode(uint8_t state)
{
    if(state == 1)
    {
        config_server.relay = ESP_BLE_MESH_RELAY_ENABLED;
        ESP_LOGI(TAG,"RELAY ENABLED");
    }
    else
    {
        config_server.relay = ESP_BLE_MESH_RELAY_DISABLED;
                ESP_LOGI(TAG,"RELAY DISABLED");
    }
}
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        mesh_example_info_restore(); /* Restore proper mesh example info */
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}
void start_experiment();
void pdr_callback()
{
    pdr=(recv/sent)*100;
    ESP_LOGI(TAG, "Current PDR is : %f percent",pdr);
    recv=0;
int64_t min=time_arr[0];
int64_t max=time_arr[0];
for(int i=0;i<len;i++)
{
        if(time_arr[i]<min)
        {
            min=time_arr[i];
        }
        if(time_arr[i]>max)
        {
            max=time_arr[i];
        }
                 ESP_LOGI(TAG, "Time array: %lld",time_arr[i]);
}
int64_t consistent_time=max-min+350000;
    int64_t execution_time=max-(status_cont_time-sync_time);
    ESP_LOGI(TAG, "Consistent time is : %lld",consistent_time);
    ESP_LOGI(TAG, "Execution time is : %lld",execution_time);
}
void example_ble_mesh_send_gen_onoff_set(void)
{
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    common.model = onoff_client.model;
    common.ctx.net_idx = store.net_idx;
    common.ctx.app_idx = store.app_idx;
    common.ctx.addr = 0xC000;   /* to all light nodes */
    common.ctx.send_ttl = 3;
    common.msg_timeout = 0;     /* 0 indicates that timeout value from menuconfig will be used */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common.msg_role = ROLE_NODE;
#endif

    set.onoff_set.op_en = false;
    set.onoff_set.onoff = store.onoff;
    set.onoff_set.tid = store.tid++;

    err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err) {
        ESP_LOGE(TAG, "Send Generic OnOff Set Unack failed");
        return;
    }

    store.onoff = !store.onoff;
     if (delay_timer != NULL) {
            xTimerStop(delay_timer, portMAX_DELAY);
        }

            delay_timer = xTimerCreate("RandomDelayTimer", pdMS_TO_TICKS(33001), pdFALSE, NULL, pdr_callback);
        if (delay_timer != NULL) {
            xTimerStart(delay_timer, portMAX_DELAY);
        }
    mesh_example_info_store(); /* Store proper mesh example info */
}
void example_ble_mesh_send_gen_onoff_set1(void)
{
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    common.model = onoff_client.model;
    common.ctx.net_idx = store.net_idx;
    common.ctx.app_idx = store.app_idx;
    common.ctx.addr = 0xC001;   /* to all relay nodes */
    common.ctx.send_ttl = 3;
    common.msg_timeout = 0;     /* 0 indicates that timeout value from menuconfig will be used */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common.msg_role = ROLE_NODE;
#endif

    set.onoff_set.op_en = false;
    set.onoff_set.onoff = store.onoff1;
    set.onoff_set.tid = store.tid++;

    err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err) {
        ESP_LOGE(TAG, "Send Generic OnOff Set Unack failed");
        return;
    }
    store.onoff1 = !store.onoff1;
    mesh_example_info_store(); /* Store proper mesh example info */
}
void example_ble_mesh_send_gen_onoff_set2(void)
{
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    common.model = onoff_client.model;
    common.ctx.net_idx = store.net_idx;
    common.ctx.app_idx = store.app_idx;
    common.ctx.addr = 0xC002;   /* to all light nodes */
    common.ctx.send_ttl = 3;
    common.msg_timeout = 0;     /* 0 indicates that timeout value from menuconfig will be used */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common.msg_role = ROLE_NODE;
#endif

    set.onoff_set.op_en = false;
    set.onoff_set.onoff = store.onoff2;
    set.onoff_set.tid = store.tid++;

    err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err) {
        ESP_LOGE(TAG, "Send Generic OnOff Set Unack failed");
        return;
    }

    store.onoff2 = !store.onoff2;
    mesh_example_info_store(); /* Store proper mesh example info */
}
void start_experiment()
{
    if(first)
    {
        //store.onoff2=LED_ON;
              sync_time=esp_timer_get_time();
        example_ble_mesh_send_gen_onoff_set();
    //     vTaskDelay(pdMS_TO_TICKS(355));
    //   example_ble_mesh_send_gen_onoff_set1(); 
      store.onoff=LED_ON;
      first=false;
    }
        // example_ble_mesh_send_gen_onoff_set2();
        // vTaskDelay(pdMS_TO_TICKS(1000));
            status_cont_time=esp_timer_get_time();
        example_ble_mesh_send_gen_onoff_set();
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // example_ble_mesh_send_gen_onoff_set2();
        sent=m;
}
void save_time()
{
    int64_t reconstructed_value = 0;

// Assuming buffer is a uint8_t array of size 8
reconstructed_value = (int64_t)data[0];
reconstructed_value |= ((int64_t)data[1]) << 8;
reconstructed_value |= ((int64_t)data[2]) << 16;
reconstructed_value |= ((int64_t)data[3]) << 24;
reconstructed_value |= ((int64_t)data[4]) << 32;
reconstructed_value |= ((int64_t)data[5]) << 40;
reconstructed_value |= ((int64_t)data[6]) << 48;
reconstructed_value |= ((int64_t)data[7]) << 56;
time_arr[pt]=reconstructed_value;

pt+=1;
if(pt==len)
{
    pt=0;
}
}
void stop_experiment(void)
{
    if (delay_timer1 != NULL) {
            xTimerStop(delay_timer1, portMAX_DELAY);
        } 
}
static void example_ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                               esp_ble_mesh_generic_client_cb_param_t *param)
{
    
    switch (event) {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT");
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, onoff %d", param->status_cb.onoff_status.present_onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT");
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET, onoff %d", param->status_cb.onoff_status.present_onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS) {
            //take the time and store it in time_arr, then increment pt by 1 then check if pt=2 reset pt
            if(c1!=0)
            {
                data[c1-1]=param->status_cb.onoff_status.present_onoff;
                c1+=1;
                if(c1==9)
                {
                    c1=0;
                    save_time();
                }
            }
            else  if (param->status_cb.onoff_status.present_onoff == 10 || param->status_cb.onoff_status.present_onoff == 11)
            {
                recv=recv+1;
                            ESP_LOGI(TAG, "Received status confirmation from light, status: %d", param->status_cb.onoff_status.present_onoff-10);  
                            c1=1;

            }
            else if (param->status_cb.onoff_status.present_onoff == 20 || param->status_cb.onoff_status.present_onoff == 21)
             {
           
                recv=recv+1;
            ESP_LOGI(TAG, "Received status confirmation from relay, status: %d", param->status_cb.onoff_status.present_onoff-20);
            c1=1;
            }
//             else if(param->status_cb.onoff_status.present_onoff == 30 || param->status_cb.onoff_status.present_onoff == 31)
//             {
// if(c==0)
// {
//     board_led_operation(LED_G, LED_ON);
//     vTaskDelay(pdMS_TO_TICKS(350));
//     board_led_operation(LED_G, LED_OFF);
//     store.onoff1=param->status_cb.onoff_status.present_onoff-30;
// ESP_LOGI(TAG,"Relay is enabled, relaying the message.....");
// example_ble_mesh_send_gen_onoff_set1();
// c=c+1;
// }
// else{
//     c=c+1;
//     if(c==3)
//     {
//         c=0;
//     }

// }
//             }
//             else if(param->status_cb.onoff_status.present_onoff == 40 || param->status_cb.onoff_status.present_onoff == 41)
// {                
//    if(c==0)
// {
//     board_led_operation(LED_G, LED_ON);
//     vTaskDelay(pdMS_TO_TICKS(200));
//     board_led_operation(LED_G, LED_OFF);
//         vTaskDelay(pdMS_TO_TICKS(100));
//     board_led_operation(LED_G, LED_ON);
//     vTaskDelay(pdMS_TO_TICKS(200));
//     board_led_operation(LED_G, LED_OFF);
//             change_relay_mode(param->status_cb.onoff_status.present_onoff-40);
// c=c+1;
// }
// else{
//     c=c+1;
//     if(c==3)
//     {
//         c=0;
//     }

// }
                
//              }
}
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT");
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            /* If failed to get the response of Generic OnOff Set, resend Generic OnOff Set  */
            example_ble_mesh_send_gen_onoff_set();
        }
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            for(int i=0;i<composition.element_count;i++)
            {
                ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",composition.elements[i].element_addr,param->value.state_change.appkey_add.app_idx,0xffff,0x1000);
            esp_ble_mesh_node_bind_app_key_to_local_model(composition.elements[i].element_addr,0xffff,0x1001,param->value.state_change.appkey_add.app_idx);
            esp_ble_mesh_model_subscribe_group_addr(composition.elements[i].element_addr,0xffff,0x1001,0xC002);
            store.app_idx = param->value.state_change.appkey_add.app_idx;
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            if (param->value.state_change.mod_app_bind.company_id == 0xFFFF &&
                param->value.state_change.mod_app_bind.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI) {
                store.app_idx = param->value.state_change.mod_app_bind.app_idx;
                mesh_example_info_store(); /* Store proper mesh example info */
            }
            break;
        default:
            break;
        }
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_generic_client_callback(example_ble_mesh_generic_client_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    board_led_operation(LED_G, LED_ON);

    return err;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Open nvs namespace for storing/restoring mesh example info */
    err = ble_mesh_nvs_open(&NVS_HANDLE);
    if (err) {
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}
