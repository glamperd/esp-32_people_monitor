// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



/****************************************************************************
*
* This file is for gatt client. It can scan ble device, connect one device.
* Run the gatt_server demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. Then the two devices will exchange
* data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "controller.h"

#include "bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"

#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "mqtt_client.h"


#include "driver/gpio.h"
#include "mdns.h"

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netdb.h"


#define GATTC_TAG "SENSOR_READ"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xCA9E
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

//static const char *TAG = "MQTT_SAMPLE";

//static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static const esp_gatt_auth_req_t gattc_auth_request_none = ESP_GATT_AUTH_REQ_NONE;

static const char remote_device_name[] = "ID107 HR"; // "SENSOR";
static const uint8_t service_id[] = {0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E}; // needs to be reversed
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
//static esp_gattc_service_elem_t *service_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;
static uint8_t charac_handles[] = {0x11, 0x14, 0x17};
static esp_bt_uuid_t charac_uuids[3];
static int charac_pointer = 0;

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

// MQTT globals
static bool mqtt_is_connected = false;
static esp_mqtt_client_handle_t mqtt_client;
static const char mqtt_topic_bt_info[] = "/epm/bt";

/* static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
}; */

//static esp_bt_uuid_t remote_filter_char_uuid = {
//    .len = ESP_UUID_LEN_128,
//    uuid 
//};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bt_uuid_t uuid;
    esp_bd_addr_t remote_bda;
    int16_t rssi;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static bool bt_scan_done = false;

//*********************************************
// web server variables

// HTTP headers and web pages
const static char http_html_hdr[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
const static char http_png_hdr[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
const static char http_page_hdr[] = "<meta content=\"width=device-width,initial-scale=1\"name=viewport><style>div{width:230px;height:300px;position:absolute;top:0;bottom:0;left:0;right:0;margin:auto}</style>";
const static char http_page_body[] = "<div><h1 align=center>Sensor %s detected</h1><p>%s RSSI is: %s<p>Human detected: %d</div>";
const static char http_on_hml[] = "<div><h1 align=center>Relay is ON</h1><a href=off.html><img src=off.png></a></div>";

// embedded binary data
extern const uint8_t on_png_start[] asm("_binary_on_png_start");
extern const uint8_t on_png_end[]   asm("_binary_on_png_end");
extern const uint8_t off_png_start[] asm("_binary_off_png_start");
extern const uint8_t off_png_end[]   asm("_binary_off_png_end");


// Event group for inter-task communication
static EventGroupHandle_t event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// actual relay status
bool relay_status;

int16_t sensorRssi = 0;
bool sensorFound = false;



static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    mqtt_client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_bt_info, 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(mqtt_client, "/topic/qos1", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_unsubscribe(mqtt_client, "/topic/qos1");
            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            mqtt_is_connected = true;
            //msg_id = esp_mqtt_client_publish(client, mqtt_topic_bt_info, "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            mqtt_is_connected = false;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}


// *****************************************

/* Read a characteristic for ITC PolyU HK sensor.
   Called whenever a characteristic read is to be requested, namely after initial service
   discovery and after each subsequent characteristic read, until the list is exhausted.
*/
static void sensor_characteristic_reader() {
    // Read Temperature
    gl_profile_tab[PROFILE_A_APP_ID].char_handle = charac_handles[charac_pointer];
    gl_profile_tab[PROFILE_A_APP_ID].uuid = charac_uuids[charac_pointer];
    ESP_LOGI(GATTC_TAG, "requesting characteristic %x", charac_handles[charac_pointer]);
    //esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[4].char_handle);
    
    // Direct read
    /* esp_gatt_status_t status = esp_ble_gattc_read_char(   
                                        gl_profile_tab[PROFILE_A_APP_ID].gattc_if, 
                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                        gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                        ESP_GATT_AUTH_REQ_NONE);
    */
    // Request notifications
    esp_gatt_status_t status = esp_ble_gattc_register_for_notify(   
                                        gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                        gl_profile_tab[PROFILE_A_APP_ID].char_handle
                                        );
    if (status != ESP_GATT_OK){
        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_read_char error");
    }

}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        //p_data->connect.status always be ESP_GATT_OK
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d, status %d", p_data->connect.conn_id, gattc_if, p_data->connect.status);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL); // &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        esp_gatt_srvc_id_t *srvc_id =(esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) { //} && srvc_id->id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            //ESP_LOGI(GATTC_TAG, "UUID16: %x len: %d", srvc_id->id.uuid.uuid.uuid128[3], srvc_id->id.uuid.len);
            char uuid_temp[ESP_UUID_LEN_128];
            memcpy(uuid_temp, &srvc_id->id.uuid.uuid.uuid128, srvc_id->id.uuid.len);
            esp_log_buffer_hex(GATTC_TAG, uuid_temp, srvc_id->id.uuid.len);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            uint16_t offset = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            ESP_LOGI(GATTC_TAG, "%d attributes found", count);
            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(char_elem_result) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_all_char(     gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             char_elem_result,
                                                             &count,
                                                             offset);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_all_char error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    for (uint8_t i=0; i<count; i++) {
                    //if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        ESP_LOGI(GATTC_TAG, "char: %d; char handle %x; props %x", i, char_elem_result[i].char_handle, char_elem_result[i].properties);
                        esp_log_buffer_hex(GATTC_TAG, char_elem_result[i].uuid.uuid.uuid128, char_elem_result[i].uuid.len);
                        for (uint8_t j=0; j<sizeof charac_handles; j++) {
                            if (char_elem_result[i].char_handle == charac_handles[j]) {
                                charac_uuids[j] = char_elem_result[i].uuid;
                                ESP_LOGI(GATTC_TAG, "found handle %x", charac_handles[j]);
                            }
                        }
                    }
                    charac_pointer = 0;
                    sensor_characteristic_reader();
                }
                /* free char_elem_result */
                free(char_elem_result);

                }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            ESP_LOGI(GATTC_TAG, " handle %x; conn %d; if %d", gl_profile_tab[PROFILE_A_APP_ID].char_handle, gl_profile_tab[PROFILE_A_APP_ID].conn_id, gattc_if);
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t  ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                     /*   gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle, */
																		 0,
																		 0,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count); 
            
            //count = 1;
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            
            ESP_LOGI(GATTC_TAG, "count %d; handle %x", count, gl_profile_tab[PROFILE_A_APP_ID].char_handle);
            esp_log_buffer_hex(GATTC_TAG, p_data->read.value, p_data->read.value_len);

            if (count > 0){
                //count = 1;
                esp_gatt_status_t ret_status;
                descr_elem_result = malloc(sizeof(descr_elem_result) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }else{
                    ESP_LOGI(GATTC_TAG, "get_descr_by_char_handle  %x; status %d", p_data->reg_for_notify.handle, p_data->reg_for_notify.status);
                    esp_log_buffer_hex(GATTC_TAG, &notify_descr_uuid.uuid, ESP_UUID_LEN_16);
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
																		 notify_descr_uuid, // 0x2902
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    ESP_LOGI(GATTC_TAG, "get_descr uuid, count %d", count);
                    //esp_log_buffer_hex(GATTC_TAG, descr_elem_result[0].uuid.uuid.uuid128, descr_elem_result[0].uuid.len); 

                    /* Every char have only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "descr not found");
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value for handle %x:", p_data->notify.handle);
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        // Request the next value.
        sensor_characteristic_reader();

        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success ");
        /*uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        */
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, status = %d", p_data->disconnect.status);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_READ_CHAR_EVT, status = %d, len=%d, handle %x", p_data->read.status, p_data->read.value_len, gl_profile_tab[PROFILE_A_APP_ID].char_handle);
        if (gl_profile_tab[PROFILE_A_APP_ID].char_handle == 0x15) { // Temperature
            uint8_t valint;
            memcpy(&valint, p_data->read.value, 2);
            float temp = valint/10.0f;
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_READ_CHAR_EVT, Temperature = %.1f", temp);
        }else if (gl_profile_tab[PROFILE_A_APP_ID].char_handle == 0x11) { // Pressure
            //uint8_t valint;
            //memcpy(&valint, p_data->read.value, 1);
            //float temp = valint/10.0f;
            //ESP_LOGI(GATTC_TAG, "ESP_GATTC_READ_CHAR_EVT, Temperature = %.1f", temp);
            esp_log_buffer_hex("ESP_GATTC_READ_CHAR_EVT barometer ", p_data->read.value, 4); // big endian!
        } else {
            esp_log_buffer_hex("ESP_GATTC_READ_CHAR_EVT", p_data->read.value, p_data->read.value_len);
        }
        if (++charac_pointer>=(sizeof(charac_handles))) {
            bt_scan_done = true;
            charac_pointer = 0;
            vTaskDelay(1000);
            printf("waited after last char\n");
        } else {
            // Request the next value.
            sensor_characteristic_reader();
        }
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            //esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            //ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d, rssi %d", scan_result->scan_rst.adv_data_len,
            //                                                                               scan_result->scan_rst.scan_rsp_len,
            //                                                                               scan_result->scan_rst.rssi);
            // Send data via MQTT
            if (mqtt_is_connected) {
            	// Send RSSI data
    			char tempstr[100];
            	sprintf(tempstr, "Advertised Data len %d, Scan Response Len %d, rssi %d",
            			scan_result->scan_rst.adv_data_len,
						scan_result->scan_rst.scan_rsp_len,
						scan_result->scan_rst.rssi );
				int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_bt_info, tempstr, 0, 0, 0);
				ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

				adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
												ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
				//ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
				//esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
				//ESP_LOGI(GATTC_TAG, "\n");
				if (adv_name != NULL) {
					// Send device name
					sprintf(tempstr, "adv name: %s", adv_name);
					msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_bt_info, tempstr, 0, 0, 0);
					if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
						ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
						sensorRssi = scan_result->scan_rst.rssi;
						sensorFound = true;
						if (connect == false) {
							connect = true;
							ESP_LOGI(GATTC_TAG, "connect to the remote device.");
							esp_ble_gap_stop_scanning();
							//esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, true);
						}
					}
				}
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

// *******************************************************
// Wifi event handler

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {

    case SYSTEM_EVENT_STA_START:
    	printf("Event STA start\n");
        ESP_ERROR_CHECK(esp_wifi_connect());
    	printf("Wifi connect done.\n");
        break;

	case SYSTEM_EVENT_STA_GOT_IP:
    	printf("Event got IP\n");
        xEventGroupSetBits(event_group, WIFI_CONNECTED_BIT);
        break;

	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupClearBits(event_group, WIFI_CONNECTED_BIT);
        break;

	default:
        break;
    }
	return ESP_OK;
}


// setup and start the wifi connection
void wifi_setup() {

	event_group = xEventGroupCreate();
	printf("event Group Created\n");

	tcpip_adapter_init();
	printf("TCP/IP adapter inited\n");

	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	printf("wifi config inited\n");
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	printf("ssid %s\n", CONFIG_WIFI_SSID);
	wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
	printf("setting wifi config\n");
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
	printf("wifi setup exit\n");
}


// configure the output PIN
void gpio_setup() {

	// configure the pit pin as GPIO, output
	gpio_pad_select_gpio(CONFIG_PIR_PIN);
    gpio_set_direction(CONFIG_PIR_PIN, GPIO_MODE_INPUT);

	// set initial status = OFF
	//gpio_set_level(CONFIG_PIR_PIN, 0);
	relay_status = false;
}

// ****************************************************

void init_wifi()
{
	// disable the default wifi logging
	esp_log_level_set("wifi", ESP_LOG_NONE);

	//nvs_flash_init();
	wifi_setup();

	// wait for connection
	printf("Waiting for connection to the wifi network...\n ");
	EventBits_t resultBits = xEventGroupWaitBits(event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
	if (resultBits & WIFI_CONNECTED_BIT) {
		printf("Connected\n\n");
	} else {
		printf("Wifi timed out waiting for connection\n\n");
	}

	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
	//hostAddr = &ip_info.ip;

	gpio_setup();

	// run the mDNS daemon
	mdns_server_t* mDNS = NULL;
	ESP_ERROR_CHECK(mdns_init(TCPIP_ADAPTER_IF_STA, &mDNS));
	ESP_ERROR_CHECK(mdns_set_hostname(mDNS, "esp32"));
	ESP_ERROR_CHECK(mdns_set_instance(mDNS, "Basic HTTP Server"));
	printf("mDNS started\n");

	// start the HTTP Server task
    //xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);
	//http_server();
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://iot.eclipse.org",
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void app_main()
{
    ESP_LOGI(GATTC_TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(GATTC_TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);


    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

	// Connect to wifi router
	init_wifi();

    mqtt_app_start();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    while(true) {
        bt_scan_done = false;
        ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
        if (ret){
            ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
        }
        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }
        
        int delay_count = 0;
        while(!bt_scan_done && (delay_count++ < 30)) {
            printf("delay\n");
            vTaskDelay(1000);
        }
        
        printf("%llu: light sleep start\n", 0llu);
        esp_sleep_enable_timer_wakeup(30000000);
        esp_light_sleep_start();
        printf("%llu: light sleep out\n", 1llu);

    }

}

