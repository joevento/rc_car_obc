#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "bluetooth_comm.h"
#include "nvs_flash.h"

#define SPP_SERVER_NAME "ESP32_OBC"
#define DEVICE_NAME "ESP32_Car"

static const char *TAG = "Bluetooth";
static uint32_t spp_handle = 0;
static bool bt_connected = false;

__attribute__((weak)) void bluetooth_on_motor_command(const uint8_t *data, size_t length) {
    (void)data;
    (void)length;
}

bool bluetooth_is_connected(void) {
    return bt_connected;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized");
            esp_bt_gap_set_device_name(DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "Client connected");
            spp_handle = param->srv_open.handle;
            bt_connected = true;
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "Client disconnected");
            bt_connected = false;
            spp_handle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT:
            #ifdef DEBUG
                ESP_LOGI(TAG, "Received %d bytes", param->data_ind.len);
            #endif
            bluetooth_on_motor_command(param->data_ind.data, param->data_ind.len);
            break;

        case ESP_SPP_CONG_EVT:
            ESP_LOGW(TAG, "Congestion %s", param->cong.cong ? "start" : "end");
            break;

        case ESP_SPP_START_EVT:
            ESP_LOGI(TAG, "SPP server started on channel %d", param->start.scn);
            break;

        case ESP_SPP_WRITE_EVT:
            break;

        default:
            break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGI(TAG, "Discovery state: %s", 
                param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED ? "started" : "stopped");
            break;
        default:
            break;
    }
}

esp_err_t bluetooth_init(void) {
    ESP_LOGI(TAG, "Initializing Bluetooth.");

    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_gap_register_callback(esp_bt_gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_spp_register_callback(esp_spp_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    ret = esp_spp_enhanced_init(&bt_spp_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Bluetooth initialized.");
    return ESP_OK;
}

esp_err_t bluetooth_send(const uint8_t *data, size_t length) {
    if (!data || length == 0) return ESP_ERR_INVALID_ARG;
    if (!bt_connected) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = esp_spp_write(spp_handle, length, (uint8_t *)data);
    return ret;
}
