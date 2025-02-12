#include "rel_common.h"
#include "audio_processing.h"
#include "nimble_handler.h"
#include "gap.h"
#include "gatt_svc.h"

#include <string.h>

static const char *TAG = "BLE";

static void on_stack_reset(int reason) {
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    int rc = ble_gatts_start();
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT 서비스 시작 실패: %d", rc);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    adv_init();
}

static void nimble_host_config_init(void) {
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
}

bool init_nimble() {
    esp_err_t ret;
    int rc;

    vTaskDelay(pdMS_TO_TICKS(100));
    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    nimble_host_config_init();
    
    return true;
}
