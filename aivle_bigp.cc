#include "rel_common.h"
#include "model_inference.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

#include "audio_processing.h"
#include "sd_card.h"
#include "nimble_handler.h"
// #include "uart_handler.h"

static const char* TAG = "MAIN";

void loop(void);

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    // NVS 플래시 메모리 초기화
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 이벤트 감시자 초기화
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 60000,   // 타임아웃 60초
        .idle_core_mask = 3,  // 모든 코어의 유휴 태스크 모니터링
        .trigger_panic = true  // WDT 타임아웃 시 PANIC 발생 (시스템 재부팅)
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));

    // SD 카드 초기화
    ret = init_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SD card");
        return;
    }

    // Audio processing 초기화
    ret = init_audio_processing();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio processing");
        return;
    }

    // Model 추론 초기화
    ret = init_model_inference();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize model inference");
        return;
    }

    // ble 초기화
    while (!init_nimble()) {
        ESP_LOGE(TAG, "Failed to initialize nimble");
        vTaskDelay(pdMS_TO_TICKS(10));
    };

    // init_uart();

    ESP_LOGI(TAG, "System ready. please bluetooth connection");

    loop();

    cleanup_sd_card();
    cleanup_audio_processing();
    cleanup_model_inference();

    ESP_LOGI(TAG, "Application ended");
}

void loop() {
    while (1) {
        nimble_port_run();
        // uart_read();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
