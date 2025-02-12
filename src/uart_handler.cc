#include "rel_common.h"
#include "driver/uart.h"
#include "audio_processing.h"
#include "model_inference.h"

#define UART_NUM UART_NUM_0

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0));
}

void uart_read() {
    uint8_t data[1024];

    int len = uart_read_bytes(UART_NUM, data, 1024, 20 / portTICK_PERIOD_MS);

    if (len > 0 && data[0] == 'r') {
        recordAudio();
        const char* result = pipeline();
        ESP_LOGI("AI", "result: %s", result);
    }
}
