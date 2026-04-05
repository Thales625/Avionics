#include "esp_err.h"
#include "driver/gpio.h"

#include "lora.h"

esp_err_t lora_init(lora_dev_t *dev) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    uart_config_t uart_config = {
        .baud_rate = dev->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // buffer RX = 256 bytes | buffer TX = 512 bytes
    ESP_ERROR_CHECK(uart_driver_install(dev->uart_num, 256, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(dev->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(dev->uart_num, dev->tx_pin, dev->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpio_set_direction(dev->m0_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->m1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->aux_pin, GPIO_MODE_INPUT);

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    return ESP_OK;
}

void lora_send(lora_dev_t *dev, telemetry_packet_t *packet) {
    if (dev == NULL || packet == NULL) return;

    uart_write_bytes(dev->uart_num, (const char *)packet, sizeof(telemetry_packet_t));
}
