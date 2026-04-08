#include "esp_err.h"
#include "driver/gpio.h"

#include "lora.h"

static inline void lora_wait_aux(lora_dev_t *dev) {
    while (gpio_get_level(dev->aux_pin) == 0);
}

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

    vTaskDelay(pdMS_TO_TICKS(100));
    lora_wait_aux(dev);

    return ESP_OK;
}

esp_err_t lora_set_rssi(lora_dev_t *dev, bool enable) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    // config mode
    gpio_set_level(dev->m0_pin, 1);
    gpio_set_level(dev->m1_pin, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    lora_wait_aux(dev);

    // clear uart buffer
    uart_flush_input(dev->uart_num);

    // send read command
    uint8_t read_cmd[3] = {0xC1, 0x05, 0x01};
    uart_write_bytes(dev->uart_num, (const uint8_t *)read_cmd, 3);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    lora_wait_aux(dev);

    // read response
    uint8_t response[4] = { 0 };
    int len = uart_read_bytes(dev->uart_num, response, 4, pdMS_TO_TICKS(500));

    if (len != 4 || response[0] != 0xC1 || response[1] != 0x05) {
        // communication failed: back to normal mode
        gpio_set_level(dev->m0_pin, 0);
        gpio_set_level(dev->m1_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        lora_wait_aux(dev);
        return ESP_FAIL;
    }

    uint8_t reg3_val = response[3];
    if (enable) {
        reg3_val |= 0x80;
    } else {
        reg3_val &= 0x7F;
    }

    // write new value
    uint8_t write_cmd[4] = {0xC0, 0x05, 0x01, reg3_val};
    uart_write_bytes(dev->uart_num, (const uint8_t *)write_cmd, 4);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    lora_wait_aux(dev);

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    lora_wait_aux(dev);

    return ESP_OK;
}


void lora_send_bytes(lora_dev_t *dev, uint8_t *bytes, size_t size) {
    if (dev == NULL) return;

    // LoRa is busy
    if (gpio_get_level(dev->aux_pin) == 0) return;

    uart_write_bytes(dev->uart_num, (const uint8_t *)bytes, size);
}

int lora_receive_bytes(lora_dev_t *dev, uint8_t *bytes, size_t size) {
    if (dev == NULL) return -1;
    
    size_t available_bytes = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(dev->uart_num, &available_bytes));

    if (available_bytes > 0) {
        return uart_read_bytes(dev->uart_num, bytes, available_bytes > size ? size : available_bytes, 0);
    }

    return 0;
}
