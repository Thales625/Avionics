#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "lora.h"

bool lora_wait_aux(lora_dev_t *dev, TickType_t timeout) {
    TickType_t start = xTaskGetTickCount();

    while (gpio_get_level(dev->aux_pin) == 0) {
        if ((xTaskGetTickCount() - start) > timeout) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return true;
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

    esp_err_t err;

    // buffer RX = 256 bytes | buffer TX = 256 bytes
    err = uart_driver_install(dev->uart_num, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(dev->uart_num, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(dev->uart_num, dev->tx_pin, dev->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    gpio_set_direction(dev->m0_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->m1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->aux_pin, GPIO_MODE_INPUT);

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(200));

    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) {
        return ESP_FAIL;
    };

    return ESP_OK;
}

esp_err_t lora_set_channel(lora_dev_t *dev, uint8_t channel) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    // config mode
    gpio_set_level(dev->m0_pin, 1);
    gpio_set_level(dev->m1_pin, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // clear uart buffer
    uart_flush_input(dev->uart_num);

    // channel reg (0x04)
    uint8_t write_cmd[4] = {0xC2, 0x04, 0x01, channel};
    uart_write_bytes(dev->uart_num, (const uint8_t *)write_cmd, 4);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t lora_set_rssi(lora_dev_t *dev, bool enable) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    // config mode
    gpio_set_level(dev->m0_pin, 1);
    gpio_set_level(dev->m1_pin, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // clear uart buffer
    uart_flush_input(dev->uart_num);

    // send read command
    uint8_t read_cmd[3] = {0xC1, 0x05, 0x01};
    uart_write_bytes(dev->uart_num, (const uint8_t *)read_cmd, 3);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // read response
    uint8_t response[4] = { 0 };
    int len = uart_read_bytes(dev->uart_num, response, 4, pdMS_TO_TICKS(500));

    if (len != 4 || response[0] != 0xC1 || response[1] != 0x05) {
        // communication failed: back to normal mode
        gpio_set_level(dev->m0_pin, 0);
        gpio_set_level(dev->m1_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;
        return ESP_FAIL;
    }

    uint8_t reg3_val = response[3];
    if (enable) {
        reg3_val |= 0x80;
    } else {
        reg3_val &= 0x7F;
    }

    // write new value
    uint8_t write_cmd[4] = {0xC2, 0x05, 0x01, reg3_val};
    uart_write_bytes(dev->uart_num, (const uint8_t *)write_cmd, 4);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t lora_set_air_data_rate(lora_dev_t *dev, lora_air_data_rate_t rate) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    // config mode
    gpio_set_level(dev->m0_pin, 1);
    gpio_set_level(dev->m1_pin, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // clear uart buffer
    uart_flush_input(dev->uart_num);

    // read REG0 command
    uint8_t read_cmd[3] = {0xC1, 0x02, 0x01};
    uart_write_bytes(dev->uart_num, (const uint8_t *)read_cmd, 3);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // read response (header + address + length + value)
    uint8_t response[4] = {0};
    int len = uart_read_bytes(dev->uart_num, response, 4, pdMS_TO_TICKS(500));

    if (len != 4 || response[0] != 0xC1 || response[1] != 0x02) {
        // communication failed: back to normal mode
        gpio_set_level(dev->m0_pin, 0);
        gpio_set_level(dev->m1_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;
        return ESP_FAIL;
    }

    uint8_t reg0_val = response[3];

    // clear bits [2:0] (Air Data Rate)
    reg0_val &= 0xF8;

    // set new air data rate
    reg0_val |= (rate & 0x07);

    // write REG0
    uint8_t write_cmd[4] = {0xC2, 0x02, 0x01, reg0_val};
    uart_write_bytes(dev->uart_num, (const uint8_t *)write_cmd, 4);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t lora_set_power(lora_dev_t *dev, lora_power_t power) {
    if (dev == NULL) return ESP_ERR_INVALID_ARG;

    // config mode
    gpio_set_level(dev->m0_pin, 1);
    gpio_set_level(dev->m1_pin, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // clear uart buffer
    uart_flush_input(dev->uart_num);

    // read REG1 command
    uint8_t read_cmd[3] = {0xC1, 0x03, 0x01};
    uart_write_bytes(dev->uart_num, (const uint8_t *)read_cmd, 3);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // read response
    uint8_t response[4] = {0};

    int len = uart_read_bytes(dev->uart_num, response, 4, pdMS_TO_TICKS(500));

    if (len != 4 || response[0] != 0xC1 || response[1] != 0x03) {
        // communication failed: back to normal mode
        gpio_set_level(dev->m0_pin, 0);
        gpio_set_level(dev->m1_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;
        return ESP_FAIL;
    }

    uint8_t reg1_val = response[3];

    // clear power bits [1:0]
    reg1_val &= 0xFC;

    // set new power
    reg1_val |= (power & 0x03);

    // write REG1 (0x03)
    uint8_t write_cmd[4] = {0xC2, 0x03, 0x01, reg1_val};
    uart_write_bytes(dev->uart_num, (const uint8_t *)write_cmd, 4);
    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(50));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    // normal mode
    gpio_set_level(dev->m0_pin, 0);
    gpio_set_level(dev->m1_pin, 0);

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) return ESP_FAIL;

    return ESP_OK;
}


int lora_send_bytes(lora_dev_t *dev, uint8_t *bytes, size_t size) {
    if (dev == NULL) return 0;

    if (!lora_wait_aux(dev, pdMS_TO_TICKS(1000))) { // lora aux timeout
        return -1;
    }

    int written = uart_write_bytes(dev->uart_num, bytes, size);

    uart_wait_tx_done(dev->uart_num, pdMS_TO_TICKS(500));

    if (!lora_wait_aux(dev, pdMS_TO_TICKS(2000))) {
        return -1;
    }

    return written;
}

int lora_receive_bytes(lora_dev_t *dev, uint8_t *bytes, size_t size, TickType_t timeout) {
    if (dev == NULL) return -1;

    return uart_read_bytes(dev->uart_num, bytes, size, timeout);
}
