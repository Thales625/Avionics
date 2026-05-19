#ifndef __LORA_H__
#define __LORA_H__

#include "driver/uart.h" // IWYU pragma: keep

typedef struct {
    int tx_pin;
    int rx_pin;
    int m0_pin;
    int m1_pin;
    int aux_pin;
    uart_port_t uart_num;
    uint32_t baud_rate;
    uint8_t channel;
} lora_dev_t;

typedef enum {
    LORA_POWER_22_DBM = 0,
    LORA_POWER_17_DBM = 1,
    LORA_POWER_13_DBM = 2,
    LORA_POWER_10_DBM = 3,
} lora_power_t;

esp_err_t lora_init(lora_dev_t *dev);
esp_err_t lora_set_channel(lora_dev_t *dev, uint8_t channel);
esp_err_t lora_set_rssi(lora_dev_t *dev, bool enable);
esp_err_t lora_set_power(lora_dev_t *dev, lora_power_t power);

void lora_send_bytes(lora_dev_t *dev, uint8_t *bytes, size_t size);
int lora_receive_bytes(lora_dev_t *dev, uint8_t *bytes, size_t size, TickType_t timeout);

#endif
