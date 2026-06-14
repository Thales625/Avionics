#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <unistd.h>

#include "lora.h"
#include "tmtc.h"

#define BAUD_RATE 115200
#define PORT_USB UART_NUM_0

#define LORA_TX GPIO_NUM_16
#define LORA_RX GPIO_NUM_17
#define LORA_M0 GPIO_NUM_4
#define LORA_M1 GPIO_NUM_19
#define LORA_AUX GPIO_NUM_34
#define LORA_UART UART_NUM_2
#define LORA_BAUD_RATE 9600

static lora_dev_t lora_dev = { 0 };

static uint8_t rx_bytes[256];
static uint8_t tx_bytes[256];

void app_main(void) {
    // init lora
    {
        lora_dev.tx_pin = LORA_RX;
        lora_dev.rx_pin = LORA_TX;
        lora_dev.m0_pin = LORA_M0;
        lora_dev.m1_pin = LORA_M1;
        lora_dev.aux_pin = LORA_AUX;
        lora_dev.uart_num = LORA_UART;
        lora_dev.baud_rate = LORA_BAUD_RATE;
        lora_dev.channel = TMTC_CHANNEL;
        ESP_ERROR_CHECK(lora_init(&lora_dev));
        lora_set_rssi(&lora_dev, true);
        lora_set_channel(&lora_dev, TMTC_CHANNEL);
        // lora_set_power(&lora_dev, LORA_POWER_22_DBM);
        // lora_set_power(&lora_dev, LORA_POWER_17_DBM);
        lora_set_power(&lora_dev, LORA_POWER_13_DBM);
        lora_set_air_data_rate(&lora_dev, TMTC_AIR_DATA_RATE);
    }

    // init usb
    {
        uart_config_t uart_config = {
            .baud_rate = BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

        uart_driver_install(PORT_USB, 512, 0, 0, NULL, 0);
        uart_param_config(PORT_USB, &uart_config);
        uart_set_pin(PORT_USB, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    // uint8_t usb_recv_data;

    while (1) {
        // read LORA and transmit to USB
        int available_bytes = lora_receive_bytes(&lora_dev, rx_bytes, sizeof(rx_bytes), 0);
        if (available_bytes > 0) {
            // send lora bytes to USB
            uart_write_bytes(PORT_USB, (const void*)rx_bytes, available_bytes);
        }

        // read USB PORT and transmit to LORA
        int usb_len = uart_read_bytes(PORT_USB, tx_bytes, sizeof(tx_bytes), pdMS_TO_TICKS(5));
        if (usb_len > 0) {
            // send usb bytes to LORA
            lora_send_bytes(&lora_dev, tx_bytes, usb_len);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
