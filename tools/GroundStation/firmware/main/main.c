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
static uint8_t bytes[256];

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

    // uint8_t recv_data;

    while (1) {
        #ifdef DEBUG
        // generate random data
        packet.magic = TELEMETRY_MAGIC;
        packet.ut = (uint32_t)(esp_timer_get_time() / 1000);
        packet.phase = PHASE_ASCENT;
        packet.accel.x = (float)(esp_random() % 100) / 100.0f;
        packet.accel.y = (float)(esp_random() % 100) / 100.0f;
        packet.accel.z = 9.81f + ((float)(esp_random() % 100) / 100.0f);
        packet.ang_vel.x = 0.1f;
        packet.ang_vel.y = -0.2f;
        packet.ang_vel.z = 0.0f;
        packet.pressure = 101325.0f - (esp_random() % 5000);
        packet.temperature = 25.0f;

        size_t payload_size = sizeof(telemetry_packet_t) - sizeof(uint16_t);
        packet.checksum = crc16((const uint8_t*)&packet, payload_size);
        #endif

        // read lora
        int available_bytes = lora_receive_bytes(&lora_dev, bytes, sizeof(bytes));
        if (available_bytes > 0) {
            // sends the raw binary packet via USB UART
            uart_write_bytes(PORT_USB, (const void*)bytes, available_bytes);
        }

        // read data (ToDo)
        // int len = uart_read_bytes(PORT_USB, &recv_data, 1, TIMEOUT_LOOP);
        
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
