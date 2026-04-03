#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/uart.h"

#include <unistd.h>

#include "telemetry.h"
#include "flight_logic.h"

#define BAUD_RATE 115200 
#define PORT_USB UART_NUM_0

// CRC-16-CCITT
static uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void app_main(void) {
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

    telemetry_packet_t packet;
    packet.magic = TELEMETRY_MAGIC;

    while (1) {
        // generate random data
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

        // sends the raw binary block via USB UART
        uart_write_bytes(PORT_USB, (const void*)&packet, sizeof(telemetry_packet_t));
        
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

