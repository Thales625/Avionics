#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/uart.h"

#include <unistd.h>

#include "telemetry.h"
#include "flight_logic.h"

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

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
        packet.checksum = 0;

        // sends the raw binary block via USB UART
        uart_write_bytes(UART_NUM_0, &packet, sizeof(telemetry_packet_t));
        
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

