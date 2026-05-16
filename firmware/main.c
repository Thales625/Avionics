#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <math.h>

#include "math_helper.h"
#include "flight_logic.h"
#include "tmtc.h"

#include "mpu6050.h"
#include "bmp280.h"
#include "lora.h"
#include "gps.h"
#include "w25q64.h"

#define FLASH_PAGE_SIZE 256
#define PACKETS_PER_PAGE (FLASH_PAGE_SIZE / sizeof(telemetry_packet_t))
#define BYTES_PER_PAGE (PACKETS_PER_PAGE * sizeof(telemetry_packet_t))

#define SPI_MISO GPIO_NUM_25
#define SPI_MOSI GPIO_NUM_27
#define SPI_CLK GPIO_NUM_26
#define W25Q_CS GPIO_NUM_14

#define GPS_TX GPIO_NUM_35
#define GPS_RX GPIO_NUM_32
#define GPS_UART UART_NUM_1

#define LORA_TX GPIO_NUM_16
#define LORA_RX GPIO_NUM_17
#define LORA_M0 GPIO_NUM_4
#define LORA_M1 GPIO_NUM_19
#define LORA_AUX GPIO_NUM_34
#define LORA_UART UART_NUM_2
#define LORA_BAUD_RATE 9600
#define LORA_SAMPLING 5

#define SDA_GPIO_PIN GPIO_NUM_21
#define SCL_GPIO_PIN GPIO_NUM_22

#define PARACHUTE_PIN GPIO_NUM_18
#define LED_PIN GPIO_NUM_2

#define BOOT_PIN GPIO_NUM_0

static bool packet_is_empty(const telemetry_packet_t *pkt) {
    const uint8_t *p = (const uint8_t *)pkt;

    for (size_t i = 0; i < sizeof(telemetry_packet_t); i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

static const char* TAG = "avionics";

static flight_logic_t flight_logic;

static lora_dev_t lora_dev = { 0 };
static gps_dev_t gps_dev = { 0 };
static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };

static QueueHandle_t telemetry_queue;

static void blink(uint32_t duration) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(LED_PIN, 0);
}

static void avionics_abort(int code) {
    // set safe state
    gpio_set_level(PARACHUTE_PIN, 0);
    gpio_set_level(LED_PIN, 0);

    // log
    ESP_LOGW(TAG, "ABORT! code: %d", code);

    // abort loop
    while (1) {
        for (int i=-1; i<code; i++) {
            blink(100);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void avionics_task(void *arg) {
    // startup indication
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(LED_PIN, 0);

    // init flight logic core
    flight_logic.state.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (mpu6050_get_motion(&mpu_dev, &flight_logic.state.accel, &flight_logic.state.ang_vel) != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050: initial reading failed");
        avionics_abort(3);
    }
    if (bmp280_read_float(&bmp_dev, &flight_logic.state.temperature, &flight_logic.state.pressure) != ESP_OK) {
        ESP_LOGW(TAG, "BMP280: initial reading failed");
        avionics_abort(3);
    }
    flight_logic_init(&flight_logic);

    // frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // ms

    // main loop
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // update ut
        flight_logic.state.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // MPU6050: read data
        if (mpu6050_get_motion(&mpu_dev, &flight_logic.state.accel, &flight_logic.state.ang_vel) != ESP_OK) {
            ESP_LOGW(TAG, "MPU6050: read failed");

            if (flight_logic.state.phase == PHASE_PRE_FLIGHT) {
                avionics_abort(3);
            }

            flight_logic.state.accel.x = NAN;
            flight_logic.state.accel.y = NAN;
            flight_logic.state.accel.z = NAN;

            flight_logic.state.ang_vel.x = NAN;
            flight_logic.state.ang_vel.y = NAN;
            flight_logic.state.ang_vel.z = NAN;
        }

        // BMP280: read data
        if (bmp280_read_float(&bmp_dev, &flight_logic.state.temperature, &flight_logic.state.pressure) != ESP_OK) {
            ESP_LOGW(TAG, "BMP280: read failed");

            if (flight_logic.state.phase == PHASE_PRE_FLIGHT) {
                avionics_abort(3);
            }

            flight_logic.state.temperature = NAN;
            flight_logic.state.pressure = NAN;
        }

        // GPS: read data
        gps_read(&gps_dev, &flight_logic.state.lat_nmea, &flight_logic.state.lon_nmea, &flight_logic.state.satellites);

        // update flight logic
        flight_logic_update(&flight_logic);

        // set values
        gpio_set_level(PARACHUTE_PIN, flight_logic.trigger_parachute);

        // log values
        ESP_LOGI(TAG, "phase: %d, altitude: %.2f, pressure: %.2f, |accel|: %.2f, sats: %d, lat: %d, lon: %d", flight_logic.state.phase, flight_logic.altitude_baro, flight_logic.state.pressure, sqrtf(flight_logic.state.accel.x*flight_logic.state.accel.x + flight_logic.state.accel.y*flight_logic.state.accel.y + flight_logic.state.accel.z*flight_logic.state.accel.z), flight_logic.state.satellites, flight_logic.state.lat_nmea, flight_logic.state.lon_nmea);

        // send data to telemetry
        xQueueSend(telemetry_queue, &flight_logic.state, 0);
    }
    vTaskDelete(NULL);
}

static void telemetry_task(void *arg) {
    // flash memory
    // telemetry_packet_t page_buffer[PACKETS_PER_PAGE]; // write buffer
    // uint32_t offset = 0;
    // uint32_t current_flash_addr = 0;
    // int32_t last_erased_sector = -1;

    // LoRa
    telemetry_packet_t packet;
    flight_state_t sample; // queue sample
    uint32_t lora_counter = 0;

    while (1) {
        if (xQueueReceive(telemetry_queue, &sample, portMAX_DELAY)) {
            // populate packet
            packet.magic = TELEMETRY_MAGIC;

            packet.ut = sample.ut;
            packet.phase = (uint8_t) sample.phase;
            packet.accel = sample.accel;
            packet.ang_vel = sample.ang_vel;
            packet.pressure = sample.pressure;
            packet.temperature = sample.temperature;
            packet.lat_nmea = sample.lat_nmea;
            packet.lon_nmea = sample.lon_nmea;
            packet.satellites = sample.satellites;

            packet.checksum = crc16((const uint8_t*)&packet, sizeof(telemetry_packet_t) - sizeof(uint16_t));

            // send via LoRa
            if (lora_counter++ >= LORA_SAMPLING) {
                lora_counter = 0;
                lora_send_bytes(&lora_dev, (uint8_t *)&packet, sizeof(packet));
            }

            // add to flash mem buffer
            /*
            page_buffer[offset++] = packet;

            if (offset >= PACKETS_PER_PAGE) {
                uint32_t start_sector = current_flash_addr / W25Q64_SECTOR_SIZE;
                uint32_t end_sector = (current_flash_addr + BYTES_PER_PAGE - 1) / W25Q64_SECTOR_SIZE;

                if (start_sector > last_erased_sector) {
                    w25q64_erase_sector(start_sector * W25Q64_SECTOR_SIZE);
                    last_erased_sector = start_sector;
                }

                if (end_sector > last_erased_sector) {
                    w25q64_erase_sector(end_sector * W25Q64_SECTOR_SIZE);
                    last_erased_sector = end_sector;
                }

                // write page buffer
                if (w25q64_write_data(current_flash_addr, (const uint8_t *)page_buffer, BYTES_PER_PAGE) == ESP_OK) {
                    ESP_LOGI(TAG, "wrote page buffer at 0x%06lX", current_flash_addr);
                } else {
                    ESP_LOGE(TAG, "fail to write!");
                }

                current_flash_addr += BYTES_PER_PAGE;
                offset = 0;
            }
            */
        }
    }

    vTaskDelete(NULL);
}

static void read_telemetry(void *arg) {
    telemetry_packet_t pkt;
    uint32_t address = 0x000000;

    for (int i = 0; i < 32; i++) {
        esp_err_t err = w25q64_read_data(
            address,
            (uint8_t *)&pkt,
            sizeof(telemetry_packet_t)
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read packet %d", i);
            break;
        }

        if (packet_is_empty(&pkt)) {
            ESP_LOGI(TAG, "Reached empty flash area at packet %d", i);
            break;
        }

        ESP_LOGI(TAG, "========== PACKET %d ==========", i);

        ESP_LOGI(TAG, "Address: 0x%06" PRIX32, address);

        ESP_LOGI(TAG, "magic:       0x%08" PRIX32, pkt.magic);
        ESP_LOGI(TAG, "ut:          %" PRIu32, pkt.ut);
        ESP_LOGI(TAG, "phase:       %u", pkt.phase);

        ESP_LOGI(TAG, "accel:       X=%.2f  Y=%.2f  Z=%.2f",
            pkt.accel.x,
            pkt.accel.y,
            pkt.accel.z);

        ESP_LOGI(TAG, "ang_vel:     X=%.2f  Y=%.2f  Z=%.2f",
            pkt.ang_vel.x,
            pkt.ang_vel.y,
            pkt.ang_vel.z);

        ESP_LOGI(TAG, "pressure:    %.2f", pkt.pressure);
        ESP_LOGI(TAG, "temperature: %.2f", pkt.temperature);

        ESP_LOGI(TAG, "lat:         %.2lf", pkt.lat_nmea);
        ESP_LOGI(TAG, "lon:         %.2lf", pkt.lon_nmea);
        ESP_LOGI(TAG, "satellites:  %d", pkt.satellites);

        ESP_LOGI(TAG, "checksum:    0x%04X", pkt.checksum);

        address += sizeof(telemetry_packet_t);
    }

    ESP_LOGI(TAG, "Done reading packets");

    vTaskDelete(NULL);
}

void app_main(void) {
    // Create xQueue
    telemetry_queue = xQueueCreate(32, sizeof(flight_state_t));

    // BOOT BUTTON configuration
    {
        gpio_config_t in_conf = {
            .pin_bit_mask = (1ULL << BOOT_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        if(gpio_config(&in_conf) != ESP_OK) {
            ESP_LOGE(TAG, "BOOT BUTTON failed to init");
            avionics_abort(1);
        }
        ESP_LOGI(TAG, "BOOT BUTTON initialized");
    }

    // GPIO configuration
    {
        gpio_config_t out_conf = {
            .pin_bit_mask = (1ULL << PARACHUTE_PIN) |
                            (1ULL << LED_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        if(gpio_config(&out_conf) != ESP_OK) {
            ESP_LOGE(TAG, "GPIO failed to init");
            avionics_abort(1);
        }
        ESP_LOGI(TAG, "GPIO initialized");

        // set default values
        gpio_set_level(PARACHUTE_PIN, 0);
        gpio_set_level(LED_PIN, 0);
    }

    // W25Q64 initialization
    {
        if (w25q64_init(SPI_MOSI, SPI_MISO, SPI_CLK, W25Q_CS) != ESP_OK) {
            ESP_LOGE(TAG, "W25Q64 failed to init");
            avionics_abort(2);
        }
        ESP_LOGI(TAG, "W25Q64 initialized");
    }

    // check boot button
    {
        gpio_set_level(LED_PIN, 1); // LED on

        ESP_LOGI(TAG, "Waiting for boot button press...");

        bool boot_pressed = false;

        // 5 seconds
        TickType_t start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(5000)) {
            if (gpio_get_level(BOOT_PIN) == 0) {
                // simple debounce
                vTaskDelay(pdMS_TO_TICKS(30));

                if (gpio_get_level(BOOT_PIN) == 0) {
                    boot_pressed = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        gpio_set_level(LED_PIN, 0); // LED off

        // read flash memory
        if (boot_pressed) {
            ESP_LOGI(TAG, "Boot button pressed");
            xTaskCreate(read_telemetry, "read_telemetry", 4096, NULL, 2, NULL);
            return;
        }

        ESP_LOGI(TAG, "Boot button not pressed");
    }

    // I2C initialization
    {
        if (i2cdev_init() != ESP_OK) {
            ESP_LOGE(TAG, "I2C failed to init");
            avionics_abort(3);
        }
        ESP_LOGI(TAG, "I2C initialized");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // LoRa initialization
    {
        lora_dev.tx_pin = LORA_RX;
        lora_dev.rx_pin = LORA_TX;
        lora_dev.m0_pin = LORA_M0;
        lora_dev.m1_pin = LORA_M1;
        lora_dev.aux_pin = LORA_AUX;
        lora_dev.uart_num = LORA_UART;
        lora_dev.baud_rate = LORA_BAUD_RATE;
        lora_dev.channel = TMTC_CHANNEL;
        if (lora_init(&lora_dev) != ESP_OK) {
            ESP_LOGE(TAG, "LoRa failed to init");
            avionics_abort(4);
        }
        lora_set_power(&lora_dev, LORA_POWER_17_DBM);
        // lora_set_power(&lora_dev, LORA_POWER_13_DBM);
        lora_set_channel(&lora_dev, TMTC_CHANNEL);
        ESP_LOGI(TAG, "LoRa initialized");
    }

    // GPS initialization
    {
        if(gps_init_desc(&gps_dev, GPS_TX, GPS_RX, GPS_UART) != ESP_OK) {
            ESP_LOGE(TAG, "GPS failed to init");
            avionics_abort(5);
        };
        ESP_LOGI(TAG, "GPS initialized");
    }

    // BMP280 initialization
    {
        bmp280_params_t params;
        bmp280_init_default_params(&params);

        bmp280_init_desc(
            &bmp_dev,
            BMP280_I2C_ADDRESS_0,
            I2C_NUM_0,
            SDA_GPIO_PIN,
            SCL_GPIO_PIN
        );
        vTaskDelay(pdMS_TO_TICKS(200));
        if (bmp280_init(&bmp_dev, &params) != ESP_OK) {
            ESP_LOGE(TAG, "BMP280 failed to init");
            avionics_abort(6);
        }
        ESP_LOGI(TAG, "BMP280 initialized");
    }

    // MPU6050 initialization
    {
        mpu6050_init_desc(
            &mpu_dev,
            MPU6050_I2C_ADDRESS_LOW,
            I2C_NUM_0,
            SDA_GPIO_PIN,
            SCL_GPIO_PIN
        );
        vTaskDelay(pdMS_TO_TICKS(200));
        if (mpu6050_init(&mpu_dev) || mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_500) || mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_4)) {
            ESP_LOGE(TAG, "MPU6050 failed to init");
            avionics_abort(7);
        }
        ESP_LOGI(TAG, "MPU6050 initialized");
    }

    // create tasks
    xTaskCreatePinnedToCore(avionics_task, "avionics_task", 4096, NULL, 10, NULL, 1); // APP_CPU
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL, 0); // PRO_CPU
}