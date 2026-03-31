#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <math.h>

#include "math_helper.h"
#include "mpu6050.h"
#include "bmp280.h"

#include "flight_logic.h"
#include "telemetry.h"

#define FLASH_PAGE_SIZE 256
#define PACKETS_PER_PAGE (FLASH_PAGE_SIZE / sizeof(telemetry_packet_t))

#define LORA_SAMPLING 10

#define SDA_GPIO_PIN GPIO_NUM_21
#define SCL_GPIO_PIN GPIO_NUM_22

#define PARACHUTE_PIN GPIO_NUM_18
#define BUZZER_PIN GPIO_NUM_23
#define LED_PIN GPIO_NUM_33
#define PBUTTON_PIN GPIO_NUM_4

static const char* TAG = "avionics";

static flight_logic_t flight_logic;

static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };

static QueueHandle_t telemetry_queue;

static void beep(uint32_t duration) {
    gpio_set_level(BUZZER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(BUZZER_PIN, 1);
}

static void avionics_abort(int code) {
    // set safe state
    gpio_set_level(PARACHUTE_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 1);

    // abort loop
    while (1) {
        for (int i=0; i<code; i++) {
            beep(100);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void avionics_task(void *arg) {
    // startup delay + buzzer indication
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(8000));
    beep(500);

    // startup indication
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5 sec
    beep(300);

    // init flight logic core
    flight_logic.state.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);
    flight_logic_init(&flight_logic);

    // frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10 ms

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

        // update flight logic
        flight_logic_update(&flight_logic);

        // set values
        gpio_set_level(PARACHUTE_PIN, flight_logic.trigger_parachute);
    }
    vTaskDelete(NULL);
}

static void telemetry_task(void *arg) {
    telemetry_packet_t page_buffer[5]; // memory write buffer

    telemetry_packet_t packet;
    flight_state_t sample; // queue sample

    int offset = 0;
    int lora_counter = 0;

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

            packet.checksum = 10; // TODO

            // send via LoRa
            if (lora_counter++ > LORA_SAMPLING) {
                lora_counter = 0;
                // lora_send(packet);
            }

            // add to RAM buffer
            page_buffer[offset++] = packet;

            if (offset >= 6) {
                // flash_write_page(buffer);
                offset = 0;
            }
        }
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    // Create xQueue
    telemetry_queue = xQueueCreate(32, sizeof(flight_state_t));

    // GPIO configuration
    {
        gpio_config_t in_conf = {
            .pin_bit_mask = (1ULL << PBUTTON_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&in_conf));
        
        gpio_config_t out_conf = {
            .pin_bit_mask = (1ULL << PARACHUTE_PIN) | 
                            (1ULL << LED_PIN) | 
                            (1ULL << BUZZER_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&out_conf));
    }

    // I2C initialization
    {
        ESP_ERROR_CHECK(i2cdev_init());
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // BMP280 initialization
    {
        bmp280_params_t params;
        bmp280_init_default_params(&params);

        ESP_ERROR_CHECK(
            bmp280_init_desc(
                &bmp_dev,
                BMP280_I2C_ADDRESS_0,
                I2C_NUM_0,
                SDA_GPIO_PIN,
                SCL_GPIO_PIN
            )
        );
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_ERROR_CHECK(bmp280_init(&bmp_dev, &params));
        ESP_LOGI(TAG, "Found BMP280");
    }

    // MPU6050 initialization
    {
        ESP_ERROR_CHECK(
            mpu6050_init_desc(
                &mpu_dev,
                MPU6050_I2C_ADDRESS_LOW,
                I2C_NUM_0,
                SDA_GPIO_PIN,
                SCL_GPIO_PIN
            )
        );
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));
        ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_500));
        ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_4));
        ESP_LOGI(TAG, "Found MPU6050");
    }

    // set default values
    gpio_set_level(PARACHUTE_PIN, 0);
    gpio_set_level(LED_PIN, 1);
    gpio_set_level(BUZZER_PIN, 1);

    // create tasks
    xTaskCreate(avionics_task, "avionics_task", 4096, NULL, 10, NULL);
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 10, NULL);
}
