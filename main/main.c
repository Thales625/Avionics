#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <stdio.h>

#include "sdcard.h"
#include "mpu6050.h"
#include "bmp280.h"

typedef enum {
    STATE_SETUP,
    STATE_PRE_FLIGHT,
    STATE_ASCENT,
    STATE_PARACHUTE_DEPLOY,
    STATE_DESCENT,
    STATE_SHUTDOWN
} flight_state_t;

#define SDA_GPIO_PIN 21
#define SCL_GPIO_PIN 22

#define PARACHUTE_PIN 18
#define BUZZER_PIN 23
#define LED_PIN 33

#define SD_CS_PIN 14
#define SD_MISO_PIN 27
#define SD_MOSI_PIN 26
#define SD_SCLK_PIN 25

#define SD_FILE "datalog.txt"

static const char *TAG = "avionics";

static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };
static FILE *file_ptr;
static flight_state_t flight_state = STATE_SETUP;

inline static void beep(uint32_t duration) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(BUZZER_PIN, 0);
}

void app_main(void) {
    // flash delay
    vTaskDelay(pdMS_TO_TICKS(5000));

    uint32_t ut;
    char text[128];

    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };
    float pressure, temperature;

    // config GPIO
    {
        ESP_ERROR_CHECK(i2cdev_init());
        
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << PARACHUTE_PIN) | 
                            (1ULL << LED_PIN) | 
                            (1ULL << BUZZER_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        gpio_set_level(PARACHUTE_PIN, 0);
        gpio_set_level(LED_PIN, 0);
        gpio_set_level(BUZZER_PIN, 0);
    }

    // BMP280
    {
        bmp280_params_t params;
        bmp280_init_default_params(&params);
        ESP_ERROR_CHECK(bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO_PIN, SCL_GPIO_PIN));
        ESP_ERROR_CHECK(bmp280_init(&bmp_dev, &params));
        ESP_LOGI(TAG, "Found BMP280 device");
    }

    // MPU6050
    {
        ESP_ERROR_CHECK(mpu6050_init_desc(&mpu_dev, MPU6050_I2C_ADDRESS_LOW, 0, SDA_GPIO_PIN, SCL_GPIO_PIN));
        esp_err_t res;
        while (1) {
            res = i2c_dev_probe(&mpu_dev.i2c_dev, I2C_DEV_WRITE);
            if (res == ESP_OK) {
                ESP_LOGI(TAG, "Found MPU6050 device");
                break;
            }
            ESP_LOGE(TAG, "MPU6050 not found");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));
        ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_500));
        ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_4));
    }
   
    // SDCARD
    {
        if (sdcard_init(SD_MOSI_PIN, SD_MISO_PIN, SD_SCLK_PIN, SD_CS_PIN) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SD card");
            return;
        }

        // open/create file
        if (sdcard_open_file(SD_FILE, "w", &file_ptr) != ESP_OK) {
            sdcard_umount();
            ESP_LOGE(TAG, "Failed to open/create file");
            return;
        }
    }

    // main loop
    while (1) {
        ut = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // read mpu6050
        ESP_ERROR_CHECK(mpu6050_get_motion(&mpu_dev, &accel, &rotation));

        // read bmp280
        ESP_ERROR_CHECK(bmp280_read_float(&bmp_dev, &temperature, &pressure));
        
        // write sdcard
		snprintf(text, sizeof(text), "%d %ld %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.2f\n", flight_state, ut, accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, pressure, temperature);
		sdcard_write(text, file_ptr);
        printf("d%s", text);
        // printf("d%.4f %.4f %.4f %.4f %.4f %.4f ", accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z); // MPU6050
        // printf("%.4f %.2f\n", pressure, temperature); // BMP280

        // state machine
        switch (flight_state) {
            case STATE_SETUP: // single itteration
                flight_state = STATE_PRE_FLIGHT;
                gpio_set_level(LED_PIN, 1);
                beep(500);
                vTaskDelay(pdMS_TO_TICKS(500));
                beep(500);
                break;

            case STATE_PRE_FLIGHT:
                 // TODO
                break;

            case STATE_ASCENT:
                 // TODO
                break;

            case STATE_PARACHUTE_DEPLOY:
                // TODO
                flight_state = STATE_DESCENT;
                break;

            case STATE_DESCENT:
                // TODO
                break;

            case STATE_SHUTDOWN:
                gpio_set_level(LED_PIN, 0);
                beep(1000);
                vTaskDelete(NULL);

                // umount sdcard
                #ifndef DEBUG
                sdcard_close_file(file_ptr);
                sdcard_umount();
                #endif

                return;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
