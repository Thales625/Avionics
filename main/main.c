#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <stdio.h>
#include <math.h>

#include "sdcard.h"
#include "mpu6050.h"
#include "bmp280.h"

typedef enum {
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
#define PBUTTON_PIN 4

#define SD_CS_PIN 14
#define SD_MISO_PIN 25
#define SD_MOSI_PIN 27
#define SD_SCLK_PIN 26

#define SD_FILE "datalog.txt"
#define SD_SYNC_INTERVAL 300 // ms

#define DESCENT_IGNORE_TIME 10000 // ms
#define DESCENT_MAX_TIME 30000 // ms

// #define DEBUG

static const char *TAG = "avionics";

static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };
static FILE *file_ptr;
static flight_state_t flight_state = STATE_PRE_FLIGHT;

inline static void beep(uint32_t duration) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(BUZZER_PIN, 0);
}

inline static void avionics_abort(void) {
    beep(5000);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500));

    // config GPIO
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

        gpio_set_level(PARACHUTE_PIN, 0);
        gpio_set_level(LED_PIN, 0);
        gpio_set_level(BUZZER_PIN, 0);
    }

    /*
    // GROUND TEST
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (gpio_get_level(PBUTTON_PIN)) vTaskDelay(pdMS_TO_TICKS(200));
    beep(1000);
    vTaskDelay(pdMS_TO_TICKS(60000));
    beep(2000);
    vTaskDelay(pdMS_TO_TICKS(30000));

    beep(1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    beep(1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    beep(1000);
    vTaskDelay(pdMS_TO_TICKS(1000));

    gpio_set_level(PARACHUTE_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(10000));

    return;
    */

    // I2C
    {
        ESP_ERROR_CHECK(i2cdev_init());
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // BMP280
    {
        bmp280_params_t params;
        bmp280_init_default_params(&params);
        ESP_ERROR_CHECK(bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO_PIN, SCL_GPIO_PIN));
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_ERROR_CHECK(bmp280_init(&bmp_dev, &params));
        ESP_LOGI(TAG, "Found BMP280 device");
    }

    // MPU6050
    {
        ESP_ERROR_CHECK(mpu6050_init_desc(&mpu_dev, MPU6050_I2C_ADDRESS_LOW, I2C_NUM_0, SDA_GPIO_PIN, SCL_GPIO_PIN));
        vTaskDelay(pdMS_TO_TICKS(200));
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
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));
        ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_500));
        ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_4));
    }

    // DEBUG
    #ifdef DEBUG
    {
        beep(500);

        mpu6050_acceleration_t accel_ = { 0 };
        mpu6050_rotation_t rotation_ = { 0 };

        float pressure_, temperature_;

        while (1) {
            ESP_ERROR_CHECK(mpu6050_get_motion(&mpu_dev, &accel_, &rotation_));
            ESP_ERROR_CHECK(bmp280_read_float(&bmp_dev, &temperature_, &pressure_));

            printf("%.4f %.4f\n", accel_.x, pressure_);

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    #endif

    // SDCARD
    {
        if (sdcard_init(SD_MOSI_PIN, SD_MISO_PIN, SD_SCLK_PIN, SD_CS_PIN) != ESP_OK) {
            abort();
            sdcard_umount();
            ESP_LOGE(TAG, "Failed to initialize SD card");
            return;
        }
    }

    uint32_t ut, ut0, ut_sd_sync;
    char text[128];

    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };
    float pressure, pressure_0, temperature;
    float altitude_baro=0.0f, max_altitude_baro=0.0f, prev_altitude_baro=0.0f;
    uint32_t parachute_ejection_count=0, descent_stable_count=0;

    // SETUP
    gpio_set_level(LED_PIN, 1);

    beep(500);
    vTaskDelay(pdMS_TO_TICKS(500));
    beep(500);

    // WAIT BUTTON
    while (gpio_get_level(PBUTTON_PIN)) vTaskDelay(pdMS_TO_TICKS(200));

    // SDCARD: open/create file
    if (sdcard_open_file(SD_FILE, "w", &file_ptr) != ESP_OK) {
        abort();
        sdcard_umount();
        ESP_LOGE(TAG, "Failed to open/create file");
        return;
    }

    // ABORT CHECK
    if (bmp280_read_float(&bmp_dev, &temperature, &pressure) != ESP_OK || mpu6050_get_motion(&mpu_dev, &accel, &rotation) != ESP_OK) {
        abort();
        return;
    } 

    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(90000)); // wait 90 sec
    beep(300);

    ESP_ERROR_CHECK(bmp280_read_float(&bmp_dev, &temperature, &pressure_0)); // read initial pressure

    ut0 = (uint32_t)(esp_timer_get_time() / 1000ULL);
    ut_sd_sync = ut0;

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

        // printf("d%s", text);

        // sync sdcard
        if (ut - ut_sd_sync > SD_SYNC_INTERVAL) {
            sdcard_sync(file_ptr);
            ut_sd_sync = ut;
        }

        // state machine
        switch (flight_state) {
            case STATE_PRE_FLIGHT:
                if (accel.x > 1.8f) flight_state = STATE_ASCENT;
                break;

            case STATE_ASCENT:
                if (pressure > 0.0f) {
                    // filter
                    altitude_baro = 0.9f * altitude_baro + 0.1f * (44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f)));

                    // altitude_baro = 44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f));

                    // printf("%.4f\n", altitude_baro); // DEBUG

                    if (altitude_baro > max_altitude_baro) {
                        max_altitude_baro = altitude_baro;
                        parachute_ejection_count = 0; // reset count
                    } else if (max_altitude_baro - altitude_baro >= 1.0f) {
                        parachute_ejection_count++;

                        if (parachute_ejection_count >= 5) { // EJECT
                            flight_state = STATE_PARACHUTE_DEPLOY;
                            ut0 = ut;
                        }
                    }
                }

                break;

            case STATE_PARACHUTE_DEPLOY:
                gpio_set_level(PARACHUTE_PIN, 1);

                if (ut - ut0 < 5000) break;

                gpio_set_level(PARACHUTE_PIN, 0);

                prev_altitude_baro = 0.9f * altitude_baro + 0.1f * (44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f)));
                ut0 = ut;

                flight_state = STATE_DESCENT;
                break;

            case STATE_DESCENT:
                altitude_baro = 0.9f * altitude_baro + 0.1f * (44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f)));

                if (ut - ut0 < DESCENT_IGNORE_TIME) { // ignore first N sec after parachute ejection
                    prev_altitude_baro = altitude_baro;
                    break; 
                }

                if (ut - ut0 > DESCENT_MAX_TIME) { // check max descent time
                    flight_state = STATE_SHUTDOWN;
                    break;
                }

                // printf("%.4f\n", fabsf(altitude_baro - prev_altitude_baro)); // DEBUG

                // check stable baro altitude
                if (fabsf(altitude_baro - prev_altitude_baro) < 0.5f) {
                    descent_stable_count++;
                } else {
                    descent_stable_count = 0;
                }
                
                // |ACC| < MAX and altitude_baro stable
                if (sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z) < 1.1 && descent_stable_count > 3) {
                    flight_state = STATE_SHUTDOWN;
                    break;
                }

                // CHECK BUTTON
                if (!gpio_get_level(PBUTTON_PIN)) flight_state = STATE_SHUTDOWN;

                break;

            case STATE_SHUTDOWN:
                // umount sdcard
                sdcard_close_file(file_ptr);
                sdcard_umount();

                beep(200);

                // blinking LED
                while (gpio_get_level(PBUTTON_PIN)) {
                    gpio_set_level(LED_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    gpio_set_level(LED_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
