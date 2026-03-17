#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <stdio.h>

#include "sdcard.h"
#include "mpu6050.h"
#include "bmp280.h"

#include "flight_logic.h"

#define SDA_GPIO_PIN GPIO_NUM_21
#define SCL_GPIO_PIN GPIO_NUM_22

#define PARACHUTE_PIN GPIO_NUM_18
#define BUZZER_PIN GPIO_NUM_23
#define LED_PIN GPIO_NUM_33
#define PBUTTON_PIN GPIO_NUM_4

#define SD_CS_PIN GPIO_NUM_14
#define SD_MISO_PIN GPIO_NUM_25
#define SD_MOSI_PIN GPIO_NUM_27
#define SD_SCLK_PIN GPIO_NUM_26

#define SD_FILE "datalog.txt"
#define SD_SYNC_INTERVAL 300 // ms

static const char* TAG = "avionics";

static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };
static FILE *file_ptr;

static uint32_t ut_sd_sync;
static flight_logic_t flight_logic;
// char text[128];

static void beep(uint32_t duration) {
    gpio_set_level(BUZZER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(BUZZER_PIN, 1);
}

inline static vector3f_t mpu_convert_accel(mpu6050_acceleration_t accel) {
    return (vector3f_t) {
        .x = accel.x,
        .y = accel.y,
        .z = accel.z
    };
}

inline static vector3f_t mpu_convert_gyro(mpu6050_rotation_t rot) {
    return (vector3f_t) {
        .x = rot.x,
        .y = rot.y,
        .z = rot.z
    };
}

void app_main(void) {
    // read sensors variables
    mpu6050_acceleration_t mpu_acc = { 0 };
    mpu6050_rotation_t mpu_gyro = { 0 };

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
        gpio_set_level(LED_PIN, 1);
        gpio_set_level(BUZZER_PIN, 1);
    }

    // I2C
    {
        ESP_ERROR_CHECK(i2cdev_init());
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // BMP280
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
        ESP_LOGI(TAG, "Found BMP280 device");
    }

    // MPU6050
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

    // SDCARD
    {
        if (sdcard_init(SD_MOSI_PIN, SD_MISO_PIN, SD_SCLK_PIN, SD_CS_PIN) != ESP_OK) {
            beep(5000);
            vTaskDelay(pdMS_TO_TICKS(1000));

            sdcard_umount();
            ESP_LOGE(TAG, "Failed to initialize SD card");
            return;
        }
    }

    // SETUP
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(8000));
    beep(500);

    // WAIT BUTTON
    while (gpio_get_level(PBUTTON_PIN)) vTaskDelay(pdMS_TO_TICKS(200));

    // SDCARD: open/create file
    if (sdcard_open_file(SD_FILE, "w", &file_ptr) != ESP_OK) {
        sdcard_umount();
        ESP_LOGE(TAG, "Failed to open/create file");
        return;
    }

    // ABORT CHECK
    if (bmp280_read_float(&bmp_dev, &flight_logic.sensor_data.temperature, &flight_logic.sensor_data.pressure) != ESP_OK || mpu6050_get_motion(&mpu_dev, &mpu_acc, &mpu_gyro) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensors");
        return;
    } 

    // INDICATE STARTUP
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5 sec
    beep(300);

    // INIT FLIGHT LOGIC
    flight_logic.sensor_data.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);
    flight_logic_init(&flight_logic);
    ut_sd_sync = flight_logic.sensor_data.ut;

    // main loop
    while (1) {
        flight_logic.sensor_data.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // read mpu6050
        ESP_ERROR_CHECK(mpu6050_get_motion(&mpu_dev, &mpu_acc, &mpu_gyro));
        flight_logic.sensor_data.accel = mpu_convert_accel(mpu_acc);
        flight_logic.sensor_data.rot = mpu_convert_gyro(mpu_gyro);

        // read bmp280
        ESP_ERROR_CHECK(bmp280_read_float(&bmp_dev, &flight_logic.sensor_data.temperature, &flight_logic.sensor_data.pressure));
        
        // write sdcard
		// snprintf(text, sizeof(text), "%d %ld %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.2f\n", flight_state, ut, accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, pressure, temperature);
		// sdcard_write(text, file_ptr);

        // sync sdcard
        if (flight_logic.sensor_data.ut - ut_sd_sync > SD_SYNC_INTERVAL) {
            sdcard_sync(file_ptr);
            ut_sd_sync = flight_logic.sensor_data.ut;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
