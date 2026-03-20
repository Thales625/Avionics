#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

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

static uint32_t ut_sd_sync;
static flight_logic_t flight_logic;
static char text[128];

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

static inline vector3f_t mpu_convert_accel(mpu6050_acceleration_t accel) {
    return (vector3f_t) {
        .x = accel.x,
        .y = accel.y,
        .z = accel.z
    };
}

static inline vector3f_t mpu_convert_gyro(mpu6050_rotation_t rot) {
    return (vector3f_t) {
        .x = rot.x,
        .y = rot.y,
        .z = rot.z
    };
}

static void avionics_task(void *arg) {
    // sensor aux variables
    mpu6050_acceleration_t mpu_acc = { 0 };
    mpu6050_rotation_t mpu_gyro = { 0 };

    float bmp_temperature = 0;
    float bmp_pressure = 0;
    
    // log file
    FILE *file_ptr;

    // startup delay + buzzer indication
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(8000));
    beep(500);

    // SDCARD: open or create log file
    if (sdcard_open_file(SD_FILE, "w", &file_ptr) != ESP_OK) {
        sdcard_umount();
        ESP_LOGE(TAG, "Failed to open/create file");
        avionics_abort(2);
    }

    // startup indication
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5 sec
    beep(300);

    // init flight logic core
    flight_logic.sensor_data.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);
    flight_logic_init(&flight_logic);
    ut_sd_sync = flight_logic.sensor_data.ut;

    // main loop
    while (1) {
        flight_logic.sensor_data.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // MPU6050: read data
        if (mpu6050_get_motion(&mpu_dev, &mpu_acc, &mpu_gyro) != ESP_OK) {
            ESP_LOGW(TAG, "MPU6050: read failed");

            if (flight_logic.state == STATE_PRE_FLIGHT) {
                avionics_abort(3);
            }

            flight_logic.sensor_data.accel.x = NAN;
            flight_logic.sensor_data.accel.y = NAN;
            flight_logic.sensor_data.accel.z = NAN;

            flight_logic.sensor_data.rot.x = NAN;
            flight_logic.sensor_data.rot.y = NAN;
            flight_logic.sensor_data.rot.z = NAN;
        } else {
            flight_logic.sensor_data.accel = mpu_convert_accel(mpu_acc);
            flight_logic.sensor_data.rot = mpu_convert_gyro(mpu_gyro);
        }

        // BMP280: read data
        if (bmp280_read_float(&bmp_dev, &bmp_temperature, &bmp_pressure) != ESP_OK) {
            ESP_LOGW(TAG, "BMP280: read failed");

            if (flight_logic.state == STATE_PRE_FLIGHT) {
                avionics_abort(3);
            }
        } else {
            flight_logic.sensor_data.temperature = bmp_temperature;
            flight_logic.sensor_data.pressure = bmp_pressure;
        }
        
        // update flight logic
        flight_logic_update(&flight_logic);

        // SDCARD: write
        snprintf(text, sizeof(text),
            "%d %" PRIu32 " %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.2f\n",
            flight_logic.state,
            flight_logic.sensor_data.ut,
            flight_logic.sensor_data.accel.x,
            flight_logic.sensor_data.accel.y,
            flight_logic.sensor_data.accel.z,
            flight_logic.sensor_data.rot.x,
            flight_logic.sensor_data.rot.y,
            flight_logic.sensor_data.rot.z,
            flight_logic.sensor_data.pressure,
            flight_logic.sensor_data.temperature
        );
		sdcard_write(text, file_ptr);

        // SDCARD: sync
        if (flight_logic.sensor_data.ut - ut_sd_sync > SD_SYNC_INTERVAL) {
            sdcard_sync(file_ptr);
            ut_sd_sync = flight_logic.sensor_data.ut;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void app_main(void) {
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

    // SDCARD initialization
    {
        if (sdcard_init(SD_MOSI_PIN, SD_MISO_PIN, SD_SCLK_PIN, SD_CS_PIN) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SD card");

            beep(5000);
            avionics_abort(1);
        }
    }

    // set default values
    gpio_set_level(PARACHUTE_PIN, 0);
    gpio_set_level(LED_PIN, 1);
    gpio_set_level(BUZZER_PIN, 1);

    // create avionics task
    xTaskCreate(avionics_task, "avionics_task", 4096, NULL, 10, NULL);
}
