#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <math.h>

#include "sdcard.h"
#include "mpu6050.h"
#include "bmp280.h"

#define SDA_GPIO_PIN 21
#define SCL_GPIO_PIN 22

#define PARACHUTE_PIN 18
#define BUZZER_PIN 23
#define LED_PIN 15

#define SD_CS_PIN 26
#define SD_MISO_PIN 32
#define SD_MOSI_PIN 25
#define SD_SCLK_PIN 33

#define SD_FILE "datalog.txt"

// #define LIST_FILES

static const char *TAG = "avionics";

static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };
static FILE *file_ptr;

// state
static float max_altitude = 0;
static bool parachute_deployed = false;


inline static void beep(int duration) {
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(BUZZER_PIN, 0);
}

inline static void deploy_parachute() {
    ESP_LOGW(TAG, ">>> Parachute ejection <<<");
    gpio_set_level(PARACHUTE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(PARACHUTE_PIN, 0);
    parachute_deployed = true;
}

inline static void check_apogee_and_deploy(float current_altitude) {
    static int descent_count = 0;
    
    if (current_altitude > max_altitude) {
        max_altitude = current_altitude;
        descent_count = 0;
    } else {
        if ((max_altitude - current_altitude) > 5.0) {
            descent_count++;
            if (descent_count >= 2 && !parachute_deployed) {
                deploy_parachute();
            }
        }
    }
}

void main_task(void *pvParameters) {
    float ut;
    const float ut0 = esp_timer_get_time() / 1000;

    char text[128];

    // state
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };
    float pressure, temperature, altitude;

    // main loop
    while (1) {
        ut = (float) esp_timer_get_time() / 1000;

        // check time
        if (ut - ut0 > 5000) break;

        // read mpu6050
        ESP_ERROR_CHECK(mpu6050_get_motion(&mpu_dev, &accel, &rotation));

        // read bmp280
        ESP_ERROR_CHECK(bmp280_read_float(&bmp_dev, &temperature, &pressure));

        // filtering
        altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));
        
        // parachute
        // check_apogee_and_deploy(altitude);

        // write sdcard
		snprintf(text, sizeof(text), "%.2f %.2f", ut, temperature);
		sdcard_write(text, file_ptr);

        // LOG
        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "UT = %.2f", ut);

        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        
        ESP_LOGI(TAG, "Temperature = %.2f", temperature);
        ESP_LOGI(TAG, "Pressure = %.2f", pressure);
        ESP_LOGI(TAG, "Altitude = %.2f", altitude);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // umount sdcard
	sdcard_close_file(file_ptr);
    sdcard_umount();

    beep(1000);

    vTaskDelete(NULL);
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    // config GPIO
    ESP_ERROR_CHECK(i2cdev_init());

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PARACHUTE_PIN) | 
                        (1ULL << LED_PIN) | 
                        (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(PARACHUTE_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);

    // TODO: CHECK BMP280 or MPU6050 ERRORS
    // BMP280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    ESP_ERROR_CHECK(bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO_PIN, SCL_GPIO_PIN));
    ESP_ERROR_CHECK(bmp280_init(&bmp_dev, &params));

    // MPU6050
    ESP_ERROR_CHECK(mpu6050_init_desc(&mpu_dev, MPU6050_I2C_ADDRESS_LOW, 0, SDA_GPIO_PIN, SCL_GPIO_PIN));
    while (1) {
        esp_err_t res = i2c_dev_probe(&mpu_dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Found MPU6050 device");
            break;
        }
        ESP_LOGE(TAG, "MPU6050 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_250));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_4));

    // SDCARD
    if (sdcard_init(SD_MOSI_PIN, SD_MISO_PIN, SD_SCLK_PIN, SD_CS_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SD card");
        return;
    }
    
    #ifdef LIST_FILES
    sdcard_list_files();
    sdcard_umount();
    return;
    #endif

    // open/create file
    if (sdcard_open_file(SD_FILE, "w", &file_ptr) != ESP_OK) {
        sdcard_umount();
        ESP_LOGE(TAG, "Failed to open/create file");
        return;
    }

    // OUTPUT
    gpio_set_level(LED_PIN, 1);
    beep(500);
    gpio_set_level(LED_PIN, 0);
    
    // task
    xTaskCreate(main_task, "main_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}