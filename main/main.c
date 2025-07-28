#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <math.h>

#include <mpu6050.h>
#include <bmp280.h>

static const char *TAG = "avionics";

#define SDA_GPIO_PIN 21
#define SCL_GPIO_PIN 22

// #define DEBUG

void main_task(void *pvParameters) {
    // BMP280
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t bmp_dev = { 0 };

    ESP_ERROR_CHECK(bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO_PIN, SCL_GPIO_PIN));
    ESP_ERROR_CHECK(bmp280_init(&bmp_dev, &params));

    // MPU6050
    mpu6050_dev_t mpu_dev = { 0 };

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

    ESP_LOGI(TAG, "Gyro range:  %d", mpu_dev.ranges.gyro);
    ESP_LOGI(TAG, "Accel range: %d", mpu_dev.ranges.accel);

    // main loop
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };

    float pressure, temperature;

    float altitude;

    while (1) {
        // read mpu6050
        ESP_ERROR_CHECK(mpu6050_get_motion(&mpu_dev, &accel, &rotation));

        // read bmp280
        ESP_ERROR_CHECK(bmp280_read_float(&bmp_dev, &temperature, &pressure));
        
        #ifdef DEBUG
        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        
        ESP_LOGI(TAG, "Temperature = %.2f", temperature);
        ESP_LOGI(TAG, "Pressure = %.2f", pressure);
        #endif

        // filtering
        altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));

        #ifndef DEBUG
        // acc.x acc.y acc.z rot.x rot.y rot.z altitude
        printf("%f %f %f %f %f %f %f\n", accel.x, accel.y, accel.z, rotation.x, rotation.y, rotation.z, altitude);
        #endif

        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main() {
    ESP_ERROR_CHECK(i2cdev_init());
    
    // task
    xTaskCreate(main_task, "main_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}