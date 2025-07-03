#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>

#include <mpu6050.h>
#include <bmp280.h>

static const char *TAG = "avionics_test";

#define SDA_GPIO_PIN 21
#define SCL_GPIO_PIN 22

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
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&mpu_dev));

    ESP_LOGI(TAG, "Accel range: %d", mpu_dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", mpu_dev.ranges.gyro);

    // main loop
    float pressure, temperature, humidity;
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t rotation = { 0 };

    while (1) {
        ESP_ERROR_CHECK(mpu6050_get_motion(&mpu_dev, &accel, &rotation));

        ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);

        if (bmp280_read_float(&bmp_dev, &temperature, &pressure, &humidity) != ESP_OK) {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        ESP_LOGI(TAG, "Temperature = %.2f", temperature);
        ESP_LOGI(TAG, "Pressure = %.2f", pressure);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    // task
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(main_task, "main_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}