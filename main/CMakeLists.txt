idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer driver log i2cdev mpu6050 bmp280 fatfs sdcard)