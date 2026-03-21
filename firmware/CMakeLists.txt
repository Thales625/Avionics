idf_component_register(
    SRCS "main.c" "flight_logic.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer math_helper driver log i2cdev mpu6050 bmp280 fatfs sdcard
)
