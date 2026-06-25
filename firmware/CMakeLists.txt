idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer esp_adc driver esp_driver_uart log i2cdev flight_logic tmtc crc flash_log mpu6050 bmp280 lora gps w25q64
)
