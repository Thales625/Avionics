idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer math_helper flight_logic telemetry driver log i2cdev mpu6050 bmp280
)
