idf_component_register(SRCS "debug.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 bmp280 sdcard esp_timer)