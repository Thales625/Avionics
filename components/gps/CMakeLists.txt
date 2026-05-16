idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS .
    REQUIRES driver esp_idf_lib_helpers log i2cdev
)