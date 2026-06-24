idf_component_register(
    SRCS "flash_log.c"
    INCLUDE_DIRS "."
    REQUIRES crc w25q64 math_helper
)
