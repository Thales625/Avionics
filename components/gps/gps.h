#ifndef __GPS_H__
#define __GPS_H__

#include "hal/uart_types.h"
#include <i2cdev.h>
#include <esp_err.h>

#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 1024

typedef struct {
    uart_port_t uart_num;
    uint8_t uart_buffer[128];
} gps_dev_t;

esp_err_t gps_init_desc(gps_dev_t *dev, gpio_num_t tx, gpio_num_t rx, uart_port_t uart_num);
esp_err_t gps_update(gps_dev_t *dev);
esp_err_t gps_read(gps_dev_t *dev, int32_t *lat, int32_t *lon, uint8_t *satellites);

#endif
