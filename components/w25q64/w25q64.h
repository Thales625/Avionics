#ifndef __W25Q64_H__
#define __W25Q64_H__

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#define W25Q64_SECTOR_SIZE 4096
#define W25Q64_PAGE_SIZE 256

/**
 * @brief Initializes SPI communication with the W25Q64 chip
 */
esp_err_t w25q64_init(gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin);

/**
 * @brief read W25Q64 status
 */
uint8_t w25q64_read_status(void);

/**
 * @brief Reads bytes from any address in the flash memory
 */
esp_err_t w25q64_read_data(uint32_t address, uint8_t *data, size_t size);

/**
 * @brief Writes bytes to the flash memory.
 *        Note: The sector (4KB) containing this address MUST be erased before writing!
 */
esp_err_t w25q64_write_data(uint32_t address, const uint8_t *data, size_t size);

/**
 * @brief Erases a 4KB sector. The address must be aligned to 4096 bytes (0x1000).
 */
esp_err_t w25q64_erase_sector(uint32_t sector_address);

#endif
