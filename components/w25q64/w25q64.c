#include "w25q64.h"

#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"

static const char *TAG = "w25q64";

#define CMD_WRITE_ENABLE    0x06
#define CMD_READ_STATUS_1   0x05
#define CMD_READ_DATA       0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_SECTOR_ERASE    0x20
#define CMD_JEDEC_ID        0x9F
#define CMD_CHIP_ERASE      0xC7 // Bulk Erase

static spi_device_handle_t w25q64_spi;
static gpio_num_t w25q64_cs_pin;

static inline void w25q64_cs_low(void) {
    gpio_set_level(w25q64_cs_pin, 0);
}

static inline void w25q64_cs_high(void) {
    gpio_set_level(w25q64_cs_pin, 1);
}

static inline void w25q64_wait_ready(void) {
    while ((w25q64_read_status() & 0x01) == 0x01) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t w25q64_read_JEDEC_ID(void) {
    w25q64_cs_low();

    uint8_t cmd = CMD_JEDEC_ID;
    uint8_t id[3] = {0};

    spi_transaction_t t_cmd = { .length = 8, .tx_buffer = &cmd };
    spi_device_polling_transmit(w25q64_spi, &t_cmd);
    spi_transaction_t t_id = { .length = 24, .rx_buffer = id };
    spi_device_polling_transmit(w25q64_spi, &t_id);
    w25q64_cs_high();

    ESP_LOGI(TAG, "Device JEDEC ID: Mfg=0x%02X, Type=0x%02X, Cap=0x%02X", id[0], id[1], id[2]);

    if (id[0] == 0xFF || id[0] == 0x00) {
        ESP_LOGE(TAG, "Invalid ID read. Check connections!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

uint8_t w25q64_read_status(void) {
    w25q64_cs_low();

    uint8_t cmd = CMD_READ_STATUS_1;
    uint8_t status = 0;

    spi_transaction_t t_cmd = { .length = 8, .tx_buffer = &cmd };
    spi_device_polling_transmit(w25q64_spi, &t_cmd);

    spi_transaction_t t_data = { .length = 8, .rx_buffer = &status };
    spi_device_polling_transmit(w25q64_spi, &t_data);

    w25q64_cs_high();
    return status;
}

static void w25q64_write_enable(void) {
    w25q64_cs_low();
    uint8_t cmd = CMD_WRITE_ENABLE;
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    spi_device_polling_transmit(w25q64_spi, &t);
    w25q64_cs_high();
}


esp_err_t w25q64_init(gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin){
    esp_err_t ret;

    w25q64_cs_pin = cs_pin;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << w25q64_cs_pin),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    w25q64_cs_high();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to init SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10e6, // 10 MHz
        .mode = 0,              // SPI mode 0
        .spics_io_num = -1,     // manual CS
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &w25q64_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // read JEDEC_ID to validate
    return w25q64_read_JEDEC_ID();
}

esp_err_t w25q64_read_data(uint32_t address, uint8_t *data, size_t size) {
    if (size == 0 || data == NULL) return ESP_ERR_INVALID_ARG;

    w25q64_wait_ready();
    w25q64_cs_low();

    uint8_t cmd_addr[4] = {
        CMD_READ_DATA,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        (address) & 0xFF
    };

    spi_transaction_t t_cmd = { .length = 32, .tx_buffer = cmd_addr };
    spi_device_polling_transmit(w25q64_spi, &t_cmd);

    spi_transaction_t t_data = { .length = size * 8, .rx_buffer = data };
    spi_device_polling_transmit(w25q64_spi, &t_data);

    w25q64_cs_high();
    return ESP_OK;
}

static esp_err_t w25q64_write_page(uint32_t address, const uint8_t *data, size_t size) {
    w25q64_write_enable();
    w25q64_cs_low();

    uint8_t cmd_addr[4] = {
        CMD_PAGE_PROGRAM,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        (address) & 0xFF
    };

    spi_transaction_t t_cmd = { .length = 32, .tx_buffer = cmd_addr };
    spi_device_polling_transmit(w25q64_spi, &t_cmd);

    spi_transaction_t t_data = { .length = size * 8, .tx_buffer = data };
    spi_device_polling_transmit(w25q64_spi, &t_data);

    w25q64_cs_high();
    w25q64_wait_ready();

    return ESP_OK;
}

esp_err_t w25q64_write_data(uint32_t address, const uint8_t *data, size_t size) {
    if (size == 0 || data == NULL) return ESP_ERR_INVALID_ARG;

    uint32_t current_addr = address;
    size_t bytes_remaining = size;
    const uint8_t *data_ptr = data;

    while (bytes_remaining > 0) {
        size_t page_offset = current_addr % W25Q64_PAGE_SIZE;
        size_t bytes_to_write = W25Q64_PAGE_SIZE - page_offset;

        if (bytes_to_write > bytes_remaining) {
            bytes_to_write = bytes_remaining;
        }

        esp_err_t ret = w25q64_write_page(current_addr, data_ptr, bytes_to_write);
        if (ret != ESP_OK) return ret;

        current_addr += bytes_to_write;
        data_ptr += bytes_to_write;
        bytes_remaining -= bytes_to_write;
    }

    return ESP_OK;
}

esp_err_t w25q64_erase_sector(uint32_t sector_address) {
    if (sector_address % W25Q64_SECTOR_SIZE != 0) {
        ESP_LOGE(TAG, "Sector address must be a multiple of 4096 bytes");
        return ESP_ERR_INVALID_ARG;
    }

    w25q64_write_enable();
    w25q64_cs_low();

    uint8_t cmd_addr[4] = {
        CMD_SECTOR_ERASE,
        (sector_address >> 16) & 0xFF,
        (sector_address >> 8) & 0xFF,
        (sector_address) & 0xFF
    };

    spi_transaction_t t_cmd = { .length = 32, .tx_buffer = cmd_addr };
    spi_device_polling_transmit(w25q64_spi, &t_cmd);

    w25q64_cs_high();
    w25q64_wait_ready();

    ESP_LOGI(TAG, "Erased sector at 0x%06lX", sector_address);
    return ESP_OK;
}

esp_err_t w25q64_erase_chip(void) {
    ESP_LOGI(TAG, "Starting chip erase...");

    w25q64_write_enable();
    w25q64_cs_low();

    uint8_t cmd = CMD_CHIP_ERASE;
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    esp_err_t ret = spi_device_polling_transmit(w25q64_spi, &t);

    w25q64_cs_high();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Bulk Erase command");
        return ret;
    }

    w25q64_wait_ready();

    ESP_LOGI(TAG, "Memory erased successfully");

    return ESP_OK;
}