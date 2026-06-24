#include "flash_log.h"

#include "esp_log.h"
#include "esp_err.h"

#include "w25q64.h"
#include <stdint.h>

static const char *TAG = "flash_log";

static bool writting = false;
static bool utc_time_set = false;

static flash_header_t header;
static uint32_t header_addr;

static flash_packet_t page_buffer[PACKETS_PER_PAGE];
static uint32_t buffer_offset;

static uint32_t current_flash_addr;

static uint32_t sizeof_header(flash_header_t *flash_header) {
    if (flash_header->format_version == 1) return 22;
    if (flash_header->format_version == 2) return 26;
    return sizeof(flash_header_t);
}

static bool packet_is_empty(flash_packet_t *pkt) {
    const uint8_t *p = (const uint8_t *)pkt;

    for (size_t i = 0; i < sizeof(flash_packet_t); i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

static esp_err_t flush_buffer(void) {
    if (buffer_offset == 0) return ESP_OK;

    uint32_t bytes_to_write = buffer_offset * sizeof(flash_packet_t);

    esp_err_t ret = w25q64_write_data(current_flash_addr, (uint8_t *)page_buffer, bytes_to_write);

    if (ret == ESP_OK) current_flash_addr += bytes_to_write;

    buffer_offset = 0;

    return ret;
}

static esp_err_t solve_corrupted_log(flash_header_t *corrupted_header){
    if (corrupted_header->status == 0x00) return ESP_OK;

    return ESP_FAIL;
}

static esp_err_t get_last_header(flash_header_t *header_out) {
    flash_header_t search_header;
    search_header.next_header_addr = 0;

    // initial values
    memset(header_out, 0xFF, sizeof_header(header_out));
    header_out->next_header_addr = 0;
    header_out->flight_number = 0;
    header_out->status = 0xFF;

    while (1) {
        if (w25q64_read_data(search_header.next_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t)) != ESP_OK) return ESP_FAIL;

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            if (search_header.status != 0x00) {
                // CORRUPTED flight log
                // TODO: find final of log
                return ESP_FAIL;
            }
        } else {
            break;
        }

        memcpy(header_out, &search_header, sizeof(flash_header_t)); // header = search_header
    }

    return ESP_OK;
}

void flash_log_list_flights(void) {
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    while (1) {
        w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t));

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            // its a valid header
            ESP_LOGI(TAG, "Flight number: %d", search_header.flight_number);
            ESP_LOGI(TAG, "Status: %d", search_header.status);
            ESP_LOGI(TAG, "UTC time: %d", search_header.utc_time);
            ESP_LOGI(TAG, "UTC date: %d", search_header.utc_date);
            ESP_LOGI(TAG, "Format version: %d", search_header.format_version);
            ESP_LOGI(TAG, "Next header address: %d", search_header.next_header_addr);
            ESP_LOGI(TAG, "------------------------");

            search_header_addr = search_header.next_header_addr;
        } else {
            break;
        }
    }
}

esp_err_t flash_log_read_flight(uint32_t flight_number) {
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    flash_packet_t packet;

    while (1) {
        w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t));

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            // its a valid header
            if (flight_number == search_header.flight_number) {
                // its the flight we're looking for
                ESP_LOGI(TAG, "---Reading packet---");
                ESP_LOGI(TAG, "Flight number: %d", search_header.flight_number);
                ESP_LOGI(TAG, "Status: %d", search_header.status);
                ESP_LOGI(TAG, "UTC time: %d", search_header.utc_time);
                ESP_LOGI(TAG, "UTC date: %d", search_header.utc_date);
                ESP_LOGI(TAG, "Format version: %d", search_header.format_version);
                ESP_LOGI(TAG, "Next header address: %d", search_header.next_header_addr);
                ESP_LOGI(TAG, "========================");

                if (search_header.status == 0xFF || search_header.next_header_addr == 0xFFFFFFFF) {
                    ESP_LOGI(TAG, "Flight log is corrupted");
                    return ESP_FAIL;
                }

                for (uint32_t addr = search_header_addr + sizeof(flash_header_t); addr < search_header.next_header_addr; addr+=sizeof(flash_packet_t)) {
                    w25q64_read_data(addr, (uint8_t *)&packet, sizeof(flash_packet_t));

                    if (packet.magic == FLASH_PACKET_MAGIC) {
                        ESP_LOGI(TAG, "Address:     0x%06" PRIX32, addr);

                        ESP_LOGI(TAG, "ut:          %" PRIu32, packet.payload.ut);
                        ESP_LOGI(TAG, "phase:       %" PRIu8, packet.payload.phase);

                        ESP_LOGI(TAG, "accel:       X=%.2f  Y=%.2f  Z=%.2f", packet.payload.accel.x, packet.payload.accel.y, packet.payload.accel.z);
                        ESP_LOGI(TAG, "ang_vel:     X=%.2f  Y=%.2f  Z=%.2f", packet.payload.ang_vel.x, packet.payload.ang_vel.y, packet.payload.ang_vel.z);

                        ESP_LOGI(TAG, "pressure:    %.2f", packet.payload.pressure);
                        ESP_LOGI(TAG, "temperature: %.2f", packet.payload.temperature);

                        ESP_LOGI(TAG, "lat:         %" PRId32, packet.payload.lat_nmea);
                        ESP_LOGI(TAG, "lon:         %" PRId32, packet.payload.lon_nmea);
                        ESP_LOGI(TAG, "satellites:  %" PRId8, packet.payload.satellites);

                        ESP_LOGI(TAG, "battery:    %.2f V", packet.payload.v_bat);

                        ESP_LOGI(TAG, "------------------");
                    }
                }
                return ESP_OK;
            }

            search_header_addr = search_header.next_header_addr;
        } else {
            break;
        }
    }
    return ESP_FAIL;
}

void flash_log_read_telemetry(void) {
    flash_packet_t packet;
    uint32_t address = 0;

    while (1) {
        esp_err_t err = w25q64_read_data(address, (uint8_t *)&packet, sizeof(flash_packet_t));

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read packet at %d", address);
            break;
        }

        ESP_LOGI(TAG, "Packet %d: 0x%02X", address / sizeof(flash_packet_t), packet);

        if (packet_is_empty(&packet)) {
            ESP_LOGI(TAG, "Reached empty flash area at packet %d", address);
            break;
        }


        address += sizeof(flash_packet_t);
    }

    ESP_LOGI(TAG, "Done reading flash memory");
}

esp_err_t flash_log_start_flight(void) {
    if (writting) return ESP_OK;

    writting = true;
    buffer_offset = 0;
    memset(page_buffer, 0xFF, sizeof(page_buffer));

    // get last header
    flash_header_t last_header;
    get_last_header(&last_header);

    // write new header
    memset(&header, 0xFF, sizeof(header));
    header.magic = FLASH_HEADER_MAGIC;
    header.format_version = FLASH_FORMAT_VERSION;
    header.next_header_addr = 0xFFFFFFFF;
    header.status = 0xFF;
    header.flight_number = last_header.flight_number+1;
    header.duration = 0xFFFFFFFF;
    header.utc_time = 0xFFFFFFFF;
    header.utc_date = 0xFFFFFFFF;

    header_addr = last_header.next_header_addr;

    w25q64_write_data(header_addr, (uint8_t *)&header, sizeof(header));

    current_flash_addr = header_addr + sizeof(header);

    return ESP_OK;
}

esp_err_t flash_log_append(const flash_payload_t *payload) {
    page_buffer[buffer_offset].magic = FLASH_PACKET_MAGIC;
    page_buffer[buffer_offset].payload = *payload;

    buffer_offset++;

    if (buffer_offset < PACKETS_PER_PAGE) return ESP_OK;

    return flush_buffer();
}

esp_err_t flash_log_set_utc(uint32_t utc_time, uint32_t utc_date) {
    if (writting && !utc_time_set) {
        header.utc_time = utc_time;
        header.utc_date = utc_date;
        utc_time_set = true;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t flash_log_finish_flight(uint32_t duration) {
    if (!writting) return ESP_OK;

    writting = false;
    utc_time_set = false;

    if (flush_buffer() != ESP_OK) return ESP_OK;

    header.next_header_addr = current_flash_addr;
    header.status = 0x00;
    header.duration = duration;

    return w25q64_write_data(header_addr, (uint8_t *)&header, sizeof(header));
}

void flash_log_clear_flights(void) {
    // clear all flights
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    while (1) {
        w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t));

        if (search_header.magic != FLASH_HEADER_MAGIC) {
            break;
        }

        search_header_addr = search_header.next_header_addr;
    }

    // clear until search_header_addr
    uint32_t last_sector_addr = (search_header_addr + W25Q64_SECTOR_SIZE - 1) & ~(W25Q64_SECTOR_SIZE - 1);
    for (uint32_t clear_addr=0; clear_addr<last_sector_addr; clear_addr+=W25Q64_SECTOR_SIZE) {
        w25q64_erase_sector(clear_addr);
    }
}

void flash_log_clear(uint32_t last_sector_idx) {
    if (last_sector_idx == 0) {
        w25q64_erase_chip();
        return;
    }

    for (uint32_t sector_idx=0; sector_idx<last_sector_idx; sector_idx+=1) {
        w25q64_erase_sector(sector_idx*W25Q64_SECTOR_SIZE);
    }
}