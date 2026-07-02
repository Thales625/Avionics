#include "flash_log.h"

#include "esp_log.h"
#include "esp_err.h"

#include <stdint.h>
#include <inttypes.h>

#include "w25q64.h"

static const char *TAG = "flash_log";

static bool initialized = false;
static bool writting = false;
static bool has_gps_data = false;

static flash_packet_t page_buffer[PACKETS_PER_PAGE];
static uint32_t buffer_offset;

static flash_header_t last_header;

static flash_header_t current_header;
static uint32_t current_header_addr;

static uint32_t current_packet_addr;


static bool packet_is_empty(uint8_t *pkt, uint32_t len) {
    const uint8_t *p = (const uint8_t *)pkt;

    for (size_t i = 0; i < len; i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

static esp_err_t flush_buffer(void) {
    if (buffer_offset == 0) return ESP_OK;

    uint32_t bytes_to_write = buffer_offset * sizeof(flash_packet_t);

    esp_err_t ret = w25q64_write_data(current_packet_addr, (uint8_t *)page_buffer, bytes_to_write);

    if (ret == ESP_OK) current_packet_addr += bytes_to_write;

    buffer_offset = 0;

    return ret;
}

static esp_err_t solve_corrupted_log(flash_header_t *corrupted_header, uint32_t corrupted_header_addr) {
    if (corrupted_header->status == 0x00 || corrupted_header->next_header_addr != 0xFFFFFFFF) return ESP_OK;

    if (corrupted_header->header_size != sizeof(flash_header_t)) {
        ESP_LOGW(TAG, "treating a log with different header size than current: %d vs %d", corrupted_header->header_size, sizeof(flash_header_t));
    }

    uint8_t *packet = (uint8_t *)malloc(corrupted_header->packet_size);
    uint32_t packet_addr = corrupted_header_addr + corrupted_header->header_size;

    while (1) {
        if (w25q64_read_data(packet_addr, packet, corrupted_header->packet_size) != ESP_OK) {
            free(packet);
            return ESP_FAIL;
        }

        if (packet_is_empty(packet, corrupted_header->packet_size)) {
            free(packet);
            corrupted_header->next_header_addr = packet_addr;
            if (w25q64_write_data(corrupted_header_addr + offsetof(flash_header_t, next_header_addr), (uint8_t *)&packet_addr, sizeof(packet_addr)) != ESP_OK) return ESP_FAIL;
            return ESP_OK;
        }

        packet_addr += corrupted_header->packet_size;
    }
}

static esp_err_t get_last_header(flash_header_t *last_header) {
    flash_header_t search_header;
    search_header.next_header_addr = 0;

    // initial values
    memset(last_header, 0xFF, sizeof(flash_header_t));
    last_header->next_header_addr = 0;
    last_header->flight_number = 0;

    while (1) {
        uint32_t prev_header_addr = search_header.next_header_addr;

        if (w25q64_read_data(search_header.next_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t)) != ESP_OK) return ESP_FAIL;

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            if (search_header.status != 0x00 && search_header.next_header_addr == 0xFFFFFFFF) {
                // corrupted flight log
                ESP_LOGI(TAG, "Corrupted log found, solving...");

                if (solve_corrupted_log(&search_header, prev_header_addr) != ESP_OK) return ESP_FAIL;

                *last_header = search_header;

                return ESP_OK;
            } else {
                // not corrupted flight log
            }
        } else {
            return ESP_OK;
        }

        memcpy(last_header, &search_header, sizeof(flash_header_t)); // last_header = search_header
    }

    return ESP_OK;
}


flash_header_t* flash_log_get_headers(uint32_t* len) {
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    uint32_t headers_idx = 0;
    uint32_t headers_allocated = 32;

    flash_header_t* headers = malloc(headers_allocated * sizeof(flash_header_t));

    while (1) {
        w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t));

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            // check if we need to allocate more space
            if (headers_idx >= headers_allocated) {
                headers_allocated += 32;
                headers = realloc(headers, headers_allocated * sizeof(flash_header_t));
            }

            // its a valid header
            uint32_t header_size = search_header.header_size;
            if (header_size > sizeof(flash_header_t)) {
                // different version header
                header_size = sizeof(flash_header_t);
            }
            else if (header_size == 0) {
                // invalid header size
                headers_idx = 0;
                break;
            }

            memcpy(&headers[headers_idx], &search_header, header_size);
            headers_idx++;

            search_header_addr = search_header.next_header_addr;
        } else {
            break;
        }
    }

    // resize headers
    if (headers_idx == 0 && headers_allocated > headers_idx) {
        headers = realloc(headers, headers_idx * sizeof(flash_header_t));
    }

    *len = headers_idx;
    return headers;
}

esp_err_t flash_log_get_header(uint32_t flight_number, flash_header_t* flash_header, uint32_t* flash_header_addr) {
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    while (1) {
        w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t));

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            // its a valid header
            if (flight_number == search_header.flight_number) {
                *flash_header = search_header;

                if (flash_header_addr != NULL) {
                    *flash_header_addr = search_header_addr;
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

void flash_log_list_flights(void) {
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    while (1) {
        w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t));

        if (search_header.magic == FLASH_HEADER_MAGIC) {
            // its a valid header
            ESP_LOGI(TAG, "Flight number: %d", search_header.flight_number);
            ESP_LOGI(TAG, "Status: %d", search_header.status);
            ESP_LOGI(TAG, "timestamp: %d", search_header.timestamp);
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
                ESP_LOGI(TAG, "timestamp: %d", search_header.timestamp);
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

                        ESP_LOGI(TAG, "battery:     %.2f V", packet.payload.v_bat);

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

esp_err_t flash_log_get_flight_packet(uint32_t flash_packet_addr, uint32_t flash_packet_size, flash_packet_t* flash_packet) {
    return w25q64_read_data(flash_packet_addr, (uint8_t *)flash_packet, flash_packet_size);
}



esp_err_t flash_log_init(void) {
    // update last header
    if (get_last_header(&last_header) != ESP_OK) return ESP_FAIL;
    initialized = true;
    return ESP_OK;
}

esp_err_t flash_log_start_flight(void) {
    if (!initialized || writting) return ESP_FAIL;

    // clear buffer
    memset(page_buffer, 0xFF, sizeof(page_buffer));
    buffer_offset = 0;

    // set current header values
    memset(&current_header, 0xFF, sizeof(current_header));
    current_header.magic = FLASH_HEADER_MAGIC;
    current_header.header_size = sizeof(flash_header_t);
    current_header.packet_size = sizeof(flash_packet_t);
    current_header.format_version = FLASH_FORMAT_VERSION;
    current_header.flight_number = last_header.flight_number+1;

    current_header_addr = last_header.next_header_addr;
    current_packet_addr = current_header_addr + sizeof(flash_header_t);

    writting = true;

    // write header
    return w25q64_write_data(current_header_addr, (uint8_t *)&current_header, sizeof(flash_header_t));
}

esp_err_t flash_log_append(flash_payload_t *payload) {
    if (!initialized || !writting) return ESP_FAIL;

    page_buffer[buffer_offset].magic = FLASH_PACKET_MAGIC;
    page_buffer[buffer_offset].payload = *payload;

    buffer_offset++;

    if (buffer_offset < PACKETS_PER_PAGE) return ESP_OK;

    return flush_buffer();
}

esp_err_t flash_log_set_gps_data(uint32_t utc_time, uint32_t utc_date, int32_t lat_nmea, int32_t lon_nmea) {
    if (writting && !has_gps_data) {
        // convert UTC to timestamp
        uint32_t day   = utc_date / 10000;
        uint32_t month = (utc_date / 100) % 100;
        uint32_t year  = 2000 + (utc_date % 100);

        uint32_t hour = utc_time / 10000;
        uint32_t min  = (utc_time / 100) % 100;
        uint32_t sec  = utc_time % 100;

        current_header.timestamp = ((year - 2026) << 26) |
            (month << 22) |
            (day   << 17) |
            (hour  << 12) |
            (min   << 6 ) |
            sec;

        // set lat and lon
        current_header.lat_nmea = lat_nmea;
        current_header.lon_nmea = lon_nmea;

        has_gps_data = true;

        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t flash_log_finish_flight(uint32_t duration) {
    if (!initialized || !writting) return ESP_FAIL;

    writting = false;

    // write remaining data
    flush_buffer();

    // write header information
    current_header.status = 0x00;
    current_header.next_header_addr = current_packet_addr;
    current_header.duration = duration;
    if (w25q64_write_data(current_header_addr, (uint8_t *)&current_header, sizeof(current_header)) != ESP_OK) return ESP_FAIL;

    // update last header
    last_header = current_header;

    return ESP_OK;
}



esp_err_t flash_log_clear_flights(void) {
    // clear all flights
    flash_header_t search_header;
    uint32_t search_header_addr = 0;

    while (1) {
        if (w25q64_read_data(search_header_addr, (uint8_t *)&search_header, sizeof(flash_header_t)) != ESP_OK) return ESP_FAIL;

        if (search_header.magic != FLASH_HEADER_MAGIC) {
            break;
        }

        search_header_addr = search_header.next_header_addr;
    }

    // clear until search_header_addr
    uint32_t last_sector_addr = (search_header_addr + W25Q64_SECTOR_SIZE - 1) & ~(W25Q64_SECTOR_SIZE - 1);
    for (uint32_t clear_addr=0; clear_addr<last_sector_addr; clear_addr+=W25Q64_SECTOR_SIZE) {
        if (w25q64_erase_sector(clear_addr) != ESP_OK) return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t flash_log_clear(uint32_t last_sector_idx) {
    if (last_sector_idx == 0) {
        return w25q64_erase_chip();
    }

    for (uint32_t sector_idx=0; sector_idx<last_sector_idx; sector_idx+=1) {
        if (w25q64_erase_sector(sector_idx*W25Q64_SECTOR_SIZE) != ESP_OK) return ESP_FAIL;
    }

    return ESP_OK;
}