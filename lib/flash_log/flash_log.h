#ifndef __FLASH_LOG_H__
#define __FLASH_LOG_H__

#include <inttypes.h> // IWYU pragma: keep
#include <unistd.h>
#include "esp_err.h"

#include "math_helper.h"

#define FLASH_HEADER_MAGIC 0x46484452 // "FHDR"
#define FLASH_PACKET_MAGIC 0x46504143 // "FPAC"

#define FLASH_FORMAT_VERSION 2

#define FLASH_PAGE_SIZE 256
#define PACKETS_PER_PAGE (FLASH_PAGE_SIZE / sizeof(flash_packet_t))
#define BYTES_PER_PAGE (PACKETS_PER_PAGE * sizeof(flash_packet_t))

typedef struct __attribute__((packed)) {
    uint32_t ut;

    vector3f_t accel;
    vector3f_t ang_vel;
    float pressure;
    float temperature;

    int32_t lat_nmea, lon_nmea;
    uint8_t satellites;

    float v_bat;

    uint8_t phase;
} flash_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t magic; // FLASH_PACKET_MAGIC

    flash_payload_t payload;
} flash_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t magic; // FLASH_HEADER_MAGIC

    uint8_t format_version;

    uint32_t next_header_addr;
    uint8_t status; // 0xFF = in progress | 0x00 = completed

    uint32_t flight_number;

    uint32_t duration; // ms

    uint32_t utc_time, utc_date;
} flash_header_t;

void flash_log_list_flights(void);

esp_err_t flash_log_read_flight(uint32_t flight_number);

void flash_log_read_telemetry(void);

esp_err_t flash_log_start_flight(void);

esp_err_t flash_log_append(const flash_payload_t *payload);

esp_err_t flash_log_set_utc(uint32_t utc_time, uint32_t utc_date);

esp_err_t flash_log_finish_flight(uint32_t duration);

void flash_log_clear_flights(void);

void flash_log_clear(uint32_t last_sector_idx);

#endif
