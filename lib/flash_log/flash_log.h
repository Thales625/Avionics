#ifndef __FLASH_LOG_H__
#define __FLASH_LOG_H__

#include <unistd.h>
#include "esp_err.h"

#include "math_helper.h"

#define FLASH_HEADER_MAGIC 0x46484452 // "FHDR"
#define FLASH_PACKET_MAGIC 0x46504143 // "FPAC"

#define FLASH_FORMAT_VERSION 4

#define FLASH_PAGE_SIZE 256
#define PACKETS_PER_PAGE (FLASH_PAGE_SIZE / sizeof(flash_packet_t))
#define BYTES_PER_PAGE (PACKETS_PER_PAGE * sizeof(flash_packet_t))

typedef struct __attribute__((packed)) {
    // identify
    uint32_t magic; // FLASH_HEADER_MAGIC
    uint8_t header_size;
    uint8_t packet_size;
    uint8_t format_version;

    // validation
    uint8_t status; // 0xFF = in progress | 0x00 = completed
    uint32_t next_header_addr;

    // flight data
    uint32_t flight_number;
    uint32_t duration; // ms
    uint32_t timestamp;
    int32_t lat_nmea, lon_nmea;
} flash_header_t;

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


flash_header_t* flash_log_get_headers(uint32_t* len);

esp_err_t flash_log_get_header(uint32_t flight_number, flash_header_t* flash_header, uint32_t* flash_header_addr);

void flash_log_list_flights(void);

esp_err_t flash_log_read_flight(uint32_t flight_number);

esp_err_t flash_log_get_flight(uint32_t flight_number);

esp_err_t flash_log_get_flight_packet(uint32_t flash_packet_addr, uint32_t flash_packet_size, flash_packet_t* flash_packet);



esp_err_t flash_log_init(void);

esp_err_t flash_log_start_flight(void);

esp_err_t flash_log_append(flash_payload_t *payload);

esp_err_t flash_log_set_gps_data(uint32_t utc_time, uint32_t utc_date, int32_t lat_nmea, int32_t lon_nmea);

esp_err_t flash_log_finish_flight(uint32_t duration);



esp_err_t flash_log_clear_flights(void);

esp_err_t flash_log_clear(uint32_t last_sector_idx);

#endif
