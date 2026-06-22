#ifndef __TMTC_H__
#define __TMTC_H__

#include <inttypes.h> // IWYU pragma: keep
#include <unistd.h>

#include "math_helper.h"

#define TELEMETRY_MAGIC 0xAABBCCDD
#define TELECOMMAND_MAGIC 0xDDCCBBAA

#define TMTC_AIR_DATA_RATE LORA_AIR_DATA_RATE_2400
#define TMTC_CHANNEL 65

// uplink
typedef struct __attribute__((packed)) {
    uint8_t id;
    int32_t param;
} telecommand_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;

    telecommand_payload_t payload;

    uint16_t checksum;
} telecommand_packet_t;

// downlink
typedef struct __attribute__((packed)) {
    uint32_t ut;

    float accel_mag;
    float ang_vel_mag;
    float pressure;
    float temperature;
    float altitude;

    int32_t lat_nmea, lon_nmea;
    uint8_t satellites;

    float v_bat;

    uint8_t phase;
} lora_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;

    lora_payload_t payload;

    uint16_t checksum;
} lora_packet_t;

// flash memory
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
    uint32_t magic;

    flash_payload_t payload;
} flash_packet_t;

// CRC-16-CCITT
uint16_t crc16(const uint8_t *data, size_t length);

#endif
