#ifndef __TMTC_H__
#define __TMTC_H__

#include <inttypes.h> // IWYU pragma: keep
#include <unistd.h>

#include "math_helper.h"

#define TELEMETRY_MAGIC 0xAABBCCDD
#define TELECOMMAND_MAGIC 0xDDCCBBAA

#define TMTC_CHANNEL 65

// downlink
typedef struct __attribute__((packed)) {
    uint32_t magic;

    uint32_t ut;
    uint8_t phase;
    vector3f_t accel;
    vector3f_t ang_vel;
    float pressure;
    float temperature;

    uint16_t checksum;
} telemetry_packet_t;

// uplink
typedef struct __attribute__((packed)) {
    uint32_t magic;

    uint8_t id;
    uint32_t param;

    uint16_t checksum;
} telecommand_packet_t;

// CRC-16-CCITT
uint16_t crc16(const uint8_t *data, size_t length);

#endif
