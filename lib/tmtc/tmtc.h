#ifndef __TMTC_H__
#define __TMTC_H__

#include <inttypes.h> // IWYU pragma: keep

#define TELEMETRY_MAGIC   0x544C4D59 // "TLMY"
#define TELECOMMAND_MAGIC 0x54434D44 // "TCMD"

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

#endif
