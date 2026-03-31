#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#include <inttypes.h> // IWYU pragma: keep

#include "math_helper.h"

#define TELEMETRY_MAGIC 0xAABBCCDD

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

#endif
