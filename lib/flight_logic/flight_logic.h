#ifndef __FLIGHT_LOGIC_H__
#define __FLIGHT_LOGIC_H__

#ifndef SIMULATION_BUILD

#include <stdbool.h>
#include <stdint.h>

#define SIM_LOG(...) ((void)0)
#else
typedef void (*sim_log_callback_t)(const char *msg);
void flight_logic_set_sim_logger(sim_log_callback_t cb);
void sim_log_internal(const char *fmt, ...);

typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;

#ifndef CTYPESGEN
#define SIM_LOG(fmt, ...) sim_log_internal(fmt, ##__VA_ARGS__)
#endif

#endif

#include "math_helper.h"

typedef enum {
    PHASE_WAITING,
    PHASE_PRE_FLIGHT,
    PHASE_ASCENT,
    PHASE_PARACHUTE_DEPLOY,
    PHASE_DESCENT,
    PHASE_SHUTDOWN
} flight_phase_t;

typedef struct {
    uint32_t ut;
    flight_phase_t phase;
    vector3f_t accel;
    vector3f_t ang_vel;
    float pressure;
    float temperature;
    int32_t lat_nmea, lon_nmea;
    uint8_t satellites;
} flight_state_t;

typedef struct {
    flight_state_t state;

    uint32_t ut_0;
    float pressure_0;

    float altitude_baro;

    bool trigger_parachute;
    bool trigger_shutdown;

    bool should_arm;
} flight_logic_t;

void flight_logic_init(flight_logic_t *core);
void flight_logic_update(flight_logic_t *core);

#endif
