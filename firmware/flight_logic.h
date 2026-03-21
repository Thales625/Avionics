#ifndef __FLIGHT_LOGIC_H__
#define __FLIGHT_LOGIC_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef SIMULATION_BUILD
typedef void (*sim_log_callback_t)(const char *msg);
void flight_logic_set_sim_logger(sim_log_callback_t cb);

void sim_log_internal(const char *fmt, ...);

#ifndef CTYPESGEN
#define SIM_LOG(fmt, ...) sim_log_internal(fmt, ##__VA_ARGS__)
#endif

#else
#define SIM_LOG(fmt, ...) ((void)0)
#endif

typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

typedef enum {
    STATE_PRE_FLIGHT,
    STATE_ASCENT,
    STATE_PARACHUTE_DEPLOY,
    STATE_DESCENT,
    STATE_SHUTDOWN
} flight_state_t;

typedef struct {
    flight_state_t state;

    struct {
        uint32_t ut; // current time in ms
        vector3f_t accel;
        vector3f_t rot;
        float pressure;
        float temperature;
    } sensor_data;

    uint32_t ut_0;
    float pressure_0;

    float altitude_baro;
    float max_altitude_baro;
    float prev_altitude_baro;
    
    uint32_t parachute_ejection_count;

    bool trigger_parachute;
    bool trigger_shutdown;
} flight_logic_t;

void flight_logic_init(flight_logic_t *core);
void flight_logic_update(flight_logic_t *core);

#endif
