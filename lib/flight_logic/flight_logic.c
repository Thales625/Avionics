#include "flight_logic.h"

#include <math.h>

#define DESCENT_MAX_TIME 30000 // ms
#define EJECTION_TIME 5000 // ms

#ifdef SIMULATION_BUILD
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>

static sim_log_callback_t sim_log_cb = NULL;

void flight_logic_set_sim_logger(sim_log_callback_t cb) {
    sim_log_cb = cb;
}

void sim_log_internal(const char *fmt, ...) {
    if (!sim_log_cb) return;

    char buffer[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    sim_log_cb(buffer);
}
#endif

void flight_logic_init(flight_logic_t *core) {
    core->state.phase = PHASE_PRE_FLIGHT;

    core->ut_0 = core->state.ut;
    core->pressure_0 = core->state.pressure;

    core->altitude_baro = 0.0f;

    core->trigger_parachute = false;
    core->trigger_shutdown = false;
}

void flight_logic_update(flight_logic_t *core) {
    static float prev_pressure = NAN;
    static float max_altitude_baro = -999.9f;
    static uint32_t parachute_ejection_count = 0;
    static uint32_t descent_time = 0;

    // update altitude
    core->altitude_baro = 44330.f*(1.f - powf(core->state.pressure/core->pressure_0, .1903f));

    switch (core->state.phase) {
        case PHASE_WAITING:
            if (core->should_arm)
                core->state.phase = PHASE_PRE_FLIGHT;
            break;

        case PHASE_PRE_FLIGHT:
            if (sqrtf(core->state.accel.x*core->state.accel.x + core->state.accel.y*core->state.accel.y + core->state.accel.z*core->state.accel.z) > 1.8f) {
                core->state.phase = PHASE_ASCENT;
                core->pressure_0 = core->state.pressure;
            }
            SIM_LOG("ACCEL: %f", core->state.accel.x);
            break;

        case PHASE_ASCENT:
            // minimum altitude check
            if (core->altitude_baro < 5.0f) return;

            // sensor update check
            if (core->state.pressure == prev_pressure) return;
            prev_pressure = core->state.pressure;

            if (core->altitude_baro > max_altitude_baro) {
                max_altitude_baro = core->altitude_baro;
                parachute_ejection_count = 0; // reset count
            } else if (max_altitude_baro - core->altitude_baro >= 1.0f) {
                parachute_ejection_count++;

                if (parachute_ejection_count >= 5) { // EJECT
                    core->state.phase = PHASE_PARACHUTE_DEPLOY;
                    core->ut_0 = core->state.ut;
                }
            }
            SIM_LOG("PRESSURE: %f", core->state.pressure);
            break;

        case PHASE_PARACHUTE_DEPLOY:
            core->trigger_parachute = true;

            core->ut_0 = core->state.ut;

            SIM_LOG("PARACHUTE DEPLOY");

            core->state.phase = PHASE_DESCENT;
            break;

        case PHASE_DESCENT:
            descent_time = core->state.ut - core->ut_0;

            if (descent_time > EJECTION_TIME) { // check ejection time
                core->trigger_parachute = false;
            }
            if (descent_time > DESCENT_MAX_TIME) { // check max descent time
                core->state.phase = PHASE_SHUTDOWN;
            }
            break;

        case PHASE_SHUTDOWN:
            core->trigger_parachute = false;
            core->trigger_shutdown = true;
            break;
    }
}
