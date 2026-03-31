#include "flight_logic.h"

#include <math.h>

#define DESCENT_MAX_TIME 30000 // ms

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
    core->max_altitude_baro = 0.0f;
    core->prev_altitude_baro = 0.0f;
    
    core->parachute_ejection_count = 0;

    core->trigger_parachute = false;
    core->trigger_shutdown = false;
}

void flight_logic_update(flight_logic_t *core) {
    switch (core->state.phase) {
        case PHASE_PRE_FLIGHT:
			// if (fabs(core->state.accel.x) > 1.8f) core->state.phase = PHASE_ASCENT;
            
            if (sqrtf(core->state.accel.x*core->state.accel.x + core->state.accel.y*core->state.accel.y + core->state.accel.z*core->state.accel.z) > 1.8f) core->state.phase = PHASE_ASCENT;

            SIM_LOG("ACCEL: %f", core->state.accel.x);

            break;

        case PHASE_ASCENT:
			if (core->state.pressure > 0.0f) {
                core->altitude_baro = 44330.f*(1.f - powf(core->state.pressure/core->pressure_0, .1903f));

				if (core->altitude_baro > core->max_altitude_baro) {
					core->max_altitude_baro = core->altitude_baro;
					core->parachute_ejection_count = 0; // reset count
				} else if (core->max_altitude_baro - core->altitude_baro >= 1.0f) {
					core->parachute_ejection_count++;

					if (core->parachute_ejection_count >= 5) { // EJECT
						core->state.phase = PHASE_PARACHUTE_DEPLOY;
						core->ut_0 = core->state.ut;
					}
				}
                SIM_LOG("PRESSURE: %f", core->state.pressure);
			}
            break;

        case PHASE_PARACHUTE_DEPLOY:
			core->trigger_parachute = true;

            core->altitude_baro = 44330.f*(1.f - powf(core->state.pressure/core->pressure_0, .1903f));
			core->ut_0 = core->state.ut;

            SIM_LOG("PARACHUTE DEPLOY");

            core->state.phase = PHASE_DESCENT;
            break;

        case PHASE_DESCENT:
            if (core->state.ut - core->ut_0 > DESCENT_MAX_TIME) { // check max descent time
                core->state.phase = PHASE_SHUTDOWN;
                core->trigger_shutdown = true;
                break;
            }
            break;

        case PHASE_SHUTDOWN:
            core->trigger_shutdown = true;
            break;
    }
}
