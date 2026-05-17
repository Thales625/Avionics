#include "flight_logic.h"

#include <math.h>

#define DESCENT_MAX_TIME 30000 // ms
#define EJECTION_MAX_TIME 2000 // ms

#define EJECTION_MIN_ALTITUDE 10.0f
#define EJECTION_ALTITUDE_THRESHOLD 0.5f
#define EJECTION_CONFIRMATION_COUNT 5

#define LAUNCH_CONFIRMATION_COUNT 3
#define LAUNCH_ACC_THRESHOLD 2.0f

#define _acc_threshold2 (LAUNCH_ACC_THRESHOLD*LAUNCH_ACC_THRESHOLD)

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
#else
#include "esp_log.h"
static const char* TAG = "flight_logic";
#endif

void flight_logic_init(flight_logic_t *core) {
    core->state.phase = PHASE_PRE_FLIGHT;

    core->pressure_0 = core->state.pressure;

    core->altitude_baro = 0.0f;

    core->trigger_parachute = false;
    core->trigger_shutdown = false;

    core->should_arm = false;
}

void flight_logic_update(flight_logic_t *core) {
    static float prev_pressure = 0.0f;
    static float max_altitude_baro = -999.9f;
    static uint32_t liftoff_count = 0;
    static uint32_t parachute_EJECTION_CONFIRMATION_COUNT = 0;
    static uint32_t descent_time = 0;
    static uint32_t ut_0 = 0;

    SIM_LOG("|ACCEL|: %f", sqrtf(core->state.accel.x*core->state.accel.x + core->state.accel.y*core->state.accel.y + core->state.accel.z*core->state.accel.z));

    bool baro_updated = core->state.pressure != prev_pressure;
    if (baro_updated) {
        prev_pressure = core->state.pressure;
    }

    if (core->state.phase < PHASE_ASCENT) {
        // sensor update check
        if (baro_updated) {
            core->pressure_0 = (0.05f*core->state.pressure) + (0.95f*core->pressure_0);
        }
    }

    // update altitude (QFE)
    if (baro_updated) {
        core->altitude_baro = 44330.f*(1.f - powf(core->state.pressure/core->pressure_0, .1903f));
    }

    switch (core->state.phase) {
        case PHASE_WAITING:
            if (core->should_arm) {
                core->state.phase = PHASE_PRE_FLIGHT;
            }
            break;

        case PHASE_PRE_FLIGHT:
            if (core->state.accel.x*core->state.accel.x + core->state.accel.y*core->state.accel.y + core->state.accel.z*core->state.accel.z > _acc_threshold2) {
                liftoff_count++;
                if (liftoff_count >= LAUNCH_CONFIRMATION_COUNT) {
                    core->state.phase = PHASE_ASCENT;
                }
            } else {
                liftoff_count = 0;
            }

            ESP_LOGI(TAG, "ascent_count: %d", liftoff_count);
            break;

        case PHASE_ASCENT:
            // minimum altitude check
            if (core->altitude_baro < EJECTION_MIN_ALTITUDE) break;

            // sensor update check
            if (!baro_updated) break;

            if (core->altitude_baro > max_altitude_baro) {
                max_altitude_baro = core->altitude_baro;
                parachute_EJECTION_CONFIRMATION_COUNT = 0; // reset count
            } else if (max_altitude_baro - core->altitude_baro >= EJECTION_ALTITUDE_THRESHOLD) {
                parachute_EJECTION_CONFIRMATION_COUNT++;

                if (parachute_EJECTION_CONFIRMATION_COUNT >= EJECTION_CONFIRMATION_COUNT) { // EJECT
                    core->state.phase = PHASE_PARACHUTE_DEPLOY;
                }
            }
            break;

        case PHASE_PARACHUTE_DEPLOY:
            core->trigger_parachute = true;
            ut_0 = core->state.ut;
            core->state.phase = PHASE_DESCENT;
            SIM_LOG("PARACHUTE DEPLOY");
            break;

        case PHASE_DESCENT:
            descent_time = core->state.ut - ut_0;

            if (descent_time > EJECTION_MAX_TIME) { // check ejection time
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