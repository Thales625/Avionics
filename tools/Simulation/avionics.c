#include "avionics.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#define DESCENT_IGNORE_TIME 10000 // ms
#define DESCENT_MAX_TIME 30000 // ms

typedef enum {
    STATE_PRE_FLIGHT,
    STATE_ASCENT,
    STATE_PARACHUTE_DEPLOY,
    STATE_DESCENT,
    STATE_SHUTDOWN
} flight_state_t;

static flight_state_t flight_state = STATE_PRE_FLIGHT;

static uint32_t ut0;

static float pressure_0;
static float altitude_baro=0.0f, max_altitude_baro=0.0f, prev_altitude_baro=0.0f;
static uint32_t parachute_ejection_count=0, descent_stable_count=0;

inline static void avionics_abort(void) {
    printf("ABORT!\n");
}

void setup(float pressure, float accel, uint32_t ut) {
     pressure_0 = pressure;  
     ut0 = ut;
}

void loop(float pressure, float accel, uint32_t ut) {
    // printf("d%s", text);
    printf("UT: %d | STATE: %d\n", ut, flight_state);

    // state machine
    switch (flight_state) {
        case STATE_PRE_FLIGHT:
            if (accel > 2.0f) flight_state = STATE_ASCENT;
            return;

        case STATE_ASCENT:
            if (pressure > 0.0f) {
                // filter
                altitude_baro = 0.9f * altitude_baro + 0.1f * (44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f)));

                // altitude_baro = 44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f));

                // printf("%.4f\n", altitude_baro); // DEBUG

                if (altitude_baro > max_altitude_baro) {
                    max_altitude_baro = altitude_baro;
                    parachute_ejection_count = 0; // reset count
                } else if (max_altitude_baro - altitude_baro >= 1.0f) {
                    parachute_ejection_count++;

                    if (parachute_ejection_count >= 5) { // EJECT
                        flight_state = STATE_PARACHUTE_DEPLOY;
                    }
                }
            }

            return;

        case STATE_PARACHUTE_DEPLOY:
            prev_altitude_baro = 0.9f * altitude_baro + 0.1f * (44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f)));
            ut0 = ut;

            flight_state = STATE_DESCENT;
            return;

        case STATE_DESCENT:
            altitude_baro = 0.9f * altitude_baro + 0.1f * (44330.0f * (1.0f - powf(pressure / pressure_0, 0.1903f)));

            if (ut - ut0 < DESCENT_IGNORE_TIME) { // ignore first N sec after parachute ejection
                prev_altitude_baro = altitude_baro;
                return;
            }

            if (ut - ut0 > DESCENT_MAX_TIME) { // check max descent time
                flight_state = STATE_SHUTDOWN;
                return;
            }

            // printf("%.4f\n", fabsf(altitude_baro - prev_altitude_baro)); // DEBUG

            // check stable baro altitude
            if (fabsf(altitude_baro - prev_altitude_baro) < 0.5f) {
                descent_stable_count++;
            } else {
                descent_stable_count = 0;
            }
            
            // |ACC| < MAX and altitude_baro stable
            if (sqrtf(accel*accel + accel*accel + accel*accel) < 1.1 && descent_stable_count > 3) {
                flight_state = STATE_SHUTDOWN;
                return;
            }

            return;

        case STATE_SHUTDOWN:
            return;
    }
}
