#ifndef AVIONICS_H
#define AVIONICS_H

#include <inttypes.h>

void setup(float pressure, float accel, uint32_t time_s);
void loop(float pressure, float accel, uint32_t time_s);

#endif
