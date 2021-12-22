#pragma once
#include <stdbool.h>

#define BAROMETER_INTERVAL 0.020f

void barometer_init(void);
float barometer_temp(void);
float barometer_press(void);
float barometer_alt(float pressure, float temperature);
void barometer_readAll(float* press, float* alt, float* temp);
bool barometer_available();
void barometer_zeroOffset();