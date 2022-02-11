#pragma once
#include <stdbool.h>
#include "math_extras.h"

bool radio_receiveControls(float dTime, euler* target, float* th, bool* stabilization);
bool radio_controlsAvailable(void);
void radio_update(void);
void radio_sendBasicTelemetry(float* batteryVoltages);