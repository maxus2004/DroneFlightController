#pragma once
#include <stdbool.h>

bool radio_receiveControls(float dTime, float* tx, float* ty, float* tz, float* th, bool* stabilization);
bool radio_controlsAvailable(void);
void radio_update(void);