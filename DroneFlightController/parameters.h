#pragma once
#include <stdbool.h>
#include <stdint.h>

extern bool calibrateAccel;
extern bool dynamic_fps_calculation;
extern bool radio_enabled;
extern bool magnetometerEnabled;
extern bool sonarUpEnabled;
extern bool sonarDownEnabled;
extern bool heightStabilizationOn;
extern bool loggingEnabled;
extern float altitudeKSonar;
extern float altitudeKBar;
extern float vSpeedK;
extern float FILTER_K;
extern uint32_t orientationFailTimeout;
extern float maxSonarAngle;