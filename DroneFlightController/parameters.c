#include "parameters.h"
#include <time.h>

bool calibrateAccel = false;
bool dynamic_fps_calculation = true;
bool radio_enabled = true;
bool magnetometerEnabled = false;
bool sonarUpEnabled = true;
bool sonarDownEnabled = true;
bool heightStabilizationOn = false;
bool loggingEnabled = false;
float altitudeKSonar = 0.1;
float altitudeKBar = 0.05;
float vSpeedK = 0.1;
float FILTER_K = 0.1;
uint32_t orientationFailTimeout = 10 * ticksPerSecond;
float maxSonarAngle = 15;
uint32_t telemetryInterval = ticksPerSecond/60;