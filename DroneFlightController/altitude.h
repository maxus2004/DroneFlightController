#pragma once
#include "math_extras.h"
#include "LPF.h"

extern LPF smoothBarVSpeed;

enum altitudeMode_t {
	BAROMETER = 0,
	SONAR_UP,
	SONAR_DOWN
};

void calculateAltitude();