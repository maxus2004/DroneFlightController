#pragma once

#include <stdbool.h>
#include "math_extras.h"

typedef struct
{
	euler KP;
	euler KI;
	euler KD;
	euler max;
	euler min;
	euler maxI;
	euler minI;
} PID_params_s;


typedef struct
{
	euler E;
	euler prevE;
	euler prev;
	euler p;
	euler i;
	euler d;
} PID_values_s;

extern PID_params_s pid_params;
extern PID_values_s pid_values;

extern bool PID_doIntegration;

void PID_init(void);
void PID_update(float dTime, euler rotation, euler rotationV, euler target, euler* torque);
void PID_resetI(void);
