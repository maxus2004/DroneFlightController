#pragma once
#include "math_extras.h"

quat Madgwick_readQuaternions();
void Madgwick_reset();
void Madgwick_setKoeff(float sampleFreq, float beta);
void Madgwick_update(vec3 *gyr, vec3* acc, vec3* mag);
void Madgwick_updateIMU(vec3* gyr, vec3* acc);