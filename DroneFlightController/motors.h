#pragma once

#include <stdint.h>

//  A0 --> M1
//  A1 --> M2
//  A8 --> M3
// A15 --> M4

void setupMotors(int maxThrottle);
void setMotors(float motors[]);
void stopMotors();
void sendDSHOT(uint16_t values[]);
