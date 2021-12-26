#pragma once 
#include <stdbool.h>

//connections
//
//UP:
//USART1 - B6(TX), B7(TX)
//DOWN:
//USART6 - A11(TX), A12(TX)

#define SONAR_INTERVAL 0.1f
#define sonar_maxDistance 4

void readDistances(float* up, float* down);
void distance_init(bool down_enabled, bool up_enabled);