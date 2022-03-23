#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "math_extras.h"

void serialPort_init();
bool serialPort_send(void* data, int length);
bool serialPort_sendLine(void* data, int length);

euler getSemiAutoTarget();

extern uint32_t lastRxPacketTime;