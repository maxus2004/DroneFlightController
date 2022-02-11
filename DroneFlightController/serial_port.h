#pragma once
#include <stdbool.h>

void serialPort_init();
bool serialPort_send(void* data, int length);