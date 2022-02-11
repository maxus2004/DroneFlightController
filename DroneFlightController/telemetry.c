#include "telemetry.h"
#include "serial_port.h"
#include "math_extras.h"
#include <stdint.h>
#include <string.h>

void sendOrientation(quat* r) {
	uint8_t data[sizeof(float) * 4 + 1];
	memcpy(data, r, sizeof(float) * 4);
	data[sizeof(float) * 4] = '\n';
	serialPort_send(data, sizeof(float) * 4+1);
}