#include "telemetry.h"
#include "serial_port.h"
#include "math_extras.h"
#include <stdint.h>
#include <string.h>
#include "globals.h"

typedef struct{
	quat rotation;
	euler target;
} telemetryData_t;

void sendTelemetrySerial() {
	telemetryData_t data = {rotationQuat, target};
	serialPort_sendLine(&data, sizeof(data));
}