#include "radio.h"
#include "radio_hardware.h"

#include <stdint.h>
#include <string.h>

uint8_t data[256];
uint8_t data_length = 0;
bool controlsAvailable = false;

bool radio_controlsAvailable(void) {
	return controlsAvailable;
}

void radio_update(void) {
	radio_updateIRQ();
	if (radio_newPacket()) {
		data_length = radio_readPacket(data);

		switch (data[0]) {
		case 66:
			controlsAvailable = true;
			break;
		}
	}
}

bool radio_receiveControls(float dTime, euler* target, float* th, bool* stabilization)
{
	controlsAvailable = false;
	if (data[0] != 66)
	{
		return false;
	}

	float joysticks[4];
	memcpy(joysticks, data + 1, 16);


	if (joysticks[0] < -1.1 || joysticks[0] > 1.1 ||
		joysticks[1] < -1.1 || joysticks[1] > 1.1 ||
		joysticks[2] < -1.1 || joysticks[2] > 1.1 ||
		joysticks[3] < -1.1 || joysticks[3] > 1.1) {
		return false;
	}

	if (dTime > 0.2f) dTime = 0.2f;

	float tzv = joysticks[0] * 180;
	float thv = joysticks[1] * 1.5f;
	float ntx = joysticks[2] * 15;
	float nty = joysticks[3] * -15;


	float nth = thv * dTime;
	float ntz = target->yaw + tzv * dTime;
	if (ntz > 180)ntz = -180;
	if (ntz < -180)ntz = 180;

	*stabilization = (bool)data[17];

	target->roll = ntx;
	target->pitch = nty;
	target->yaw = ntz;
	*th = nth;

	return true;
}

void radio_sendBasicTelemetry(float *batteryVoltages) {
	uint8_t packet[1 + sizeof(float) * 3];
	packet[0] = 77;
	memcpy(packet + 1, batteryVoltages, sizeof(float) * 3);
	radio_sendPacket(packet, sizeof(packet));
}