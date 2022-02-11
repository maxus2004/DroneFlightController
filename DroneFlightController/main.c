#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_utils.h>
#include <stdbool.h>
#include <math.h>

#include "cpu_clock.h"
#include "IMU.h"
#include "time.h"
#include "math_extras.h"
#include "world_acceleration.h"
#include "states.h"
#include "PID.h"
#include "string.h"
#include "filter.h"
#include "motors.h"
#include "radio.h"
#include "radio_hardware.h"
#include "LPF.h"
#include "barometer.h"
#include "distance_sensors.h"
#include "altitude.h"
#include "parameters.h"
#include "globals.h"
#include "serial_port.h"
#include "telemetry.h"

void emergencyLoop() {
	stopMotors();
	delayMillis(1);
}

void startEmergencyLoop() {
	while (true) {
		emergencyLoop();
	}
}

void setAltitudeStabilisation(bool enabled) {
	heightStabilizationOn = enabled;
	if (enabled)
	{
		targetAltitude = altitude;
		hI = thrust;
	}
}

void radioReceiveSend() {
	radio_update();
	if (!radio_controlsAvailable()) return;

	bool newHeightStabilizationOn;
	float thrustChange;
	float time = getSeconds();
	bool success = radio_receiveControls(time - prevReceive, &target, &thrustChange, &newHeightStabilizationOn);

	if (!success) return;

	prevReceive = time;

	if (newHeightStabilizationOn != heightStabilizationOn)
	{
		setAltitudeStabilisation(newHeightStabilizationOn);
	}

	if (heightStabilizationOn)
	{
		targetAltitude += thrustChange;
	}
	else
	{
		thrust += thrustChange;
		if (thrust > 1) thrust = 1;
		else if (thrust < 0) thrust = 0;
	}

	if (STATE == AUTO_LANDING)STATE = FLYING;

	radio_sendBasicTelemetry(batteryVolates);
}
void calculateTime() {
	prevLoopTicks = currentLoopTicks;
	currentLoopTicks = getTicks();
	currentLoopTime = (float)currentLoopTicks / (float)ticksPerSecond;
	if (dynamic_fps_calculation)
	{
		// do not calculate new dTicks value if timer overflowed
		if (currentLoopTicks >= prevLoopTicks)
		{
			dTicks = currentLoopTicks - prevLoopTicks;
		}
		dTime = (float)dTicks / (float)ticksPerSecond;
		fps = (float)ticksPerSecond / (float)dTicks;
	}
	else
	{
		dTicks = ticksPerSecond / IMU_UPDATE_FREQUENCY;
		dTime = 1.0f / IMU_UPDATE_FREQUENCY;
		fps = IMU_UPDATE_FREQUENCY;
	}
}

void readData() {
	bool mag_available;

	uint32_t startedWaiting = getTicks();
	IMU_waitForNewData();
	uint32_t stopedWaiting = getTicks();
	IMU_readData(&ACC, &GYR, &MAG, &mag_available);

	if (dTicks > 0) {
		LOAD_PERCENT = 100 - 100 * (stopedWaiting - startedWaiting) / dTicks;
		if (LOAD_PERCENT > MAX_LOAD_PERCENT)MAX_LOAD_PERCENT = LOAD_PERCENT;
	}

	if (fps > 0)
	{
		Madgwick_setKoeff(fps, FILTER_K);
		if (magnetometerEnabled)
			Madgwick_update(&GYR, &ACC, &MAG);
		else
			Madgwick_updateIMU(&GYR, &ACC);
	}

	rotationQuat = Madgwick_readQuaternions();
	if (currentLoopTicks - prevOrientationSend > telemetryInterval) {
		prevOrientationSend = currentLoopTicks;
		sendOrientation(&rotationQuat);
	}
	rotation = quatToEuler(rotationQuat);
	rotationV.roll = GYR.x * 57.29f;
	rotationV.pitch = GYR.y * 57.29f;
	rotationV.yaw = GYR.z * 57.29f;
	worldAcc = calculateWorldAcc(rotationQuat, ACC);
	prevBarAlt = barAlt;

	if (barometer_available()) {
		barometer_readAll(&pressure, &barAlt, &temperature);
		newBarAlt = true;
	}
	else {
		newBarAlt = false;
	}

	prevSonarUpAlt = sonarUpAlt;
	prevSonarDownAlt = sonarDownAlt;
	float distUp = -1, distDown = -1;
	readDistances(&distUp, &distDown);
	if (distUp > 0) {
		sonarUpAlt = -distUp;
		lastSonarUpData = currentLoopTicks;
	}
	newSonarUpAlt = distUp > 0;
	if (distDown > 0) {
		sonarDownAlt = distDown;
		lastSonarDownData = currentLoopTicks;
	}
	newSonarDownAlt = distDown > 0;
}


void updateMotors() {
	PID_update(dTime, rotation, rotationV, target, &torque);
	float m1 = thrust + torque.roll + torque.pitch + torque.yaw;
	float m2 = thrust - torque.roll + torque.pitch - torque.yaw;
	float m3 = thrust - torque.roll - torque.pitch + torque.yaw;
	float m4 = thrust + torque.roll - torque.pitch - torque.yaw;
	if (m1 < 0.005f)m1 = 0.005f;
	if (m2 < 0.005f)m2 = 0.005f;
	if (m3 < 0.005f)m3 = 0.005f;
	if (m4 < 0.005f)m4 = 0.005f;
	motors[0] = sqrtf(m1);
	motors[1] = sqrtf(m2);
	motors[2] = sqrtf(m3);
	motors[3] = sqrtf(m4);

	setMotors(motors);
}

void sanityCheck() {
	// TOO_HIGH_THRUST_CHANGE
	if (thrust - prevThrust > 0.2f)
	{
		STATE = EMERGENCY;
		ERROR_ID = TOO_HIGH_THRUST_CHANGE;
		startEmergencyLoop();
	}

	// ORIENTATION_FAIL
	if (thrust < 0.1f ||
		(fabsf(target.pitch - rotation.pitch) < 10.0f
			&& fabsf(target.roll - rotation.roll) < 10.0f))
	{
		orientationFailTimeout = currentLoopTicks;
	}

	if (currentLoopTicks - orientationFailTimeout > ticksPerSecond * 2.0f)
	{
		STATE = EMERGENCY;
		ERROR_ID = ORIENTATION_FAIL;
		startEmergencyLoop();
	}
}
void thrustPID() {
	float Kp = 0.3f;
	float Ki = 0.0f;
	float Kd = 0.3f;

	float e = targetAltitude - altitude;
	hP = Kp * e;
	hI = hI + Ki * dTime * (e);
	hD = Kd * -vSpeed;

	thrust = hP + hI + hD;
	if (thrust > 1)thrust = 1;
	if (thrust < 0)thrust = 0;
}

void startingLoop() {
	static float prevFILTER_K = -1;
	static bool barometerZeroSet = false;

	if (prevFILTER_K < 0) {
		prevFILTER_K = FILTER_K;
		FILTER_K = 1;
	}

	calculateTime();
	readData();
	prevThrust = thrust;
	if (!barometerZeroSet && currentLoopTime > 5.0f)
	{
		barometerZeroSet = true;
		barometer_zeroOffset();
	}
	if (currentLoopTime > 8.0f)
	{
		FILTER_K = prevFILTER_K;
		target.yaw = rotation.yaw;
		altitude = barAlt;
		vSpeed = 0.0f;
		STATE = LANDED;
		prevReceive = currentLoopTime;
		if (radio_enabled)radio_startReceiving();
	}
	calculateAltitude();
	sanityCheck();
}
void landedLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	if (radio_enabled)radioReceiveSend();
	calculateAltitude();
	if (thrust > 0.0f)
	{
		if (thrust > 0.2f)
		{
			STATE = EMERGENCY;
			ERROR_ID = TOO_HIGH_THRUST_ON_STARTUP;
			return;
		}
		STATE = FLYING;
		target.yaw = rotation.yaw;
		PID_doIntegration = true;
		pid_values.i.yaw = 0;
		pid_values.i.pitch = 0;
		pid_values.i.roll = 0;
	}
	else
	{
		stopMotors();
		for (int i = 0; i < 4; i++)
			motors[i] = 0;
	}
	sanityCheck();
}
void flyingLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	if (radio_enabled)radioReceiveSend();
	if (currentLoopTime - prevReceive > 0.5f)
	{
		STATE = AUTO_LANDING;
	}
	calculateAltitude();
	if (heightStabilizationOn)
	{
		thrustPID();
	}
	if (thrust <= 0.0f)
	{
		PID_doIntegration = false;
		targetAltitude = 0.0f;
		STATE = LANDED;
	}
	sanityCheck();
	updateMotors();
}
void autolandingLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	if (radio_enabled)radioReceiveSend();
	calculateAltitude();
	if (heightStabilizationOn)
	{
		targetAltitude -= dTime * 2.0f;
		thrustPID();
	}
	else
	{
		thrust -= dTime * 0.2f;
	}
	target.pitch = 0;
	target.roll = 0;

	if (thrust <= 0)
	{
		PID_doIntegration = false;
		targetAltitude = 0;
		STATE = LANDED;
	}

	sanityCheck();
	updateMotors();
}

void startup() {

	initCpuClock();
	initTickCounter();

	delaySeconds(1);

	STATE = STARTING;
	serialPort_init();
	if (radio_enabled)
		radio_init();
	IMU_init();
	IMU_calibrateGyro();
	if (calibrateAccel)
		IMU_calibrateAccel();
	barometer_init();
	distance_init(sonarDownEnabled, sonarUpEnabled);
	PID_init();
	setupMotors(2048);
}

int main(void)
{
	LPF_init(&smoothBarVSpeed, 500);

	startup();
	currentLoopTicks = getTicks();
	prevLoopTicks = getTicks();

	while (1)
	{
		switch (STATE) {
		case STARTING:
			startingLoop();
			break;
		case LANDED:
			landedLoop();
			break;
		case FLYING:
			flyingLoop();
			break;
		case AUTO_LANDING:
			autolandingLoop();
			break;
		case EMERGENCY:
			startEmergencyLoop();
			break;
		}
	}
}