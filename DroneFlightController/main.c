#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_utils.h>
#include <stdbool.h>
#include <math.h>

#include "cpu_clock.h"
#include "IMU.h"
#include "time.h"
#include "math_extras.h"
#include "states.h"
#include "PID.h"
#include "string.h"
#include "filter.h"
#include "motors.h"
#include "radio.h"
#include "LPF.h"
#include "barometer.h"
#include "distance_sensors.h"

//parameters
bool dynamic_fps_calculation = false;
bool radio_enabled = true;
bool magnetometerEnabled = false;
bool sonarUpEnabled = true;
bool sonarDownEnabled = true;
bool heightStabilizationOn = false;
float altitudeKSonar = 0.1;
float altitudeKBar = 0.05;
float vSpeedK = 0.04;
float FILTER_K = 0.1;


uint32_t dTicks = 0, currentLoopTicks = 0, prevLoopTicks = 0;
float currentLoopTime = 0, dTime = 0, fps = 0;
float prevReceive = 0, prevRadioRestart = 0;
euler rotation = { 0, 0, 0 };
quat rotationQuat = { 0, 0, 0 ,0 };
euler target = { 0, 0, 0 };
float targetAltitude = 0;
vec3 worldAcc = { 0, 0, 0 };
LPF smoothXAcceleration;
LPF smoothYAcceleration;
LPF smoothZAcceleration;
float thrust = 0, prevThrust = 0;
float motors[] = { 0, 0, 0, 0 };
float motorsTarget[] = { 0, 0, 0, 0 };
euler torque = { 0, 0, 0 };
float pressure = 0, temperature = 0;
float sonarUpAlt = 0, prevSonarUpAlt = 0;
float sonarDownAlt = 0, prevSonarDownAlt = 0;
float barAlt = 0, prevBarAlt = 0;
bool newBarAlt = false;
bool newSonarUpAlt = false;
uint32_t lastSonarUpData = 0;
bool newSonarDownAlt = false;
uint32_t lastSonarDownData = 0;
LPF smoothBarVSpeed;
float vSpeed = 0;
float altitude = 0, prevAltitude = 0;
uint32_t orientationFailTimeout = 10*ticksPerSecond;

enum altitudeMode_t {
	BAROMETER = 0,
	SONAR_UP,
	SONAR_DOWN
};

enum altitudeMode_t prevAltitideCalcMode = BAROMETER;
enum altitudeMode_t altitideCalcMode = BAROMETER;
float hP = 0, hI = 0, hD = 0;
float absAcc = 0;
uint32_t LOAD_PERCENT = 0;
uint32_t MAX_LOAD_PERCENT = 0;

void emergencyLoop() {
	stopMotors();
	delayMillis(1);
}

void startEmergencyLoop() {
	while (1) {
		emergencyLoop();
	}
}

void sendTelemetry() {
	typedef struct
	{
		uint8_t packetID;
		euler rotation;
		euler target;
		vec3 absAcceleration;
		float thrust;
		float altitude;
		float sonarDownAlt;
		float targetAltitude;
	} __attribute__((packed)) s_telemetry;

	s_telemetry telemetry =
	{
		77,
		rotation,
		target,
		worldAcc,
		thrust,
		altitude,
		sonarDownAlt,
		targetAltitude,
	};
	radio_sendPacket(&telemetry, sizeof(s_telemetry));
}
void sendTelemetryFormat() {
	char format[][32] = {
	"euler rotation;",
	"euler target;",
	"vector3 absAcceleration;",
	"float thrust;",
	"float altitude;",
	"float sonarAlt;",
	"float targetAltitude;"
	};
	for (int i = 0; i < sizeof(format) / sizeof(format[0]); i++) {
		char packet[64];
		packet[0] = 88;
		packet[1] = i;
		memcpy(packet + 2, format[i], strlen(format[i]));

		radio_sendPacket(packet, strlen(format[i]) + 2);
		delayMillis(50);
		radio_sendPacket(packet, strlen(format[i]) + 2);
		delayMillis(50);
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
	if (radio_irq())
	{
		radio_processInterrupt();
	}

	if (radio_newData())
	{
		bool newHeightStabilizationOn;
		float thrustChange;
		float time = getSeconds();
		bool success = radio_receiveControls(time - prevReceive, &target.roll, &target.pitch, &target.yaw, &thrustChange, &newHeightStabilizationOn);
		if (success)
		{
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

			sendTelemetry();
		}
		else
		{
			radio_startReceiving();
		}
	}
}
void calculateTime() {
	prevLoopTicks = currentLoopTicks;
	currentLoopTicks = getTicks();
	currentLoopTime = (float)currentLoopTicks / (float)ticksPerSecond;
	if (dynamic_fps_calculation) {
		if (currentLoopTicks >= prevLoopTicks) {
			dTicks = currentLoopTicks - prevLoopTicks;
			// do not calculate new dTicks value if timer overflowed
		}
		dTime = (float)dTicks / (float)ticksPerSecond;
		fps = (float)ticksPerSecond / (float)dTicks;
	}
	else {
		dTicks = ticksPerSecond / IMU_UPDATE_FREQUENCY;
		dTime = 1.0f / IMU_UPDATE_FREQUENCY;
		fps = IMU_UPDATE_FREQUENCY;
	}
}
void readData() {
	vec3 acc, gyr, mag;
	bool mag_available;

	uint32_t startedWaiting = getTicks();
	IMU_waitForNewData();
	uint32_t stopedWaiting = getTicks();
	IMU_readData(&acc, &gyr, &mag, &mag_available);

	if (dTicks > 0) {
		LOAD_PERCENT = 100 - 100 * (stopedWaiting - startedWaiting) / dTicks;
		if (LOAD_PERCENT > MAX_LOAD_PERCENT)MAX_LOAD_PERCENT = LOAD_PERCENT;
	}

	if (fps > 0)
	{
		Madgwick_setKoeff(fps, FILTER_K);
		if (magnetometerEnabled)
			Madgwick_update(&gyr, &acc, &mag);
		else
			Madgwick_updateIMU(&gyr, &acc);
	}

	rotationQuat = Madgwick_readQuaternions();
	rotation = quatToEuler(rotationQuat);

	float aw = 0, ax = acc.x, ay = acc.y, az = acc.z;
	float qw = rotationQuat.w, qx = rotationQuat.x, qy = rotationQuat.y, qz = rotationQuat.z;
	float qw1 = qw, qx1 = -qx, qy1 = -qy, qz1 = -qz;

	float bw = qw * aw - qx * ax - qy * ay - qz * az;
	float bx = qw * ax + qx * aw + qy * az - qz * ay;
	float by = qw * ay - qx * az + qy * aw + qz * ax;
	float bz = qw * az + qx * ay - qy * ax + qz * aw;

	float b1x = bw * qx1 + bx * qw1 + by * qz1 - bz * qy1;
	float b1y = bw * qy1 - bx * qz1 + by * qw1 + bz * qx1;
	float b1z = bw * qz1 + bx * qy1 - by * qx1 + bz * qw1;

	b1x *= 1.0033839f;
	b1y *= 1.0033839f;
	b1z *= 1.0033839f;

	worldAcc.x = b1x;
	worldAcc.y = b1y;
	worldAcc.z = b1z - 9.81f;

	absAcc = sqrtf(ax * ax + ay * ay + az * az);

	LPF_update(&smoothXAcceleration, worldAcc.x);
	LPF_update(&smoothYAcceleration, worldAcc.y);
	LPF_update(&smoothZAcceleration, worldAcc.z);

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


void calculateAltitude() {
	prevAltitude = altitude;
	prevAltitideCalcMode = altitideCalcMode;

	//select best altitude calculation mode

	if (sonarDownEnabled
		&& currentLoopTicks < lastSonarDownData + ticksPerSecond / 5
		&& sonarDownAlt < 4
		&& fabsf(rotation.pitch) < 15 && fabsf(rotation.roll) < 15)
	{
		altitideCalcMode = SONAR_DOWN;
	}
	else if (sonarUpEnabled
		&& currentLoopTicks < lastSonarUpData + ticksPerSecond / 5
		&& sonarUpAlt > -4
		&& fabsf(rotation.pitch) < 15 && fabsf(rotation.roll) < 15)
	{
		altitideCalcMode = SONAR_UP;
	}
	else
	{
		altitideCalcMode = BAROMETER;
	}

	//calculate vSpeed
	if (dTime > 0) {
		vSpeed += worldAcc.z * dTime;
		if (newBarAlt) {
			LPF_update(&smoothBarVSpeed, barAlt - prevBarAlt);
			if (vSpeed < smoothBarVSpeed.value)
				vSpeed += vSpeedK * BAROMETER_INTERVAL;
			else {
				vSpeed -= vSpeedK * BAROMETER_INTERVAL;
			}
		}
	}

	//calculate altitude
	if (altitideCalcMode == SONAR_DOWN)
	{
		if (prevAltitideCalcMode != SONAR_DOWN)
		{
			altitude -= prevAltitude - sonarDownAlt;
			if (heightStabilizationOn)
			{
				targetAltitude = targetAltitude - (prevAltitude - altitude);
			}
		}

		if (dTime > 0)
		{
			
			altitude += vSpeed * dTime;
			if (newSonarDownAlt) {
				if (altitude < sonarDownAlt)
					altitude += altitudeKSonar * SONAR_INTERVAL;
				else {
					altitude -= altitudeKSonar * SONAR_INTERVAL;
				}
			}
		}

	}
	else if (altitideCalcMode == SONAR_UP)
	{
		if (prevAltitideCalcMode != SONAR_UP)
		{
			altitude -= prevAltitude - sonarUpAlt;
			if (heightStabilizationOn)
			{
				targetAltitude = targetAltitude - (prevAltitude - altitude);
			}
		}

		if (dTime > 0)
		{
			altitude += vSpeed * dTime;
			if (newSonarUpAlt) {
				if (altitude < sonarUpAlt)
					altitude += altitudeKSonar * SONAR_INTERVAL;
				else {
					altitude -= altitudeKSonar * SONAR_INTERVAL;
				}
			}
		}

	}
	else
	{
		if (prevAltitideCalcMode != BAROMETER)
		{
			altitude -= prevAltitude - barAlt;
			if (heightStabilizationOn)
			{
				targetAltitude = targetAltitude - (prevAltitude - altitude);
			}
		}

		if (dTime > 0)
		{
			altitude += vSpeed * dTime;
			if (newBarAlt) {
				if (altitude < barAlt)
					altitude += altitudeKBar * BAROMETER_INTERVAL;
				else {
					altitude -= altitudeKBar * BAROMETER_INTERVAL;
				}
			}
		}

	}
}
void updateMotors() {
	PID_update(dTime, rotation, target, &torque);
	float m1 = thrust + torque.roll + torque.pitch - torque.yaw;
	float m2 = thrust - torque.roll + torque.pitch + torque.yaw;
	float m3 = thrust - torque.roll - torque.pitch - torque.yaw;
	float m4 = thrust + torque.roll - torque.pitch + torque.yaw;
	if (m1 < 0.005)m1 = 0.005;
	if (m2 < 0.005)m2 = 0.005;
	if (m3 < 0.005)m3 = 0.005;
	if (m4 < 0.005)m4 = 0.005;
	motors[0] = sqrtf(m1);
	motors[1] = sqrtf(m2);
	motors[2] = sqrtf(m3);
	motors[3] = sqrtf(m4);
	setMotors(motors);
}
void sanityCheck() {
	// prevent randomly flying to the ceiling
	if (thrust - prevThrust > 0.2f)
	{
		STATE = EMERGENCY;
		ERROR_ID = TOO_HIGH_THRUST_CHANGE;
		startEmergencyLoop();
	}

	//prevent flying around the room while cutting everywhing with propellers if can't control orientation
	if (fabsf(target.pitch - rotation.pitch) < 10.0f
		&& fabsf(target.roll - rotation.roll) < 10.0f) 
	{
		orientationFailTimeout = currentLoopTicks;
	}

	if (currentLoopTicks - orientationFailTimeout > ticksPerSecond*2)
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

float prevFILTER_K = -1;
bool barometerZeroSet = false;
void startingLoop() {
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
	if (currentLoopTime > 8)
	{
		FILTER_K = prevFILTER_K;
		target.yaw = rotation.yaw;
		altitude = barAlt;
		vSpeed = 0;
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
	if (thrust > 0)
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
		motors[0] = 0;
		motors[1] = 0;
		motors[2] = 0;
		motors[3] = 0;
	}
	sanityCheck();
}
void flyingLoop() {
	calculateTime();
	readData();
	prevThrust = thrust;
	if (radio_enabled)radioReceiveSend();
	if (currentLoopTime - prevReceive > 0.5)
	{
		STATE = AUTO_LANDING;
	}
	calculateAltitude();
	if (heightStabilizationOn)
	{
		thrustPID();
	}
	if (thrust <= 0)
	{
		PID_doIntegration = false;
		targetAltitude = 0;
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
		targetAltitude -= dTime * 2;
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
	if (radio_enabled)
		radio_init();
	IMU_init();
	IMU_calibrateGyro();
	//IMU_calibrateAccel();
	barometer_init();
	distance_init(sonarDownEnabled, sonarUpEnabled);
	PID_init();
	setupMotors(2048);
}

int main(void)
{
	LPF_init(&smoothBarVSpeed, 50);
	LPF_init(&smoothZAcceleration, 1000);
	LPF_init(&smoothXAcceleration, 1000);
	LPF_init(&smoothYAcceleration, 1000);

	startup();
	//if (radio_enabled)sendTelemetryFormat();
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