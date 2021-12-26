#include "altitude.h"
#include "globals.h"
#include "parameters.h"
#include "time.h"
#include "distance_sensors.h"
#include "barometer.h"
#include "math.h"

enum altitudeMode_t prevAltitideCalcMode = BAROMETER;
enum altitudeMode_t altitideCalcMode = BAROMETER;

void calculateAltitude() {
	prevAltitude = altitude;
	prevAltitideCalcMode = altitideCalcMode;

	//select best altitude calculation mode

	if (sonarDownEnabled
		&& currentLoopTicks < lastSonarDownData + ticksPerSecond / 5
		&& sonarDownAlt < sonar_maxDistance
		&& fabsf(rotation.pitch) < maxSonarAngle && fabsf(rotation.roll) < maxSonarAngle)
	{
		altitideCalcMode = SONAR_DOWN;
	}
	else if (sonarUpEnabled
		&& currentLoopTicks < lastSonarUpData + ticksPerSecond / 5
		&& sonarUpAlt > -sonar_maxDistance
		&& fabsf(rotation.pitch) < maxSonarAngle && fabsf(rotation.roll) < maxSonarAngle)
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