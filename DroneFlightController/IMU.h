//  SCK  <-  A5
//	MISO ->  A6
//	MOSI <-  A7
//	CSG  <-  A4
//	CSM  <-  B0
//	CSB  <-  B1
//	INTM ->  B2
//	INTB ->  B10

#pragma once
#include "math_extras.h"

void IMU_init();
void IMU_readData(vec3* acc, vec3* gyr, vec3* mag, bool* mag_available);
void IMU_calibrateGyro();
void IMU_calibrateAccel();
void IMU_waitForNewData();

#define IMU_UPDATE_FREQUENCY 7000;