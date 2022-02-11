#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math_extras.h>
#include <LPF.h>

extern uint32_t dTicks, currentLoopTicks, prevLoopTicks;
extern uint32_t prevOrientationSend;
extern float currentLoopTime, dTime, fps;
extern float prevReceive, prevRadioRestart;
extern euler rotation;
extern euler rotationV;
extern quat rotationQuat;
extern euler target;
extern float targetAltitude;
extern vec3 GYR, ACC, MAG;
extern vec3 worldAcc;
extern float thrust, prevThrust;
extern float motors[4];
extern euler torque;
extern float pressure, temperature;
extern float sonarUpAlt, prevSonarUpAlt;
extern float sonarDownAlt, prevSonarDownAlt;
extern float barAlt, prevBarAlt;
extern bool newBarAlt;
extern bool newSonarUpAlt;
extern uint32_t lastSonarUpData;
extern bool newSonarDownAlt;
extern uint32_t lastSonarDownData;
extern float vSpeed;
extern float altitude, prevAltitude;
extern LPF smoothBarVSpeed;
extern float hP, hI, hD;
extern float batteryVolates[];
extern uint32_t LOAD_PERCENT;
extern uint32_t MAX_LOAD_PERCENT;