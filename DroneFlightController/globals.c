#include "globals.h"

uint32_t dTicks = 0, currentLoopTicks = 0, prevLoopTicks = 0;
float currentLoopTime = 0, dTime = 0, fps = 0;
float prevReceive = 0, prevRadioRestart = 0;
euler rotation = { 0, 0, 0 };
quat rotationQuat = { 0, 0, 0 ,0 };
euler target = { 0, 0, 0 };
float targetAltitude = 0;
vec3 worldAcc = { 0, 0, 0 };
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
float vSpeed = 0;
float altitude = 0, prevAltitude = 0;
LPF smoothBarVSpeed;
float hP = 0, hI = 0, hD = 0;
uint32_t LOAD_PERCENT = 0;
uint32_t MAX_LOAD_PERCENT = 0;