#pragma once
#include <stdint.h>

#define ticksPerSecond 1000000

void initTickCounter();

uint32_t getTicks();
uint32_t getMillis();
uint64_t getMicros();
float getSeconds();

void delayTicks(uint32_t ticksDelay);
void delayMillis(uint32_t millis);
void delayMicros(uint64_t micros);
void delaySeconds(float seconds);

void waitUntilTicks(uint32_t targetTicks);
void waitUntilMillis(uint32_t targetMillis);
void waitUntilMicros(uint64_t targetMicros);
void waitUntilSeconds(float targetSeconds);