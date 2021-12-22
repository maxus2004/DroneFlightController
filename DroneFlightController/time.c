#include <stdint.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_cortex.h>
#include <stm32f4xx_ll_utils.h>
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_bus.h>
#include "time.h"

volatile uint32_t fullTicks = 0;
#define microTicks (TIM10->CNT)

void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM10->SR = ~TIM_SR_CC1IF;
	TIM10->CNT -= 32768;
	fullTicks++;
}

void initTickCounter() {
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM10);

	LL_TIM_SetPrescaler(TIM10, 83);
	LL_TIM_SetCounterMode(TIM10, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM10, 65535);
	LL_TIM_SetClockDivision(TIM10, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_DisableARRPreload(TIM10);
	LL_TIM_OC_SetMode(TIM10, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FROZEN);
	LL_TIM_OC_SetCompareCH1(TIM10, 32768);
	LL_TIM_OC_SetPolarity(TIM10, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_CC_DisableChannel(TIM10, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_DisableChannel(TIM10, LL_TIM_CHANNEL_CH1N);
	LL_TIM_OC_DisableFast(TIM10, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableIT_CC1(TIM10);
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	LL_TIM_GenerateEvent_UPDATE(TIM10);
	LL_TIM_EnableCounter(TIM10);

}

uint32_t getTicks() {
	return (fullTicks<<15) + microTicks;
}
uint32_t getMillis() {
	return getTicks() / (ticksPerSecond / 1000);
}
uint64_t getMicros() {
	return (uint64_t)getTicks() * 1000000 / ticksPerSecond;
}
float getSeconds() {
	return (float)getTicks() / (float)ticksPerSecond;
}

void delayTicks(uint32_t ticksDelay) {
	uint32_t start = getTicks();
	while (getTicks() - start < ticksDelay);
}
void delayMillis(uint32_t millis) {
	uint32_t start = getMillis();
	uint32_t a;
	while ((a = getMillis()) - start < millis);
}
void delayMicros(uint64_t micros) {
	uint64_t start = getMicros();
	while (getMicros() - start < micros);
}
void delaySeconds(float seconds) {
	float start = getSeconds();
	while (getSeconds() - start < seconds);
}

void waitUntilTicks(uint32_t targetTicks) {
	while (getTicks() < targetTicks);
}
void waitUntilMillis(uint32_t targetMillis) {
	while (getMillis() < targetMillis);
}
void waitUntilMicros(uint64_t targetMicros) {
	while (getMicros() < targetMicros);
}
void waitUntilSeconds(float targetSeconds) {
	while (getSeconds() < targetSeconds);
}