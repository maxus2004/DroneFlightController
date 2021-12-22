#include "motors.h"
#include "time.h"
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_dma2d.h>
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>
#include <string.h>


uint32_t DSHOT_DMA_DATA[16];
uint32_t DSHOT_DMA_ON_MASK = (1 << 0) | (1 << 1) | (1 << 8) | (1 << 15);
uint32_t DSHOT_DMA_OFF_MASK = ((1 << 0) | (1 << 1) | (1 << 8) | (1 << 15)) << 16;
uint32_t motorPins[4] = { 1 << 0 << 16, 1 << 1 << 16, 1 << 8 << 16, 1 << 15 << 16 };
uint8_t motorCount = 4;
uint16_t MAX_THROTTLE = 48;
uint16_t MIN_THROTTLE = 48;

uint32_t prevSend = 0;
uint32_t cancelledDSHOTPackets = 0;

#define DSHOT_SPEED_DIVIDER 4

void setupMotors(int maxThrottle) {
	MAX_THROTTLE = maxThrottle;

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	uint32_t pins[] = { LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_8, LL_GPIO_PIN_15 };
	for (int i = 0; i < 4; i++) {
		LL_GPIO_ResetOutputPin(GPIOA, pins[i]);
		LL_GPIO_SetPinMode(GPIOA, pins[i], LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinOutputType(GPIOA, pins[i], LL_GPIO_OUTPUT_PUSHPULL);
		LL_GPIO_SetPinSpeed(GPIOA, pins[i], LL_GPIO_SPEED_FREQ_HIGH);
	}
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

	/* TIM1 DMA Init */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/* TIM1_UP Init */
	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)&GPIOA->BSRR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)&DSHOT_DMA_ON_MASK);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_WORD);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_WORD);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_5);
	/* TIM1_CH1 Init */
	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_6);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&GPIOA->BSRR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)&DSHOT_DMA_DATA);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_WORD);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_WORD);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
	/* TIM1_CH2 Init */
	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_6);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&GPIOA->BSRR);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&DSHOT_DMA_OFF_MASK);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_NOINCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_WORD);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_WORD);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	LL_TIM_SetPrescaler(TIM1, DSHOT_SPEED_DIVIDER - 1);
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM1, 138);
	LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_SetRepetitionCounter(TIM1, 0);
	LL_TIM_DisableARRPreload(TIM1);
	LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FROZEN);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FROZEN);
	LL_TIM_OC_SetCompareCH1(TIM1, 50);
	LL_TIM_OC_SetCompareCH2(TIM1, 100);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	LL_TIM_SetOffStates(TIM1, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_DISABLE);
	LL_TIM_CC_SetLockLevel(TIM1, LL_TIM_LOCKLEVEL_OFF);
	LL_TIM_OC_SetDeadTime(TIM1, 0);
	LL_TIM_DisableBRK(TIM1);
	LL_TIM_ConfigBRK(TIM1, LL_TIM_BREAK_POLARITY_HIGH);
	LL_TIM_DisableAutomaticOutput(TIM1);

	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, 16);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, 16);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, 16);

	/* DMA interrupt init */
	/* DMA2_Stream1_IRQn interrupt configuration */
	NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream5_IRQn interrupt configuration */
	NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

void DMA2_Stream1_IRQHandler(void) {
	LL_DMA_ClearFlag_TC1(DMA2);
}
void DMA2_Stream2_IRQHandler(void) {
	LL_DMA_ClearFlag_TC2(DMA2);
}
void DMA2_Stream5_IRQHandler(void) {
	LL_DMA_ClearFlag_TC5(DMA2);
}

void sendDSHOT(uint16_t values[]) {

	uint32_t ticks = getTicks();
	if (ticks - prevSend < ticksPerSecond/8000) {
		cancelledDSHOTPackets++;
		return;
	}
	prevSend = ticks;

	memset(DSHOT_DMA_DATA, 0, sizeof(DSHOT_DMA_DATA));
	for (int i = 0; i < motorCount; i++)
	{
		uint16_t data = (uint16_t)(values[i] << 5);

		uint8_t XOR = ((data >> 4) ^ (data >> 8) ^ (data >> 12)) % 16;
		data = data | XOR;

		for (int j = 0; j < 16; j++)
		{
			DSHOT_DMA_DATA[15 - j] |= ((data >> j) % 2) ? 0 : motorPins[i];
		}
	}


	LL_TIM_DisableDMAReq_UPDATE(TIM1);
	LL_TIM_DisableDMAReq_CC1(TIM1);
	LL_TIM_DisableDMAReq_CC2(TIM1);

	LL_TIM_DisableCounter(TIM1);
	LL_TIM_SetCounter(TIM1, LL_TIM_GetAutoReload(TIM1));

	LL_TIM_EnableDMAReq_UPDATE(TIM1);
	LL_TIM_EnableDMAReq_CC1(TIM1);
	LL_TIM_EnableDMAReq_CC2(TIM1);

	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);

	LL_TIM_EnableCounter(TIM1);
}

void setMotors(float motors[]) {
	uint16_t values[4];
	for (int i = 0; i < 4; i++) {
		if (values[i] < 0)values[i] = 0;
		if (values[i] > 1)values[i] = 1;
		values[i] = (uint16_t)((motors[i]) * (float)(MAX_THROTTLE - MIN_THROTTLE)) + MIN_THROTTLE;
	}
	sendDSHOT(values);
}

void stopMotors() {
	uint16_t values[4] = {0,0,0,0};
	sendDSHOT(values);
}
