#include <distance_sensors.h>
#include <stm32f4xx_ll_usart.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_bus.h>
#include "time.h"

bool upEnabled = false;
bool downEnabled = false;

float latestUpDist = -1;
float isReadUp = false;
float latestDownDist = -1;
float isReadDown = false;

bool waitingDataUSART1 = false;
bool waitingDataUSART6 = false;

int USART1_BUFFER_LEN = 0;
uint8_t USART1_BUFFER[2];
int USART6_BUFFER_LEN = 0;
uint8_t USART6_BUFFER[2];

uint32_t lastPing = 0;

void USART1_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_RXNE(USART1) && waitingDataUSART1) {
		//new byte interrupt
		USART1_BUFFER[USART1_BUFFER_LEN] = LL_USART_ReceiveData8(USART1);
		USART1_BUFFER_LEN++;
		if (USART1_BUFFER_LEN == 2) {
			waitingDataUSART1 = false;
			USART1_BUFFER_LEN = 0;
			latestUpDist = ((float)(((uint16_t)USART1_BUFFER[0] << 8) | USART1_BUFFER[1])) / 1000;
			isReadUp = false;
		}
	}
	else {
		//error interrupt
		(void)USART1->DR;
		waitingDataUSART1 = false;
		USART1_BUFFER_LEN = 0;
	}
}

void USART6_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_RXNE(USART6) && waitingDataUSART6) {
		//new byte interrupt
		USART6_BUFFER[USART6_BUFFER_LEN] = LL_USART_ReceiveData8(USART6);
		USART6_BUFFER_LEN++;
		if (USART6_BUFFER_LEN == 2) {
			waitingDataUSART1 = false;
			USART6_BUFFER_LEN = 0;
			latestDownDist = ((float)(((uint16_t)USART6_BUFFER[0] << 8) | USART6_BUFFER[1])) / 1000;
			isReadDown = false;
		}
	}
	else {
		//error interrupt
		(void)USART6->DR;
		waitingDataUSART6 = false;
		USART6_BUFFER_LEN = 0;
	}
}

static void USART_sendByte(USART_TypeDef* USART, uint8_t byte)
{
	while (!LL_USART_IsActiveFlag_TXE(USART)) {}
	LL_USART_TransmitData8(USART, byte);
}


void pingDown() {
	USART6_BUFFER_LEN = 0;
	waitingDataUSART6 = true;
	USART_sendByte(USART6, 0x55);
}
void pingUp() {
	USART1_BUFFER_LEN = 0;
	waitingDataUSART1 = true;
	USART_sendByte(USART1, 0x55);
}

void readDistances(float* up, float* down) {
	uint32_t ticks = getTicks();
	if (ticks > lastPing + ticksPerSecond*0.01f) {
		lastPing = ticks;
		if (isReadUp && upEnabled) pingUp();
		if (isReadDown && downEnabled) pingDown();
	}

	if (isReadDown)
		*down = -1;
	else if (downEnabled) {
		*down = latestDownDist;
		isReadDown = true;
	}

	if (isReadUp)
		*up = -1;
	else if (upEnabled) {
		*up = latestUpDist;
		isReadUp = true;
	}
}

void distance_init(bool down_enabled, bool up_enabled) {
	downEnabled = down_enabled;
	upEnabled = up_enabled;
	if (down_enabled)
	{
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_VERY_HIGH);
		LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
		LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_12, LL_GPIO_PULL_NO);
		LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_12, LL_GPIO_AF_8);
		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_VERY_HIGH);
		LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);
		LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
		LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_11, LL_GPIO_AF_8);

		NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		NVIC_EnableIRQ(USART6_IRQn);

		LL_USART_SetBaudRate(USART6, SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600);
		LL_USART_SetDataWidth(USART6, LL_USART_DATAWIDTH_8B);
		LL_USART_SetStopBitsLength(USART6, LL_USART_STOPBITS_1);
		LL_USART_SetParity(USART6, LL_USART_PARITY_NONE);
		LL_USART_SetTransferDirection(USART6, LL_USART_DIRECTION_TX_RX);
		LL_USART_SetHWFlowCtrl(USART6, LL_USART_HWCONTROL_NONE);
		LL_USART_SetOverSampling(USART6, LL_USART_OVERSAMPLING_16);
		LL_USART_ConfigAsyncMode(USART6);
		LL_USART_Enable(USART6);
		LL_USART_EnableIT_RXNE(USART6);
		LL_USART_EnableIT_ERROR(USART6);
	}
	if (up_enabled)
	{
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);
		LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_7);
		LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
		LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
		LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);
		LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);

		NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		NVIC_EnableIRQ(USART1_IRQn);

		LL_USART_SetBaudRate(USART1, SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600);
		LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
		LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
		LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
		LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
		LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
		LL_USART_SetOverSampling(USART1, LL_USART_OVERSAMPLING_16);
		LL_USART_ConfigAsyncMode(USART1);
		LL_USART_Enable(USART1);
		LL_USART_EnableIT_RXNE(USART1);
		LL_USART_EnableIT_ERROR(USART1);
	}
}