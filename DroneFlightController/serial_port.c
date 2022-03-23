#include <stm32f4xx_ll_usart.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_bus.h>
#include <stdbool.h>
#include <string.h>
#include "serial_port.h"
#include "globals.h"
#include "time.h"


static bool sending = false;
static uint8_t txBuffer[128];
static int txIndex = 0;
static int txLen = 0;
static int maxTxLen = 128;
static uint8_t rxBuffer[128];
static int rxIndex = 0;
static int rxLen = 0;
static int maxRxLen = 128;

uint32_t lastRxPacketTime = 0;

typedef struct {
	uint8_t header[8];
	float yaw;
	float pitch;
	float roll;
	float thrust;
} rxPacket_t;

uint8_t rxPacketHeader[8] = { 50,51,52,53,54,55,56,57 };

rxPacket_t lastRxPacket;

void processPacket(rxPacket_t* packet) {
	lastRxPacketTime = currentLoopTicks;
	controlMode = SEMI_AUTO;
	memcpy(&lastRxPacket, packet, sizeof(rxPacket_t));
}

euler getSemiAutoTarget() {
	euler target = {lastRxPacket.yaw,lastRxPacket.pitch,lastRxPacket.roll};
	return target;
}

void USART2_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_RXNE(USART2)) {
		//new byte interrupt
		uint8_t newByte = USART2->DR;

		if (rxLen < 8) {
			if (newByte != rxPacketHeader[rxLen]) {
				rxLen = 0;
			}
			else {
				rxBuffer[rxLen] = newByte;
				rxLen++;
			}
		}
		else {
			if (rxLen < maxRxLen) {
				rxBuffer[rxLen] = newByte;
				rxLen++;

				if (rxLen >= sizeof(rxPacket_t)) {
					processPacket((rxPacket_t*)rxBuffer);
					rxLen = 0;
				}
			}
			else {
				rxLen = 0;
			}
		}
	}
	else if (sending && LL_USART_IsActiveFlag_TXE(USART2)) {
		//byte sent interrupt
		USART2->DR = txBuffer[txIndex];
		txIndex++;
		if (txIndex == txLen) {
			sending = false;
			LL_USART_DisableIT_TXE(USART2);
		}
	}
	else {
		//error interrupt
		(void)USART2->DR;
	}
}

void serialPort_init() {

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);

	NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	NVIC_EnableIRQ(USART2_IRQn);

	LL_USART_SetBaudRate(USART2, SystemCoreClock / 2, LL_USART_OVERSAMPLING_16, 115200);
	LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
	LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
	LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
	LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
	LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
	LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);
	LL_USART_ConfigAsyncMode(USART2);
	LL_USART_Enable(USART2);
	LL_USART_EnableIT_RXNE(USART2);
	LL_USART_EnableIT_ERROR(USART2);
}


bool serialPort_sendLine(void* data, int length) {
	if (sending) {
		if (txLen + length + 1 > maxTxLen) return false;
		memcpy(txBuffer + txLen, data, length);
		txBuffer[txLen + length] = '\n';
		txLen += length + 1;
	}
	else {
		if (length + 1 > maxTxLen) return false;
		memcpy(txBuffer, data, length);
		txBuffer[length] = '\n';
		txLen = length + 1;
		sending = true;
		txIndex = 0;
		LL_USART_EnableIT_TXE(USART2);
	}
	return true;
}

bool serialPort_send(void* data, int length) {
	if (sending) {
		if (txLen + length > maxTxLen) return false;
		memcpy(txBuffer + txLen, data, length);
		txLen += length;
	}
	else {
		if (length > maxTxLen) return false;
		memcpy(txBuffer, data, length);
		txLen = length;
		sending = true;
		txIndex = 0;
		LL_USART_EnableIT_TXE(USART2);
	}
	return true;
}