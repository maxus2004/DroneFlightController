//radio connections
//B10 ---> IRQ
//B12 ---> CS
//B13 ---> SCK
//B14 ---> MISO
//B15 ---> MOSI

#pragma once
#include <stdbool.h>
#include <stm32f401xc.h>

void radio_processInterrupt(void);
bool radio_irq(void);
bool radio_receiveControls(float dTime, float* tx, float* ty, float* tz, float* th, bool* stabilization);
void radio_init(void);
bool radio_newData(void);
void radio_sendPacket(void* data, uint8_t length);
uint8_t radio_readPacket(uint8_t* data);
void radio_startReceiving(void);
