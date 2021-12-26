//radio connections
//B10 ---> IRQ
//B12 ---> CS
//B13 ---> SCK
//B14 ---> MISO
//B15 ---> MOSI

#pragma once
#include <stdbool.h>
#include <stm32f401xc.h>

void radio_init(void);
bool radio_newPacket(void);
bool radio_sendPacket(void* data, uint8_t length);
uint8_t radio_readPacket(uint8_t* data);
bool radio_startReceiving(void);
void radio_updateIRQ(void);