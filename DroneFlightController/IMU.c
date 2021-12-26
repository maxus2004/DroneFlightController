#include <stm32f401xc.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_spi.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_cortex.h>
#include <stm32f4xx_ll_utils.h>
#include <stdbool.h>
#include <math.h>
#include "IMU.h"
#include "math_extras.h"
#include "time.h"

float GYRO_OFFSET_X = 0, GYRO_OFFSET_Y = 0, GYRO_OFFSET_Z = 0;
float GYRO_SCALE_X = 0.001221731, GYRO_SCALE_Y = 0.001221731, GYRO_SCALE_Z = 0.001221731;
float ACC_OFFSET_X = -17.7800903, ACC_OFFSET_Y = 7.91796875, ACC_OFFSET_Z = -4.0949707;
float ACC_SCALE_X = 0.00477896724, ACC_SCALE_Y = 0.00480090128, ACC_SCALE_Z = 0.00479352335;

void IMU_GPIO_init() {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_VERY_HIGH);
}

void IMU_SPI_init() {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_5);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV16);
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
	LL_SPI_DisableCRC(SPI1);
	LL_SPI_Enable(SPI1);
}

#define csm1 GPIOB->BSRR = 1<<0
#define csm0 GPIOB->BSRR = 1<<(16+0)
#define csg1 GPIOA->BSRR = 1<<4
#define csg0 GPIOA->BSRR = 1<<(16+4)
#define intm ((GPIOB->IDR&(1<<2)) != 0)

static uint8_t SPI_sendReceive(uint8_t data) {
	while (SPI1->SR & SPI_SR_BSY);
	SPI1->DR = data;
	while (!(SPI1->SR & SPI_SR_TXE)) {}
	while (!(SPI1->SR & SPI_SR_RXNE)) {}
	return SPI1->DR;
}

void MAG_writeBurst(uint8_t reg, void* data, uint8_t length)
{
	csm0;
	SPI_sendReceive(reg);
	for (int i = 0; i < length; i++)
	{
		SPI_sendReceive(((uint8_t*)data)[i]);
	}
	csm1;
}

void MAG_readBurst(uint8_t reg, void* data, uint8_t length)
{
	csm0;
	SPI_sendReceive(0b11000000 | reg);
	for (int i = 0; i < length; i++)
	{
		((uint8_t*)data)[i] = SPI_sendReceive(0xff);
	}
	csm1;
}

void GYR_writeBurst(uint8_t reg, void* data, uint8_t length)
{
	csg0;
	SPI_sendReceive(reg);
	for (int i = 0; i < length; i++)
	{
		SPI_sendReceive(((uint8_t*)data)[i]);
	}
	csg1;
}

void GYR_readBurst(uint8_t reg, void* data, uint8_t length)
{
	csg0;
	SPI_sendReceive(0x80 | reg);
	for (int i = 0; i < length; i++)
	{
		((uint8_t*)data)[i] = SPI_sendReceive(0xff);
	}
	csg1;
}

uint8_t GYR_readReg(uint8_t reg) {
	csg0;
	SPI_sendReceive(reg | 0x80);
	uint8_t value = SPI_sendReceive(0xff);
	csg1;
	return value;
}

uint8_t MAG_readReg(uint8_t reg) {
	csm0;
	SPI_sendReceive(reg | 0x80);
	uint8_t value = SPI_sendReceive(0xff);
	csm1;
	return value;
}

void GYR_writeReg(uint8_t reg, uint8_t value) {
	csg0;
	SPI_sendReceive(reg);
	SPI_sendReceive(value);
	csg1;
}

void MAG_writeReg(uint8_t reg, uint8_t value) {
	csm0;
	SPI_sendReceive(reg);
	SPI_sendReceive(value);
	csm1;
}


void IMU_init() {
	IMU_GPIO_init();
	IMU_SPI_init();
	// magnetometer init
	if (MAG_readReg(0x0f) != 0b00111101) {
		// magnetometer not connected
		// stop program execution
		while (1);
	}
	MAG_writeReg(0x20, 0b01111110); // temperature off, ODR = 155Hz, xy UHP(3.5mgauus noise) mode
	MAG_writeReg(0x21, 0b00000000); // +- 4 gauss
	MAG_writeReg(0x22, 0b00000000); // LP mode off, 4 wire SPI, continuous converion mode
	MAG_writeReg(0x23, 0b00001100); // z UHP, LSB first
	MAG_writeReg(0x24, 0b01000000); // fast read off, block data update

		//gyro and accelerometer init
	if (GYR_readReg(0x0f) != 0b01101011) {
		// gyro not connected
		// stop program execution
		while (1);
	}
	GYR_writeReg(0x10, 0b10100100); //ACC: ODR = 6kHz, +-16G, BW = 1.5kHz
	GYR_writeReg(0x11, 0b10101100); //GYR: ODR = 6kHz, +-2000dps
	GYR_writeReg(0x12, 0b01000100); //reboot off, block data update, interrupt config(default), 4 wire spi, increment address, LSB first, reset off
	GYR_writeReg(0x13, 0b00000100); //disable i2c, gyro LPF disabled, interrupt config(default)
	GYR_writeReg(0x14, 0b01100000); //gyro + accel rounding, DEN active low (default)
	GYR_writeReg(0x15, 0b00000011); //DEN trigger disabled, accel high perf mode, gyro LPF 900hz if enabled
	GYR_writeReg(0x16, 0b00000000); //gyro high perf mode, gyro hpf disabled, hpf 16mHz if enabled
	GYR_writeReg(0x17, 0b00000000); //default
	GYR_writeReg(0x18, 0b00000000); //default
	GYR_writeReg(0x19, 0b00000000); //default
}

void IMU_waitForNewData() {
	while ((GYR_readReg(0x1e) & 0b00000011) != 0b00000011);
}

void IMU_readData(vec3* acc, vec3* gyr, vec3* mag, bool* mag_available) {
	//reading magnetometer
	if (intm) {
		int16_t MAG_data[3];
		MAG_readBurst(0x28, MAG_data, 6);
		mag->x = MAG_data[0] / 6842.0f;
		mag->y = MAG_data[1] / 6842.0f;
		mag->z = MAG_data[2] / 6842.0f;
		*mag_available = true;
	}
	else {
		*mag_available = false;
	}
	//reading gyro and accel
	int16_t GYRACC_data[6];
	GYR_readBurst(0x22, GYRACC_data, 12);
	gyr->x = (GYRACC_data[0] + GYRO_OFFSET_X) * GYRO_SCALE_X;
	gyr->y = (GYRACC_data[1] + GYRO_OFFSET_Y) * GYRO_SCALE_Y;
	gyr->z = (GYRACC_data[2] + GYRO_OFFSET_Z) * GYRO_SCALE_Z;
	acc->x = (GYRACC_data[3] + ACC_OFFSET_X) * ACC_SCALE_X;
	acc->y = (GYRACC_data[4] + ACC_OFFSET_Y) * ACC_SCALE_Y;
	acc->z = (GYRACC_data[5] + ACC_OFFSET_Z) * ACC_SCALE_Z;
}

void IMU_calibrateGyro() {
	// read gyro for some time to stabilize readings
	for (int i = 0; i < 6664; i++) {
		int16_t GYR_data[6];
		IMU_waitForNewData();
		GYR_readBurst(0x22, GYR_data, 12);
	}

	// read gyro and accumulate offset
	int64_t x = 0, y = 0, z = 0;
	int cycles = 6664;
	for (int i = 0; i < cycles; i++) {
		int16_t GYR_data[6];
		IMU_waitForNewData();
		GYR_readBurst(0x22, GYR_data, 12);
		x += GYR_data[0];
		y += GYR_data[1];
		z += GYR_data[2];
	}

	//apply offsets
	GYRO_OFFSET_X = -(double)x / (double)cycles;
	GYRO_OFFSET_Y = -(double)y / (double)cycles;
	GYRO_OFFSET_Z = -(double)z / (double)cycles;

	//if offsets are too high, calibrate again
	if (fabs(GYRO_OFFSET_X) > 40 || fabs(GYRO_OFFSET_Y) > 40 || fabs(GYRO_OFFSET_Z) > 40) {
		IMU_calibrateGyro();
	}
}

void IMU_calibrateAccel() {
	// read accel for some time to stabilize readings
	for (int i = 0; i < 30000; i++) {
		int16_t ACC_data[6];
		IMU_waitForNewData();
		GYR_readBurst(0x22, ACC_data, 12);
	}

	// read accel and measure max and min
	float maxx = 0, minx = 0, maxy = 0, miny = 0, maxz = 0, minz = 0;
	float startTime = getSeconds();
	for (int i = 0; i < 6; i++) {
		__asm("bkpt");
		int64_t x = 0, y = 0, z = 0;
		for (int i = 0; i < 12000; i++) {
			int16_t ACC_data[6];
			IMU_waitForNewData();
			GYR_readBurst(0x22, ACC_data, 12);
			x += ACC_data[3];
			y += ACC_data[4];
			z += ACC_data[5];
		}
		if (x / 12000.0f > maxx)maxx = x / 12000.0f;
		if (x / 12000.0f < minx)minx = x / 12000.0f;
		if (y / 12000.0f > maxy)maxy = y / 12000.0f;
		if (y / 12000.0f < miny)miny = y / 12000.0f;
		if (z / 12000.0f > maxz)maxz = z / 12000.0f;
		if (z / 12000.0f < minz)minz = z / 12000.0f;
	}

	//apply offsets
	ACC_SCALE_X = (19.62f) / (maxx - minx);
	ACC_SCALE_Y = (19.62f) / (maxy - miny);
	ACC_SCALE_Z = (19.62f) / (maxz - minz);
	ACC_OFFSET_X = -(maxx + minx) / 2.0f;
	ACC_OFFSET_Y = -(maxy + miny) / 2.0f;
	ACC_OFFSET_Z = -(maxz + minz) / 2.0f;
	//if offsets are too high, calibrate again
	if (fabs(ACC_OFFSET_X) > 200 || fabs(ACC_OFFSET_Y) > 200 || fabs(ACC_OFFSET_Z) > 200) {
		IMU_calibrateAccel();
	}
}