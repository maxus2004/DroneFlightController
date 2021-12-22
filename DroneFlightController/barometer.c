#include "barometer.h"
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_bus.h>
#include <math.h>
#include <stdbool.h>

#include "time.h"


float BAR_ALT_OFFSET = 0;

#define cs1 GPIOB->BSRR = 1<<1
#define cs0 GPIOB->BSRR = 1<<(16+1)

static uint8_t SPI_SendReceive(uint8_t data) {
	while (SPI1->SR & SPI_SR_BSY);
	SPI1->DR = data;
	while (!(SPI1->SR & SPI_SR_TXE)) {}
	while (!(SPI1->SR & SPI_SR_RXNE)) {}
	return SPI1->DR;
}

struct CalibrationData {
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float par_t1;
	float par_t2;
	float par_t3;
	float t_lin;
};

struct CalibrationData calibration_data = { 0 };

static void readCalibrationData() {
	int8_t data[21];
	cs0;;
	SPI_SendReceive(0x31 | 0x80);
	SPI_SendReceive(0xff);
	for (int i = 0; i < 21; i++) {
		data[i] = SPI_SendReceive(0xff);
	}
	cs1;

	calibration_data.par_t1 = (float)((uint8_t)data[0] | (uint16_t)data[1] << 8) / 0.00390625f;
	calibration_data.par_t2 = (float)((uint8_t)data[2] | (uint16_t)data[3] << 8) / 1073741824.0f;
	calibration_data.par_t3 = (float)(data[4]) / 281474976710656.0f;
	calibration_data.par_p1 = (float)(((int16_t)data[5] | (int16_t)data[6] << 8) - 16384) / 1048576.0f;
	calibration_data.par_p2 = (float)(((int16_t)data[7] | (int16_t)data[8] << 8) - 16384) / 536870912.0f;
	calibration_data.par_p3 = (float)(data[9]) / 4294967296.0f;
	calibration_data.par_p4 = (float)(data[10]) / 137438953472.0f;
	calibration_data.par_p5 = (float)((uint8_t)data[11] | (uint16_t)data[12] << 8) / 0.125f;
	calibration_data.par_p6 = (float)((uint8_t)data[13] | (uint16_t)data[14] << 8) / 64.0f;
	calibration_data.par_p7 = (float)(data[15]) / 256.0f;
	calibration_data.par_p8 = (float)(data[16]) / 32768.0f;
	calibration_data.par_p9 = (float)((int16_t)data[17] | (int16_t)data[18] << 8) / 281474976710656.0f;
	calibration_data.par_p10 = (float)(data[19]) / 281474976710656.0f;
	calibration_data.par_p11 = (float)(data[20]) / 36893488147419103232.0f;

}
static uint8_t readReg(uint8_t reg) {
	cs0;;
	SPI_SendReceive(0x80 | reg);
	SPI_SendReceive(0xff);
	uint8_t data = SPI_SendReceive(0xff);
	cs1;
	return data;
}
static void writeReg(uint8_t reg, uint8_t data) {
	cs0;;
	SPI_SendReceive(reg);
	SPI_SendReceive(data);
	cs1;
}

void barometer_init() {
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	cs1;
	delayMillis(10);

	writeReg(0x1a, 0b00000000);             // SPI 4 wire, watchdog disabled
	writeReg(0x1b, 0b00110011);             // power on, pressure on, temperature on
	writeReg(0x1c, 0b00000011);             // pressure oversampling x8, temperature x1
	writeReg(0x1d, 0x02);                   // 50 Hz
	writeReg(0x1f, 0b00000100);             // filter coefficient 3
	readCalibrationData();
}

uint32_t barometer_press_raw(void) {
	uint32_t data = 0;
	cs0;
	SPI_SendReceive(0x04 | 0x80);
	SPI_SendReceive(0xff);
	data |= SPI_SendReceive(0xff);
	data |= (uint32_t)SPI_SendReceive(0xff) << 8;
	data |= (uint32_t)SPI_SendReceive(0xff) << 16;
	cs1;
	return data;
}
uint32_t barometer_temp_raw(void) {
	uint32_t data = 0;
	cs0;
	SPI_SendReceive(0x07 | 0x80);
	SPI_SendReceive(0xff);
	data |= SPI_SendReceive(0xff);
	data |= (uint32_t)SPI_SendReceive(0xff) << 8;
	data |= (uint32_t)SPI_SendReceive(0xff) << 16;
	cs1;
	return data;
}
void barometer_readAllRaw(uint32_t* press_raw, uint32_t* temp_raw) {
	cs0;
	SPI_SendReceive(0x04 | 0x80);
	SPI_SendReceive(0xff);
	*press_raw |= SPI_SendReceive(0xff);
	*press_raw |= (uint32_t)SPI_SendReceive(0xff) << 8;
	*press_raw |= (uint32_t)SPI_SendReceive(0xff) << 16;
	*temp_raw |= SPI_SendReceive(0xff);
	*temp_raw |= (uint32_t)SPI_SendReceive(0xff) << 8;
	*temp_raw |= (uint32_t)SPI_SendReceive(0xff) << 16;
	cs1;
}

float barometer_temp2(uint32_t uncomp_temp) {
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp - calibration_data.par_t1);
	partial_data2 = (float)(partial_data1 * calibration_data.par_t2);
	/* Update the compensated temperature in calib structure since this is
	* needed for pressure calculation */
	calibration_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calibration_data.par_t3;
	//calibration_data.t_lin = 1;
	/* Returns compensated temperature */


	return calibration_data.t_lin;
}

float barometer_temp(void) {
	return barometer_temp2(barometer_temp_raw());
}

float barometer_press2(uint32_t uncomp_press) {
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	/* Calibration data */
	partial_data1 = calibration_data.par_p6 * calibration_data.t_lin;
	partial_data2 = calibration_data.par_p7 * (calibration_data.t_lin * calibration_data.t_lin);
	partial_data3 = calibration_data.par_p8 * (calibration_data.t_lin * calibration_data.t_lin * calibration_data.t_lin);
	partial_out1 = calibration_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calibration_data.par_p2 * calibration_data.t_lin;
	partial_data2 = calibration_data.par_p3 * (calibration_data.t_lin * calibration_data.t_lin);
	partial_data3 = calibration_data.par_p4 * (calibration_data.t_lin * calibration_data.t_lin * calibration_data.t_lin);
	partial_out2 = (float)uncomp_press * (calibration_data.par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calibration_data.par_p9 + calibration_data.par_p10 * calibration_data.t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calibration_data.par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;
	return comp_press;
}

float barometer_press(void) {
	return barometer_press2(barometer_press_raw());
}

float barometer_alt(float pressure, float temperature) {
	float alt = (1.0f - powf(pressure / 101325.0f, 0.190284f)) * (temperature + 273.15f) / 0.0065f;
	//float alt = (101325.0f-pressure) / 12.69f;
	return alt;
}

bool barometer_available() {
	return (readReg(0x03) & (1 << 5)) != 0;
}

void barometer_readAll(float* press, float* alt, float* temp) {
	uint32_t press_raw = 0, temp_raw = 0;
	barometer_readAllRaw(&press_raw, &temp_raw);
	*temp = barometer_temp2(temp_raw);
	*press = barometer_press2(press_raw);
	*alt = barometer_alt(*press, *temp) + BAR_ALT_OFFSET;
}

void barometer_zeroOffset() {
	BAR_ALT_OFFSET = -barometer_alt(barometer_press(), barometer_temp());
}