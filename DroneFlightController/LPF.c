#include "LPF.h"
#include <string.h>
#include <stdlib.h>

void LPF_init(LPF* lpf, int length) {
	lpf->values = calloc(length, sizeof(float));
	lpf->length = length;
	lpf->value = 0;
	lpf->index = 0;
}

void LPF_update(LPF* lpf, float input) {
	lpf->value -= lpf->values[lpf->index] / lpf->length;
	lpf->value += input / lpf->length;
	lpf->values[lpf->index] = input;
	lpf->index++;
	if (lpf->index >= lpf->length)lpf->index = 0;
}

void LPF_clear(LPF* lpf) {
	lpf->value = 0;
	memset(lpf->values, 0, sizeof(float) * lpf->length);
}