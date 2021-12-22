#pragma once

typedef struct {
	int length;
	float* values;
	float value;
	int index;
} LPF;

void LPF_init(LPF* lpf, int length);
void LPF_update(LPF* lpf, float newValue);
void LPF_clear(LPF* lpf);