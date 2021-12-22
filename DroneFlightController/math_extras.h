#pragma once

typedef struct{
	float x;
	float y;
	float z;
} vec3;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} quat;

typedef struct {
	float yaw;
	float pitch;
	float roll;
} euler;

euler quatToEuler(quat);