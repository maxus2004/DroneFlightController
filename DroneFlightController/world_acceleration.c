#include "world_acceleration.h"

vec3 calculateWorldAcc(quat rotationQuat, vec3 acc) {
	float aw = 0, ax = acc.x, ay = acc.y, az = acc.z;
	float qw = rotationQuat.w, qx = rotationQuat.x, qy = rotationQuat.y, qz = rotationQuat.z;
	float qw1 = qw, qx1 = -qx, qy1 = -qy, qz1 = -qz;

	float bw = qw * aw - qx * ax - qy * ay - qz * az;
	float bx = qw * ax + qx * aw + qy * az - qz * ay;
	float by = qw * ay - qx * az + qy * aw + qz * ax;
	float bz = qw * az + qx * ay - qy * ax + qz * aw;

	float b1x = bw * qx1 + bx * qw1 + by * qz1 - bz * qy1;
	float b1y = bw * qy1 - bx * qz1 + by * qw1 + bz * qx1;
	float b1z = bw * qz1 + bx * qy1 - by * qx1 + bz * qw1;

	b1x *= 1.0033839f;
	b1y *= 1.0033839f;
	b1z *= 1.0033839f;
	b1z = b1z - 9.81f;

	vec3 worldAcc = { b1x ,b1y ,b1z };
	return worldAcc;
}