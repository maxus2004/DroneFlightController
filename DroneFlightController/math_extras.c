#include <math.h>
#include "math_extras.h"

euler quatToEuler(quat q) {
	euler e;
	e.roll = atan2f(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 57.29f;
	e.pitch = asinf(2 * (q.w * q.y - q.z * q.x)) * 57.29f;
	e.yaw = atan2f(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 57.29f;
	return e;
}