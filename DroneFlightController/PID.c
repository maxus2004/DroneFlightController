#include "PID.h"
#include "LPF.h"

//PARAMETERS--------------------------

PID_params_s pid_params = {
	//   yaw    pitch   roll
		{0.015f, 0.015f, 0.015f }, //P
		{0.015f, 0.015f , 0.015f  }, //I
		{0.0f , 0.0015f, 0.0015f }, //D
		{0.4f , 0.5f  , 0.5f   }, //max
		{-0.4f, -0.5f , -0.5f  }, //min
		{0.0f , 0.1f  , 0.1f   }, //maxI
		{ 0.0f, -0.1f , -0.1f  }  //minI
};

//-----------------------------------

LPF rotationV_LPF[3] = { 0,0,0 };

PID_values_s pid_values = {
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 }
};

static PID_values_s* pid = &pid_values;

bool PID_doIntegration = false;

void PID_init(void) {
	for (int i = 0; i < 3; i++)
		LPF_init(&rotationV_LPF[i],10);
}

void PID_resetI(void)
{
	pid->i.yaw = 0;
	pid->i.roll = 0;
	pid->i.pitch = 0;
}

void PID_update(float dTime, euler rotation, euler rotationV, euler target, euler* torque)
{
	//error
	pid->E.yaw = target.yaw - rotation.yaw;
	pid->E.pitch = target.pitch - rotation.pitch;
	pid->E.roll = target.roll - rotation.roll;
	if (pid->E.yaw > 180)
	{
		pid->E.yaw -= 360;
	}
	else if (pid->E.yaw < -180)
	{
		pid->E.yaw += 360;
	}

	//proportional
	pid->p.roll = pid_params.KP.roll * pid->E.roll;
	pid->p.pitch = pid_params.KP.pitch * pid->E.pitch;
	pid->p.yaw = pid_params.KP.yaw * pid->E.yaw;

	//integral
	if (PID_doIntegration)
	{
		pid->i.roll = pid->i.roll + pid_params.KI.roll * dTime * (pid->E.roll);
		if (pid->i.roll > pid_params.maxI.roll) pid->i.roll = pid_params.maxI.roll;
		if (pid->i.roll < pid_params.minI.roll) pid->i.roll = pid_params.minI.roll;
		pid->i.pitch = pid->i.pitch + pid_params.KI.pitch * dTime * (pid->E.pitch);
		if (pid->i.pitch > pid_params.maxI.pitch) pid->i.pitch = pid_params.maxI.pitch;
		if (pid->i.pitch < pid_params.minI.pitch) pid->i.pitch = pid_params.minI.pitch;
		pid->i.yaw = pid->i.yaw + pid_params.KI.yaw * dTime * (pid->E.yaw);
		if (pid->i.yaw > pid_params.maxI.yaw) pid->i.yaw = pid_params.maxI.yaw;
		if (pid->i.yaw < pid_params.minI.yaw) pid->i.yaw = pid_params.minI.yaw;
	}

	//derivative
	LPF_update(&rotationV_LPF[0], rotationV.roll);
	LPF_update(&rotationV_LPF[1], rotationV.pitch);
	LPF_update(&rotationV_LPF[2], rotationV.yaw);
	pid->d.roll = -pid_params.KD.roll * rotationV_LPF[0].value;
	pid->d.pitch = -pid_params.KD.pitch * rotationV_LPF[1].value;
	pid->d.yaw = -pid_params.KD.yaw * rotationV_LPF[2].value;


	//sum pid
	torque->roll = pid->p.roll + pid->i.roll + pid->d.roll;
	if (torque->roll > pid_params.max.roll)torque->roll = pid_params.max.roll;
	if (torque->roll < pid_params.min.roll)torque->roll = pid_params.min.roll;
	torque->pitch = pid->p.pitch + pid->i.pitch + pid->d.pitch;
	if (torque->pitch > pid_params.max.pitch)torque->pitch = pid_params.max.pitch;
	if (torque->pitch < pid_params.min.pitch)torque->pitch = pid_params.min.pitch;
	torque->yaw = pid->p.yaw + pid->i.yaw + pid->d.yaw;
	if (torque->yaw > pid_params.max.yaw)torque->yaw = pid_params.max.yaw;
	if (torque->yaw < pid_params.min.yaw)torque->yaw = pid_params.min.yaw;

	//prev values
	pid->prevE.yaw = pid->E.yaw;
	pid->prevE.pitch = pid->E.pitch;
	pid->prevE.roll = pid->E.roll;
	pid->prev.yaw = rotation.yaw;
	pid->prev.pitch = rotation.pitch;
	pid->prev.roll = rotation.roll;
}
