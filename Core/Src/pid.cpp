#include "pid.h"

#include <string.h>

Pid::Pid()
{
}

void Pid::set_pid(float kp,float ki,float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void Pid::clear_pid_state()
{
	memset(state, 0, 3 * sizeof(float));
}

float Pid::cala_pid(float pid_err)
{
	this->state[2] += pid_err;
	float out =  (this->kp) * pid_err + (this->ki) * (this->state[2]) + (this->kd) * (pid_err - (this->state[0]));
	this->state[0] = pid_err;
	return out;
}
