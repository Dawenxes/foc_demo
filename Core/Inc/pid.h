#ifndef __PID_H__
#define __PID_H__


class Pid
{
public:
	Pid();
	void set_pid(float kp,float ki,float id);
	void clear_pid_state();
	float cala_pid(float pid_err);
private:
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float state[3] = {0};
};


#endif
