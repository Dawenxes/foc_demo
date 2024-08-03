#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "motor.h"
#include "pid.h"

class Control
{
public:
	Control(Motor * motor);
	void update();

	void run();
	void stop();
	void set_mode(uint8_t);

	void set_speed(int32_t);
	void set_curr(float);
	void set_pos(float);
	void set_volt(float);

	void set_pos_kp(float);
	void set_pos_ki(float);
	void set_pos_kd(float);

	void set_speed_kp(float);
	void set_speed_ki(float);
	void set_speed_kd(float);

	void set_iq_kp(float);
	void set_iq_ki(float);
	void set_iq_kd(float);

	void set_id_kp(float);
	void set_id_ki(float);
	void set_id_kd(float);

	void set_max_id_iq(float);
	void set_max_speed(int32_t);
	void set_max_vd_vq(float);

	void set_pole(uint8_t);

	void set_speed_count(uint8_t count);

	void set_pos_count(uint8_t count);


private:
	Motor * motor = nullptr;
	uint8_t pos_count = 9;
	uint8_t curr_pos_count = 0;
	float pos_out = 0;
	uint8_t speed_count = 3;
	uint8_t curr_speed_count = 0;
	float speed_out = 0;
	float iq_out = 0;
	float id_out = 0;

	float ibus = 0;

	Pid pid_id;
	Pid pid_iq;
	Pid pid_pos;
	Pid pid_speed;

	uint8_t is_run = 0;
	uint8_t mode = 0; 

	float ref_pos = 60;
	int32_t ref_speed = 1500;
	float ref_curr = 0.5;
	float ref_volt = 3;


	float pos_kp = 85.0f;
	float pos_ki = 0.1f;
	float pos_kd = 0;


	float rs = 0.667f;
	float ls = 0.000316f;
//	float bandwith =  600 * 6.28f;
//	float st = 4;
//	float k = 1000000.f;
//	float rs = 0.42f;
//	float ls = 0.00112f;
	float bandwith =  600 * 6.28f;
	float st = 4;
	float k = 1500000.f;

	float speed_kp = bandwith / (st * k);
	float speed_ki = bandwith * bandwith /(st * st * st * k);
	float speed_kd = 0;

	float id_kp = ls * bandwith;
	float id_ki = rs * bandwith;
	float id_kd = 0;

	float iq_kp = ls * bandwith;
	float iq_ki = rs * bandwith;
	float iq_kd = 0;

	float max_id_iq = 10;
	int32_t max_speed = 8000;
	float max_vd_vq = 23;

};




#endif
