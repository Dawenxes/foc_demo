#ifndef __SMOSENSOR_H__
#define __SMOSENSOR_H__
#include "timer.h"
#include "sensorbase.h"
#include "sincos.h"

class SmoSensor:public SensorBase
{
public:
	SmoSensor(Timer *timer,std::function<uint8_t()> read_hall_fp);
	void update();
	float get_anglesum() const;
	int32_t get_speed() const;
	void get_angle(float * theta,float * angle_sin,float * angle_cos);
  void input_curr(float alpha,float beta)
	{
		curr_ia = alpha;
		curr_ib = beta;
	}
	void input_volt(float alpha,float beta)
	{
		curr_ua = alpha;
		curr_ub = beta;
	}
private:
	uint8_t hall_state = 0;
	int8_t hall_dir = 1;
	std::function<uint8_t()> read_hall_fp = nullptr;
	int32_t cur_pos = 0;
	Timer * timer = nullptr;
	uint32_t last_hall_time = 0;
	uint32_t last_hall_cnt = 0;
	int32_t hall_speed = 0;


	float curr_ia = 0;
	float curr_ib = 0;
	float curr_ua = 0;
	float curr_ub = 0;

	float IaErr = 0;
	float IbErr = 0;
	float Za = 0;
	float Zb = 0;
	float Ea = 0;
	float Eb = 0;
	float EstIa = 0;
	float EstIb = 0;

	float Ts = 1 / 12000.f;
//	float Ls = 0.00112f; // h
//	float Rs = 0.42f; // R
	float Ls = 0.000316f; // h
	float Rs = 0.667f; // R

  float Fsmopos = 1 - Ts * Rs / Ls;
  float Gsmopos = Ts / Ls;
	
	float E0 = 0.5f;
  float K = 0.045f;
};

#endif
