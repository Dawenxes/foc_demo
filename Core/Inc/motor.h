#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

#include "sensorbase.h"
#include "pwm.h"
#include "adc.h"

class Motor
{
public:
	Motor(uint8_t pole,float vol,float max_curr,float max_speed,SensorBase * sensor,Pwm * pwm,Adc * adc);
	void update();
	void set_vd_vq(float vd,float vq);
	int32_t get_speed() const;
	void get_curr(float * id,float *iq) const;
	float get_pos() const;
	float get_pwm_peroid();
	void set_pole(uint8_t pole);

private:
	uint8_t pole = 2;
	float vol = 24;
	float max_curr = 10;
	int32_t max_speed = 8000;
	SensorBase * sensor = nullptr;
	Pwm * pwm = nullptr;
	Adc * adc = nullptr;
	float iu = 0;
	float iv = 0;
	float theta = 0;
	float angle_sin = 0;
	float angle_cos= 0;
	bool canrun = 1;
	float alpha = 0;
	float beta = 0;
	float id = 0;
	float iq = 0;
	int32_t speed = 0;
	float pos = 0;
	float ibus = 0;
};

#endif
