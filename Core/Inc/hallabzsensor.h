#ifndef __HALLABZSENSOR_H__
#define __HALLABZSENSOR_H__

#include <stdint.h>
#include <functional>

#include "timer.h"
#include "sensorbase.h"

class HallAbzSensor:public SensorBase
{
public:
	HallAbzSensor(Timer *timer,uint32_t count,std::function<uint8_t()> read_hall_fp,std::function<uint8_t()> read_abz_fp);
	void update_hall();
	void update_abz(char whitch);
	float get_anglesum() const;
	int32_t get_speed() const;
	void get_angle(float * theta,float * angle_sin,float * angle_cos);
	void set_offset(float offset);
private:
	Timer * timer = nullptr;
	std::function<uint8_t()> read_hall_fp = nullptr;
	std::function<uint8_t()> read_abz_fp = nullptr;
	uint8_t hall_state = 0;
	int8_t hall_dir = 1;
	int32_t cur_pos = 0;
	uint32_t last_hall_time = 0;
	uint32_t last_hall_cnt = 0;
	int32_t hall_speed = 0;
	int32_t abzcount = 0;
	uint32_t count = 2000;
	float offset = 0;
};


#endif
