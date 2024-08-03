#ifndef __HALL_H__
#define __HALL_H__

#include <stdint.h>
#include <functional>

#include "timer.h"
#include "sensorbase.h"

class HallSensor:public SensorBase
{
public:
	HallSensor(Timer *timer,std::function<uint8_t()> read_hall_fp);
	void update();
	float get_anglesum() const;
	int32_t get_speed() const;
	void get_angle(float * theta,float * angle_sin,float * angle_cos);
	void set_offset(float offset);
private:
	uint8_t hall_state = 0;
	int8_t hall_dir = 1;
	std::function<uint8_t()> read_hall_fp = nullptr;
	int32_t cur_pos = 0;
	Timer * timer = nullptr;
	uint32_t last_hall_time = 0;
	uint32_t last_hall_cnt = 0;
	int32_t hall_speed = 0;
	float offset = 0;
};

#endif
