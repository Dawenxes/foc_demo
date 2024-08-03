#include "hallsensor.h"
#include "sincos.h"
#include "timer.h"
#include "sincos.h"

#include "platform.h"

#include <stdint.h>
#include <math.h>

static signed char const ELECTRIC_SECTORS[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 }; 

HallSensor::HallSensor(Timer * timer,std::function<uint8_t()> read_hall_fp):
		read_hall_fp(read_hall_fp),
		timer(timer)

{

}

void HallSensor::update()
{
	const uint8_t new_state = this->read_hall_fp();
	const int8_t err = ELECTRIC_SECTORS[new_state] - ELECTRIC_SECTORS[hall_state];
	const int8_t last_dir = hall_dir;
	uint8_t is_hall_err = 0;
	if(hall_state == 0) {

	}
	else if(err == -1 || err == 5)
	{
		hall_dir = -1;
		--cur_pos;
	}
	else if(err == 1 || err == -5)
	{
		hall_dir = 1;
		++cur_pos;
	}
	else {
		hall_state = 0;
		is_hall_err = 1;
	}
	if(!is_hall_err)
	{
		int32_t sp;
		if(hall_state == 0 || last_dir != hall_dir)
		{
			sp = 0;
		}
		else
		{
			uint64_t tm = ((uint64_t)(timer->get_first() - last_hall_time) * 0xFFFFFFFF) + ((int64_t)timer->get_second() - last_hall_cnt);
			const float tm_sf =  tm * (1.f / timer->get_freq());
			sp = 10.f / tm_sf;
		}
		sp *= hall_dir;
		hall_speed = sp + 0.5f;
		last_hall_time = timer->get_first();
		last_hall_cnt = timer->get_second();
		hall_state = new_state;
	}
}

float HallSensor::get_anglesum() const
{
	return cur_pos / 6;
}

int32_t HallSensor::get_speed() const
{
	float ret;
	uint64_t tm = ((uint64_t)(timer->get_first() - last_hall_time) * 0xFFFFFFFF) + ((int64_t)timer->get_second() - last_hall_cnt);
	float tm_sf =  tm * (1.f / timer->get_freq());
	if(tm_sf > 0.2)
	{
		ret = 0;
	}else
	{
		ret = hall_speed;
	}
	return ret;
}

void HallSensor::get_angle(float * theta,float * angle_sin,float * angle_cos)
{
	if(!hall_state)
	{
		hall_state = this->read_hall_fp();
	}
	float angle = ELECTRIC_SECTORS[hall_state] * 60.f;
	angle += offset;

	float hall_speed = this->hall_speed;

	uint64_t tm = ((uint64_t)(timer->get_first() - last_hall_time) * 0xFFFFFFFF) + ((int64_t)timer->get_second() - last_hall_cnt);
	float add_angle = tm * (1.f / timer->get_freq()) * hall_speed * 6 - 30 * hall_dir;
	if(fabsf(add_angle) < 60)
	{
		angle += add_angle;
	}
	else if(add_angle > 0)
	{
		angle += 30;
	}
	else {
		angle -= 30;
	}
	if(angle > 360)angle -= 360;else if(angle < 0)angle += 360;
	if(theta)
	{
		(*theta) = angle;
	}
	mysincos(angle,angle_sin,angle_cos);
}

void HallSensor::set_offset(float offset)
{
	this->offset = offset;
}


