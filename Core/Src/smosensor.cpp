#include "smosensor.h"

#include "sincos.h"
#include "timer.h"
#include "sincos.h"

#include "platform.h"

#include <stdint.h>
#include <math.h>

static signed char const ELECTRIC_SECTORS[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 }; 

SmoSensor::SmoSensor(Timer * timer,std::function<uint8_t()> read_hall_fp):
		read_hall_fp(read_hall_fp),
		timer(timer)

{

}

void SmoSensor::update()
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
		DISABLE_IRQ;
		hall_speed = sp;
		ENABLE_IRQ;
		last_hall_time = timer->get_first();
		last_hall_cnt = timer->get_second();
		hall_state = new_state;
	}
}

float SmoSensor::get_anglesum() const
{
	return cur_pos / 6;
}

int32_t SmoSensor::get_speed() const
{
	float ret;
	uint64_t tm = ((uint64_t)(timer->get_first() - last_hall_time) * 0xFFFFFFFF) + ((int64_t)timer->get_second() - last_hall_cnt);
	float tm_sf =  tm * (1.f / timer->get_freq());
	if(tm_sf > 0.2)
	{
		ret = 0;
	}else
	{
		DISABLE_IRQ;
		ret = hall_speed;
		ENABLE_IRQ;
	}
	return ret;
}

void SmoSensor::get_angle(float * theta,float * angle_sin,float * angle_cos)
{
	if(!hall_state)
	{
		hall_state = this->read_hall_fp();
	}
	
	float smo_angle;
	{
		EstIa = Fsmopos * EstIa + Gsmopos * (curr_ua - Ea - Za);
		EstIb = Fsmopos * EstIb + Gsmopos * (curr_ub - Eb - Zb);

		IaErr = EstIa - curr_ia;
		IbErr = EstIb - curr_ib;
		if (fabsf(IaErr) < E0)
		{
				Za = K * (IaErr / E0);
		}
		else if (IaErr > E0)
		{
				Za = K;
		}
		else if (IaErr <= -E0)
		{
				Za = (-K);
		}

		if (fabsf(IbErr) < E0)
		{
				Zb = K * (IbErr / E0);
		}
		else if (IbErr > E0)
		{
				Zb = K;
		}
		else if (IbErr <= -E0)
		{
				Zb = (-K);
		}
		Ea = Ea + K * (Za - Ea);
    Eb = Eb + K * (Zb - Eb);
		
		const uint8_t dir = 0;
		
		float angle;
		angle = -atan2(Ea,Eb);
    if(angle > (-3.14159f/2)){
        angle += (3.14159f/2);
    }else{
        angle += (3.14159f/2.f*5.f);
    }
		angle = angle * 180 / 3.14159f;
		mysincos(angle,angle_sin,angle_cos);
		smo_angle = angle;
	}
	
	if(abs(hall_speed) < 2000)
	{
		float angle = ELECTRIC_SECTORS[hall_state] * 60.f;
		if(theta)
		{
			(*theta) = angle;
		}
		mysincos(angle,angle_sin,angle_cos);
	}
	else
	{
		(*theta) = smo_angle;
		mysincos(smo_angle,angle_sin,angle_cos);
		
	}

}


