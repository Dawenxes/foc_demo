#include "pwm.h"

#include "main.h"

Pwm::Pwm(uint32_t freq,uint32_t half_period,std::function<void(uint32_t ccr1,uint32_t ccr2,uint32_t ccr3)> set_pwm_fp):
		set_pwm_fp(set_pwm_fp),
		half_period(half_period),
		freq(freq)
{

}

void Pwm::set_pwm(float u,float v,float w) const
{
	this->set_pwm_fp(half_period * u + half_period,half_period * v + half_period,half_period * w + half_period);
}

uint32_t Pwm::get_freq()const
{
	return freq;
}
