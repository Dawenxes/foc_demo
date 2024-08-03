#ifndef __PWM_H__
#define __PWM_H__


#include <functional>
#include <stdint.h>
class Pwm
{
public:
	Pwm(uint32_t freq,uint32_t half_period,std::function<void(uint32_t ccr1,uint32_t ccr2,uint32_t ccr3)> set_pwm_fp);
	void set_pwm(float u,float v,float w) const;
	uint32_t get_freq()const;
private:
	std::function<void(uint32_t ccr1,uint32_t ccr2,uint32_t ccr3)> set_pwm_fp = nullptr;
	uint32_t half_period = 1750;
	uint32_t freq = 12000;
};

#endif
