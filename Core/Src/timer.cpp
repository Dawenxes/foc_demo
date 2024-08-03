#include "timer.h"


Timer::Timer(uint32_t freq,std::function<uint32_t()> get_second_fp):
	freq(freq),
	get_second_fp(get_second_fp)
{

}

uint32_t Timer::get_first()const
{
	return first;
}

uint32_t Timer::get_second()const
{
	return get_second_fp();
}

uint32_t Timer::get_freq()const
{
	return freq;
}

void Timer::update()
{
	++first;
}
