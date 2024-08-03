#ifndef __TIMER_H_
#define __TIMER_H_


#include <stdint.h>
#include <functional>


class Timer
{
public:
	Timer(uint32_t freq,std::function<uint32_t()> get_second_fp);
	uint32_t get_first()const;
	uint32_t get_second()const;
	uint32_t get_freq()const;
	void update();
private:
	uint32_t first = 0;
	uint32_t freq = 84000000;
	std::function<uint32_t()> get_second_fp = nullptr;
};




#endif
