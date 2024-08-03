#include "../Inc/adc.h"
#include "main.h"


#include <stdint.h>

Adc::Adc(Mode mode,std::function<void(bool is_cai,float * a1,float * a2,float * a3)> get_uvw_fp):
		get_uvw_fp(get_uvw_fp),
		mode(mode)
{

}

void Adc::get_uv(float * u,float * v) const
{
	float a1,a2,a3;
	this->get_uvw_fp(false,&a1,&a2,&a3);
	a1 -= cai_1;
	a2 -= cai_2;
	if(mode == Mode::UV)
	{
		(*u) = a1;
		(*v) = a2;
	}
	else if(mode == Mode::UW)
	{
		(*u) = a1;
		(*v) = -a1 - a2;
	}
	else if(mode == Mode::VW)
	{
		(*u) = -a1 - a2;
		(*v) = a1;
	}
	else if(mode == Mode::UVW)
	{
		(*u) = a1;
		(*v) = a2;
	}
}

void Adc::cai_adc()
{
	float a1,a2,a3;
	float aa1 = 0.f,aa2 = 0.f;
	uint16_t tms = 1000;
	for(uint16_t i = 0;i < tms;++i)
	{
		this->get_uvw_fp(true,&a1,&a2,&a3);
		aa1 += a1;
		aa2 += a2;
	}
	aa1 /= tms;
	aa2 /= tms;
	cai_1 = aa1;
	cai_2 = aa2;
}
