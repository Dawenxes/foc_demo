#include "svpwm.h"

void clark(float iu,float iv,float * alpha,float * beta)
{
	(*alpha) = iu;
	(*beta) = (iu + iv * 2) * 0.577350269f;
}

void park(float sinval,float cosval,float alpha,float beta,float * id,float * iq)
{
	(*id) = alpha * cosval + beta * sinval;
	(*iq) = -alpha * sinval + beta *cosval;
}


void iclark(float alpha,float beta,float * u,float * v,float * w)
{
	(*u) = beta;
	(*v) = beta * 0.5f + alpha * 0.8660254f;
	(*w) = (*v) - (*u);
}

void ipark(float sinval,float cosval,float vd,float vq,float * alpha,float * beta)
{
	(*alpha) = vd * cosval - vq * sinval;
	(*beta) = vq * cosval + vd * sinval;
}

uint8_t cala_sector(float u,float v,float w)
{
	uint8_t sector = 3;
	if(v > 0)--sector;
	if(w > 0)--sector;
	if(u < 0)sector = 7 - sector;
	return sector;
}


void cala_tabc(float power,float u,float v,float w,uint8_t sector,float * ta,float * tb,float * tc)
{
	switch(sector)
	{
		case 1:
		case 4:(*ta) = v;(*tb) = u - w;(*tc) = -v;break;
		case 2:
		case 5:(*ta) = w + v;(*tb) = u;(*tc) = -u;break;
		case 3:
		case 6:(*ta) = w;(*tb) = -w;(*tc) = -(u + v);break;
		default:
			(*ta) = (*tb) = (*tc) = 0;
	}
	float km = power * 0.57735f;
	(*ta) = (*ta) / km;
	(*tb) = (*tb) / km;
	(*tc) = (*tc) / km;
}
