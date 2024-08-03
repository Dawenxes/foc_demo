#include "motor.h"

#include "svpwm.h"

#include <math.h>

#include "smosensor.h"

Motor::Motor(uint8_t pole,float vol,float max_curr,float max_speed,SensorBase * sensor,Pwm * pwm,Adc * adc):
	pole(pole),
	vol(vol),
	max_curr(max_curr),
	max_speed(max_speed),
	sensor(sensor),
	pwm(pwm),
	adc(adc)
{

}

void Motor::update()
{
	// 获得u,v相电流
	adc->get_uv(&this->iu,&this->iv);

	// 计算alpha,beta轴电流
	clark(this->iu, this->iv, &this->alpha, &this->beta);

	// 更新传感器电流（用作无感）
	sensor->input_curr(this->alpha,this->beta);
	
	// 从传感器获得角度
	sensor->get_angle(&this->theta,&this->angle_sin,&this->angle_cos);
	
	// 计算i轴和q轴电流
	park(this->angle_sin, this->angle_cos, this->alpha, this->beta, &this->id, &this->iq);

	this->speed = sensor->get_speed() / this->pole;

	this->pos = sensor->get_anglesum() / this->pole;
}


void Motor::set_vd_vq(float vd,float vq)
{
	// 计算alpha,beta轴电压
	float alpha_u,beta_u;
	ipark(angle_sin,angle_cos,vd,vq,&alpha_u,&beta_u);

	// 更新传感器电压（用作无感）
	sensor->input_volt(alpha_u,beta_u);
	
	// 计算三轴电压
	float u,v,w;
	iclark(alpha_u,beta_u,&u,&v,&w);

	// 计算当前扇区
	uint8_t sector = cala_sector(u,v,w);

	// 计算三相占空比
	float u_out,v_out,w_out;
	cala_tabc(this->vol,u,v,w,sector,&u_out,&v_out,&w_out);

	// 更新PWM模块
	pwm->set_pwm(u_out, v_out, w_out);

	// 计算(估算)母线电流
	this->ibus = fabsf(0.6667f * (vd * id + vq * iq) / vol);
}

int32_t Motor::get_speed() const
{
	return this->speed;
}

void Motor::get_curr(float * id,float *iq) const
{
	(*id) = this->id;
	(*iq) = this->iq;
}

float Motor::get_pos() const
{
	return this->pos;
}

float Motor::get_pwm_peroid()
{
	return 1.f / pwm->get_freq();
}

void Motor::set_pole(uint8_t pole)
{
	this->pole = pole;
}



