#include "control.h"
#include "platform.h"
#include <math.h>

Control::Control(Motor * motor):
	motor(motor)
{

}


static float limit_pid_value(Pid * pid,float x,float l,float h)
{
	if(x < l)
	{
		pid->clear_pid_state();
		return l;
	}
	if(x > h)
	{
		pid->clear_pid_state();
		return h;
	}
	return x;
}


void Control::update()
{
	// 更新电机状态
	motor->update();

	// 获得电机机械速度
	int32_t motor_speed = motor->get_speed();
	
	// 限制最大速度
	if(motor_speed > this->max_speed)
	{
		motor_speed = max_speed;
	}
	else if(motor_speed < -max_speed)
	{
		motor_speed = -max_speed;
	}

	// 获得电机D、Q轴电流
	float id,iq;
	motor->get_curr(&id, &iq);

	// 获得电机机械位置
	float pos = motor->get_pos();

	// 获得PWM周期
	float pwm_period = motor->get_pwm_peroid();

	// 模式分析
	uint8_t can_mode_pos = mode >> 2;
	uint8_t can_mode_speed = (mode >> 1) & 1;
	uint8_t can_mode_curr = mode & 1;

	// ID电流环
	float vd_out;
	{
		pid_id.set_pid(id_kp, id_ki * pwm_period,id_kd * pwm_period);
		float pid_err = 0 - id;
		id_out = pid_id.cala_pid(pid_err);
		vd_out = limit_pid_value(&pid_id,id_out,-max_vd_vq,max_vd_vq);
	}

	// 位置环
	++curr_pos_count;
	if(curr_pos_count >= pos_count)
	{
		curr_speed_count = 0;
		float tm = curr_pos_count * pwm_period;
		pid_pos.set_pid(pos_kp, pos_ki * tm, pos_kd * tm);
		DISABLE_IRQ;
		float pid_err = ref_pos - pos;
		ENABLE_IRQ;
		pos_out = pid_pos.cala_pid(pid_err);
	}

	// 转速环
	++curr_speed_count;
	if(curr_speed_count >= speed_count)
	{
		curr_speed_count = 0;
		float tm = speed_count * pwm_period;
		pid_speed.set_pid(speed_kp, speed_ki * tm, speed_kd * tm);
		int32_t pid_err;
		if(can_mode_pos)
		{
			int32_t pos_out_ref = limit_pid_value(&pid_pos,pos_out,-max_speed,max_speed);
			pid_err = pos_out_ref - motor_speed;
		}
		else
		{
			pid_err = ref_speed - motor_speed;
		}

		speed_out = pid_speed.cala_pid(pid_err);
	}

	// IQ电流环
	{
		pid_iq.set_pid(iq_kp, iq_ki  * pwm_period, iq_kd  * pwm_period);

		float pid_err;
		if(can_mode_speed)
		{
			float speed_out_ref = limit_pid_value(&pid_speed,speed_out,-max_id_iq,max_id_iq);
			pid_err = speed_out_ref - iq;
		}
		else if(can_mode_pos)
		{
			float pos_out_ref = limit_pid_value(&pid_pos,pos_out,-max_id_iq,max_id_iq);
			pid_err = pos_out_ref - iq;
		}
		else
		{
			DISABLE_IRQ;
			pid_err = ref_curr - iq;
			ENABLE_IRQ;
			
		}

		iq_out = pid_iq.cala_pid(pid_err);
	}

	// 输出电压控制
	float vq_out;
	if(can_mode_curr)
	{
		vq_out = limit_pid_value(&pid_iq,iq_out,-max_vd_vq,max_vd_vq);
	}
	else if(can_mode_speed)
	{
		vq_out = limit_pid_value(&pid_speed,speed_out,-max_vd_vq,max_vd_vq);
	}
	else if(can_mode_pos)
	{
		vq_out = limit_pid_value(&pid_pos,pos_out,-max_vd_vq,max_vd_vq);
	}
	else
	{
		vq_out = ref_volt;
		vd_out = 0; //  一环都没开，则完全开环运转
	}
	if(is_run)
	{
		motor->set_vd_vq(vd_out, vq_out);
	}
	else
	{
		motor->set_vd_vq(0, 0);
		pid_id.clear_pid_state();
		pid_iq.clear_pid_state();
		pid_pos.clear_pid_state();
		pid_speed.clear_pid_state();
	}
}


void Control::run()
{
	is_run = true;
}
void Control::stop()
{
	is_run = false;
}
void Control::set_mode(uint8_t mode)
{
	this->mode = mode;
}

void Control::set_speed(int32_t ref_speed)
{
	this->ref_speed = ref_speed;
}
void Control::set_curr(float ref_curr)
{
	DISABLE_IRQ;
	this->ref_curr = ref_curr;
	ENABLE_IRQ;
}
void Control::set_pos(float ref_pos)
{
	DISABLE_IRQ;
	this->ref_pos = ref_pos;
	ENABLE_IRQ;
}
void Control::set_volt(float ref_volt)
{
	this->ref_volt = ref_volt;
}

void Control::set_pos_kp(float kp)
{
	this->pos_kp = kp;
}
void Control::set_pos_ki(float ki)
{
	this->pos_ki = ki;
}
void Control::set_pos_kd(float kd)
{
	this->pos_kd = kd;
}

void Control::set_speed_kp(float kp)
{
	this->speed_kp = kp;
}
void Control::set_speed_ki(float ki)
{
	this->speed_ki = ki;
}
void Control::set_speed_kd(float kd)
{
	this->speed_kd = kd;
}

void Control::set_iq_kp(float kp)
{
	this->iq_kp = kp;
}
void Control::set_iq_ki(float ki)
{
	this->iq_ki = ki;
}
void Control::set_iq_kd(float kd)
{
	this->iq_kd = kd;
}

void Control::set_id_kp(float kp)
{
	this->id_kp = kp;
}
void Control::set_id_ki(float ki)
{
	this->id_ki = ki;
}
void Control::set_id_kd(float kd)
{
	this->id_kd = kd;
}

void Control::set_max_id_iq(float max_id_iq)
{
	this->max_id_iq = max_id_iq;
}
void Control::set_max_speed(int32_t max_speed)
{
	this->max_speed = max_speed;
}
void Control::set_max_vd_vq(float max_vd_vq)
{
	this->max_vd_vq = max_vd_vq;
}

void Control::set_pole(uint8_t pole)
{
	this->motor->set_pole(pole);
}

void Control::set_speed_count(uint8_t count)
{
	this->speed_count = count;
}
void Control::set_pos_count(uint8_t count)
{
	this->pos_count = count;
}
