#ifndef __SENSORBASE_H__
#define __SENSORBASE_H__

#include <stdint.h>

class SensorBase
{
public:
	// 获取电机旋转的电气圈数
	virtual float get_anglesum() const = 0;
  // 获得电机的电气转速，r/min
	virtual int32_t get_speed() const = 0;
	// 获得电机的电气角度
	virtual void get_angle(float * theta,float * angle_sin,float * angle_cos) = 0;

	// 滑膜估算器需要调用这两个函数
	virtual void input_curr(float alpha,float beta){};
	virtual void input_volt(float alpha,float beta){};
};


#endif
