#ifndef __ADC_H__
#define __ADC_H__

#include <functional>

class Adc
{
public:
	enum class Mode {
		UV,
		UW,
		VW,
		UVW
	};
	Adc(Mode mode,std::function<void(bool is_cai,float * a1,float * a2,float * a3)> get_uvw_fp);
	void cai_adc();
	void get_uv(float * u,float * v) const;
private:
	float cai_1 = 0;
	float cai_2 = 0;
	float cai_3 = 0;
	std::function<void(bool is_cai,float * a1,float * a2,float * a3)> get_uvw_fp = nullptr;
	Mode mode = Mode::UV;
};

#endif
