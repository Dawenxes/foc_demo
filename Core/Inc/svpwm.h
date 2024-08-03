#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void clark(float iu,float iv,float * alpha,float * beta);

void park(float sinval,float cosval,float alpha,float beta,float * id,float * iq);

void iclark(float alpha,float beta,float * u,float * v,float * w);

void ipark(float sinval,float cosval,float vd,float vq,float * alpha,float * beta);

uint8_t cala_sector(float u,float v,float w);

void cala_tabc(float power,float u,float v,float w,uint8_t sector,float * ta,float * tb,float * tc);

#ifdef __cplusplus
}
#endif

#endif
