#ifndef __FOC_DRV_H
#define __FOC_DRV_H

#include "motor_types.h"

#define FOC_INV_SQRT3    0.57735027f
#define FOC_2_INV_SQRT3  1.1547004f
#define FOC_SQRT3_DIV2   0.866f
#define FOC_SQRT3        1.732f
#define FOC_3_DIV2       1.5f
#define FOC_HALF         0.5f
#define FOC_QUARTER      0.25f

typedef struct
{
    float Iu;
    float Iv;
    float Iw;
    float Ialpha;
    float Ibeta;

    float SinVal;
    float CosVal;
    float Id;
    float Iq;

  float IdLPF;
  float IqLPF;
  float IdLPFFactor;
  float IqLPFFactor;

    float Ud;
    float Uq;
    float Ualpha;
    float Ubeta;
    float Ubus;

  u16   PwmCycle;
  u16   PwmLimit;
    u16   DutyCycleA;
    u16   DutyCycleB;
    u16   DutyCycleC;
}FOC_STRUCT;

void Clark_Transform(FOC_STRUCT *p);
void Park_Transform(FOC_STRUCT *p);
void IPark_Transform(FOC_STRUCT *p);
void Calculate_SVPWM(FOC_STRUCT *p);

#endif
