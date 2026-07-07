#ifndef __PID_DRV_H
#define __PID_DRV_H

#include "motor_types.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Ref;
    float Fbk;
    float Out;
    float Err;
    float ErrLast;
    float AllowErr;
  float Integrate;
    float OutMax;
    float OutMin;
    float KpMax;
    float KpMin;
}PID_STRUCT;

void PID_Clear(PID_STRUCT *p);
void PID_Control(PID_STRUCT *p);

#endif
