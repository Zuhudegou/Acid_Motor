#ifndef __MATH_DRV_H
#define __MATH_DRV_H

#include "motor_types.h"

void Calculate_Sin_Cos(float angle, float *sinval, float *cosval);
void Amplitude_Limit(float *input, float min, float max);

#endif
