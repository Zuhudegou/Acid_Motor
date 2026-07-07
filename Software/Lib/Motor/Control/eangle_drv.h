#ifndef __EANGLE_DRV_H
#define __EANGLE_DRV_H

#include "motor_types.h"

#define EANGLE_RPM_TO_RPS   0.01666f

typedef struct
{
    u8    Dir;
    u8    PolePairs;
    s32   EncoderVal;
    s32   EncoderValMax;
    s32   EncoderValChange;
    u16   CalibFlag;
    s32   CalibOffset;
    float Ts;
    float ElectricalAnglePU;
    float ElectricalAngleSpdSet;
    float ElectricalAngleSetPU;
}E_ANGLE_STRUCT;

void Electrical_Angle_Generator(E_ANGLE_STRUCT *p);
void Calculate_Encoder_Data(E_ANGLE_STRUCT *p);

#endif
