#ifndef __SPEED_DRV_H
#define __SPEED_DRV_H

#include "motor_types.h"

typedef enum{
    ACCELERATE = 0,
    UNIFORM,
    DECELERATE
}MOTION_STATE;

typedef struct
{
    u8    PolePairs;
    s16   CurrentSpeedDir;
    u16   SpeedCalculateCnt;
    u16   SpeedDivisionFactor;
    u16   ElectricalValMax;
    float   ElectricalPosThis;
    float   ElectricalPosLast;
    float   ElectricalPosChange;
    float ElectricalSpeedFactor;
    float ElectricalSpeedRaw;

    float ElectricalSpeedLPF;
    float ElectricalSpeedLPFFactor;

    float MechanicalSpeed;
  float MechanicalSpeedSet;
  float MechanicalSpeedSetLast;
}SPEED_STRUCT;

typedef struct
{
    s8    SpeedOutDir;
    u8    PolePairs;
    float Ts;
    float TargetSpeed;
    float AccSpeed;

    float SpeedTargetIncrement;
    float SpeedIncrement;
    float SpeedChangeIncrement;
    float SpeedOut;

    float SpeedIncrementDelta;
    float SpeedIncrementLast;
    MOTION_STATE MotionState;
    u8    FinishFlag;
}TSHAPEDACCDEC_STRUCT;

void Calculate_Speed(SPEED_STRUCT *p);
void T_Shaped_Acc_Dec(TSHAPEDACCDEC_STRUCT *p);

#endif
