#include <math.h>

#include "speed_drv.h"

void Calculate_Speed(SPEED_STRUCT *p)
{
    const float pole_pairs = (p->PolePairs > 0u) ? (float)p->PolePairs : 1.0f;

    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;

    p->ElectricalPosLast = p->ElectricalPosThis;

    if(p->ElectricalPosChange >= 0.5f)
    {
        p->ElectricalPosChange = p->ElectricalPosChange - 1.0f;
    }

    if(p->ElectricalPosChange <= -0.5f)
    {
        p->ElectricalPosChange = p->ElectricalPosChange + 1.0f;
    }

    p->ElectricalSpeedRaw = p->ElectricalPosChange * p->ElectricalSpeedFactor;

    p->ElectricalSpeedLPF = p->ElectricalSpeedRaw * p->ElectricalSpeedLPFFactor
                          + p->ElectricalSpeedLPF * (1 - p->ElectricalSpeedLPFFactor);

    p->MechanicalSpeed = p->ElectricalSpeedLPF / pole_pairs;
}

void T_Shaped_Acc_Dec(TSHAPEDACCDEC_STRUCT *p)
{
    const float pole_pairs = (p->PolePairs > 0u) ? (float)p->PolePairs : 1.0f;
    const float ts = (p->Ts > 0.0f) ? p->Ts : 0.00005f;

    p->SpeedTargetIncrement = p->TargetSpeed / 60.0f * ts;

    p->SpeedIncrement = p->AccSpeed / 60.0f * ts * ts;

    if(p->SpeedChangeIncrement < p->SpeedTargetIncrement){

        p->SpeedChangeIncrement += p->SpeedIncrement;

        if(p->SpeedChangeIncrement >= p->SpeedTargetIncrement){
            p->SpeedChangeIncrement = p->SpeedTargetIncrement;
        }
    }else if(p->SpeedChangeIncrement > p->SpeedTargetIncrement){

        p->SpeedChangeIncrement -= p->SpeedIncrement;
        if(p->SpeedChangeIncrement <= p->SpeedTargetIncrement){
            p->SpeedChangeIncrement = p->SpeedTargetIncrement;
        }
    }

    p->SpeedIncrementDelta = fabsf(p->SpeedChangeIncrement) - p->SpeedIncrementLast;

    p->SpeedIncrementLast = fabsf(p->SpeedChangeIncrement);

    p->MotionState = p->SpeedIncrementDelta > 0 ? ACCELERATE :
                    (p->SpeedIncrementDelta < 0 ? DECELERATE : UNIFORM);

    p->SpeedOutDir = p->SpeedChangeIncrement > 0 ? -1 : 1;

    p->SpeedOut = p->SpeedChangeIncrement * 60.0f / ts * pole_pairs;
}
