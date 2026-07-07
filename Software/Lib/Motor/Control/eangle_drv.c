#include "eangle_drv.h"

void Electrical_Angle_Generator(E_ANGLE_STRUCT *p)
{

    p->ElectricalAngleSetPU += (p->Ts * p->ElectricalAngleSpdSet * EANGLE_RPM_TO_RPS);

    if(p->ElectricalAngleSetPU >= 1)
    {
        p->ElectricalAngleSetPU = p->ElectricalAngleSetPU - 1;
    }

    if(p->ElectricalAngleSetPU < 0)
    {
        p->ElectricalAngleSetPU = p->ElectricalAngleSetPU + 1;
    }
}

void Calculate_Encoder_Data(E_ANGLE_STRUCT *p)
{
    const s32 encoder_counts = p->EncoderValMax + 1;

    if(p->Dir == 1)
    {
        p->EncoderVal = p->EncoderValMax - p->EncoderVal;
    }

    s32 ElectricalVal = ((p->EncoderVal - p->CalibOffset) * p->PolePairs) % encoder_counts;

    if(ElectricalVal < 0)
    {
        ElectricalVal = ElectricalVal + encoder_counts;
    }

    p->ElectricalAnglePU = (float)ElectricalVal / (float)encoder_counts;
}
