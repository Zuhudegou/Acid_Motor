#include "foc_drv.h"

void Clark_Transform(FOC_STRUCT *p)
{
    p->Ialpha = p->Iu;
    p->Ibeta  = (p->Iu * FOC_INV_SQRT3) + (p->Iv * FOC_2_INV_SQRT3);
}

void Park_Transform(FOC_STRUCT *p)
{
    p->Id = (p->Ialpha * p->CosVal) + (p->Ibeta * p->SinVal);
    p->Iq = (-p->Ialpha * p->SinVal) + (p->Ibeta * p->CosVal);
}

void IPark_Transform(FOC_STRUCT *p)
{
    p->Ualpha = p->Ud * p->CosVal - p->Uq * p->SinVal;
    p->Ubeta  = p->Uq * p->CosVal + p->Ud * p->SinVal;
}

void Calculate_SVPWM(FOC_STRUCT *p)
{
    float U1, U2, U3 = 0;
    float X, Y, Z = 0;
    float T1, T2, T1Temp, T2Temp = 0;
    u8 A, B, C, N = 0;
    u16 Ta, Tb, Tc = 0;

    U1 = p->Ubeta;
    U2 = (FOC_SQRT3_DIV2 * p->Ualpha) - (FOC_HALF * p->Ubeta);
    U3 = (-FOC_SQRT3_DIV2 * p->Ualpha) - (FOC_HALF * p->Ubeta);

    if (U1 > 0) { A = 1; } else { A = 0; }
    if (U2 > 0) { B = 1; } else { B = 0; }
    if (U3 > 0) { C = 1; } else { C = 0; }
    N = 4 * C + 2 * B + A;

    X = (FOC_SQRT3 * p->PwmCycle * p->Ubeta) / p->Ubus;
    Y = (FOC_3_DIV2 * p->Ualpha * p->PwmCycle + FOC_SQRT3_DIV2 * p->Ubeta * p->PwmCycle) / p->Ubus;
    Z = (-FOC_3_DIV2 * p->Ualpha * p->PwmCycle + FOC_SQRT3_DIV2 * p->Ubeta * p->PwmCycle) / p->Ubus;

    switch (N)
    {
        case 3: { T1 = -Z; T2 =  X; } break;
        case 1: { T1 =  Z; T2 =  Y; } break;
        case 5: { T1 =  X; T2 = -Y; } break;
        case 4: { T1 = -X; T2 =  Z; } break;
        case 6: { T1 = -Y; T2 = -Z; } break;
        case 2: { T1 =  Y; T2 = -X; } break;
        default: { T1 = 0; T2 = 0; } break;
    }

    T1Temp = T1;
    T2Temp = T2;
    if (T1 + T2 > p->PwmLimit)
    {
        T1 = p->PwmLimit * T1Temp / (T1Temp + T2Temp);
        T2 = p->PwmLimit * T2Temp / (T1Temp + T2Temp);
    }

    Ta = (p->PwmCycle - T1 - T2) * FOC_QUARTER;
    Tb = Ta + T1 * FOC_HALF;
    Tc = Tb + T2 * FOC_HALF;

    switch (N)
    {
        case 3:
            p->DutyCycleA = Ta;
            p->DutyCycleB = Tb;
            p->DutyCycleC = Tc;
            break;
        case 1:
            p->DutyCycleA = Tb;
            p->DutyCycleB = Ta;
            p->DutyCycleC = Tc;
            break;
        case 5:
            p->DutyCycleA = Tc;
            p->DutyCycleB = Ta;
            p->DutyCycleC = Tb;
            break;
        case 4:
            p->DutyCycleA = Tc;
            p->DutyCycleB = Tb;
            p->DutyCycleC = Ta;
            break;
        case 6:
            p->DutyCycleA = Tb;
            p->DutyCycleB = Tc;
            p->DutyCycleC = Ta;
            break;
        case 2:
            p->DutyCycleA = Ta;
            p->DutyCycleB = Tc;
            p->DutyCycleC = Tb;
            break;
        default:
            p->DutyCycleA = Ta;
            p->DutyCycleB = Tb;
            p->DutyCycleC = Tc;
            break;
    }
}
