#include "motor_identify.h"
#include "motor_internal.h"

void Motor_Identify(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;
    E_ANGLE_STRUCT * const ea = &mc->EAngle;
    IDENTIFY_STRUCT * const id = &mc->Identify;
    FOC_STRUCT * const foc = &mc->Foc;
    SAMPLE_STRUCT * const smp = &mc->Sample;
    const float ts = (m->cfg.ts > 0.0f) ? m->cfg.ts : 0.00005f;

    switch (id->State)
    {
        case RESISTANCE_IDENTIFICATION:
        {
            if (id->Flag == 0)
            {
                foc->Uq = 0;
                foc->Ud = 0;
                id->Count = 0;
                id->WaitTim = 0;
                id->Flag = 1;
            }

            if (id->Flag == 1)
            {
                float current = (smp->IuReal * foc->Ud * 1.5f) / smp->BusReal;
                if (current >= 0.6f * id->CurMax)
                {
                    id->Flag = 2;
                }
                else
                {
                    foc->Ud += 0.0001f;
                    id->VoltageSet[0] = foc->Ud;
                }
            }

            if (id->Flag == 2)
            {
                id->WaitTim++;
                if (id->WaitTim > 4000)
                {
                    id->CurSum += smp->IuReal;
                }
                if (id->WaitTim >= 4100)
                {
                    id->CurAverage[0] = id->CurSum * 0.01f;
                    id->WaitTim = 0;
                    id->CurSum = 0;
                    id->Flag = 3;
                }
            }

            if (id->Flag == 3)
            {
                float current = (smp->IuReal * foc->Ud * 1.5f) / smp->BusReal;
                if (current >= id->CurMax)
                {
                    id->Flag = 4;
                }
                else
                {
                    foc->Ud += 0.0001f;
                    id->VoltageSet[1] = foc->Ud;
                }
            }

            if (id->Flag == 4)
            {
                id->WaitTim++;
                if (id->WaitTim > 4000)
                {
                    id->CurSum += smp->IuReal;
                }
                if (id->WaitTim >= 4100)
                {
                    id->CurAverage[1] = id->CurSum * 0.01f;
                    id->WaitTim = 0;
                    id->CurSum = 0;
                    id->Flag = 5;
                }
            }

            if (id->Flag == 5)
            {
                id->Rs = (id->VoltageSet[1] - id->VoltageSet[0])
                         / (id->CurAverage[1] - id->CurAverage[0]);
                foc->Ud = 0;
                id->Flag = 0;
                id->State = INDUCTANCE_IDENTIFICATION;
            }

            foc->SinVal = 0;
            foc->CosVal = 1;
            IPark_Transform(foc);
        }
        break;

        case INDUCTANCE_IDENTIFICATION:
        {
            if (id->Flag == 0)
            {
                foc->Uq = 0;
                foc->Ud = 0;
                if (smp->IuReal >= -0.05f && smp->IuReal <= 0.05f)
                {
                    id->Flag = 1;
                }
            }

            if (id->Flag == 1)
            {
                foc->Ud = id->VoltageSet[1];
                id->WaitTim++;

                if (smp->IuReal >= id->CurAverage[1] * 0.95f)
                {
                    id->LsSum += id->Rs * 0.334f * ts * id->WaitTim;
                    id->WaitTim = 0;
                    id->Count++;
                    id->Flag = 0;
                    foc->Ud = 0;
                    if (id->Count >= 100)
                    {
                        id->Flag = 2;
                    }
                }
            }

            if (id->Flag == 2)
            {
                id->Ls = id->LsSum * 0.01f;
                id->Ld = id->Ls;
                id->Flag = 0;
                id->LsSum = 0;
                id->WaitTim = 0;
                id->State = ENCODER_ROTOR_ALIGN;
            }

            foc->SinVal = 0;
            foc->CosVal = 1;
            IPark_Transform(foc);
        }
        break;

        case ENCODER_ROTOR_ALIGN:
        {
            if (ea->CalibFlag == 0)
            {
                foc->Ud += 0.0001f;
                foc->Uq = 0;
                foc->SinVal = 1;
                foc->CosVal = 0;
                if (foc->Ud >= id->VoltageSet[1])
                {
                    foc->Ud = 0;
                    ea->CalibFlag = 1;
                    ea->EncoderValChange = ea->EncoderVal;
                }
            }

            if (ea->CalibFlag == 1)
            {
                foc->Ud += 0.0001f;
                foc->Uq = 0;
                foc->SinVal = 0;
                foc->CosVal = 1;
                if (foc->Ud >= id->VoltageSet[1])
                {
                    const s32 encoder_counts = ea->EncoderValMax + 1;
                    const s32 encoder_half_counts = encoder_counts / 2;
                    ea->EncoderValChange = ea->EncoderVal - ea->EncoderValChange;
                    if (ea->EncoderValChange < -encoder_half_counts)
                    {
                        ea->EncoderValChange += encoder_counts;
                    }
                    else if (ea->EncoderValChange > encoder_half_counts)
                    {
                        ea->EncoderValChange -= encoder_counts;
                    }
                    if (ea->EncoderValChange > 0)
                    {
                        ea->Dir = 1;
                    }
                    else if (ea->EncoderValChange < 0)
                    {
                        ea->Dir = 0;
                    }

                    if (ea->Dir == 1)
                    {
                        ea->CalibOffset = ea->EncoderValMax - ea->EncoderVal;
                    }
                    else
                    {
                        ea->CalibOffset = ea->EncoderVal;
                    }

                    ea->CalibFlag = 0;
                    foc->Ud = 0;
                    id->EndFlag = 1;
                }
            }
            IPark_Transform(foc);
        }
        break;
    }

    foc->Ubus = smp->BusReal;
    Calculate_SVPWM(foc);
}
