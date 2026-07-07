#include <math.h>
#include "motor_system.h"
#include "motor_internal.h"
#include "motor_identify.h"
#include "motor_sensoruse.h"

void Motor_System_Init(motor_t *m)
{
    Motor_Struct_Init(m);
}

void Motor_System_Run(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;
    SAMPLE_STRUCT * const smp = &mc->Sample;

    if (smp->CalibEndFlag == 1)
    {
        Calculate_Bus_Voltage(smp);
        Calculate_Phase_Current(smp);

        mc->IdPid.OutMax =  smp->BusReal * INV_SQRT3;
        mc->IdPid.OutMin = -smp->BusReal * INV_SQRT3;
        mc->IqPid.OutMax =  smp->BusReal * INV_SQRT3;
        mc->IqPid.OutMin = -smp->BusReal * INV_SQRT3;

        if (smp->BusReal <= m->cfg.bus_min || smp->BusReal >= m->cfg.bus_max)
        {
            mc->Motor.RunState = MOTOR_ERROR;
            mc->Motor.ErrorCode = POWER_VOLT_ERR;
        }

        if (smp->IuReal > m->cfg.over_current || smp->IuReal < -m->cfg.over_current ||
            smp->IvReal > m->cfg.over_current || smp->IvReal < -m->cfg.over_current ||
            smp->IwReal > m->cfg.over_current || smp->IwReal < -m->cfg.over_current)
        {
            mc->Motor.RunState = MOTOR_ERROR;
            mc->Motor.ErrorCode = OVER_CURRENT_ERR;
        }
    }

    switch (mc->Motor.RunState)
    {
        case ADC_CALIB:
        {
            Calculate_Adc_Offset(smp);
            if (smp->CalibEndFlag == 1)
            {
                mc->Motor.RunState = MOTOR_IDENTIFY;
            }
        }
        break;

        case MOTOR_IDENTIFY:
        {
            Motor_Identify(m);
            if (mc->Identify.EndFlag == 1)
            {
                if (mc->Motor.RunState != MOTOR_ERROR)
                {
                    mc->Motor.RunState = MOTOR_SENSORUSE;
                    Calculate_Encoder_Data(&mc->EAngle);
                    motor_home_position(m);
                    mc->Speed.SpeedCalculateCnt = 0U;
                    mc->Speed.ElectricalPosThis = mc->EAngle.ElectricalAnglePU;
                    mc->Speed.ElectricalPosLast = mc->EAngle.ElectricalAnglePU;
                    mc->Speed.ElectricalPosChange = 0.0f;
                    mc->Speed.ElectricalSpeedRaw = 0.0f;
                    mc->Speed.ElectricalSpeedLPF = 0.0f;
                    mc->Speed.MechanicalSpeed = 0.0f;
                }
            }
        }
        break;

        case MOTOR_SENSORUSE:
        {
            Calculate_Encoder_Data(&mc->EAngle);
            Sensoruse_Control(m);
        }
        break;

        case MOTOR_ERROR:
        {
            mc->Foc.DutyCycleA = 0;
            mc->Foc.DutyCycleB = 0;
            mc->Foc.DutyCycleC = 0;
            m->output_enable = 0;
        }
        break;

        case MOTOR_STOP:
        {
            mc->Foc.DutyCycleA = 0;
            mc->Foc.DutyCycleB = 0;
            mc->Foc.DutyCycleC = 0;
        }
        break;

        default:
        break;
    }
}
