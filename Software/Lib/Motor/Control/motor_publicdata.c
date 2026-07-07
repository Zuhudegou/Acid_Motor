#include "motor_publicdata.h"
#include "motor_internal.h"

void Motor_Struct_Init(struct motor_s *m)
{
    if (m == NULL)
    {
        return;
    }

    MOTORCONTROL_STRUCT * const mc = &m->mc;
    const motor_config_t * const cfg = &m->cfg;
    const uint32_t encoder_counts = (uint32_t)cfg->encoder_lines * 4u;
    const u16 encoder_max = (encoder_counts > 0u) ? (u16)(encoder_counts - 1u) : 0u;
    const u16 speed_div = (cfg->speed_div > 0u) ? cfg->speed_div : 1u;
    const u8 pole_pairs = (cfg->pole_pairs > 0u) ? cfg->pole_pairs : 1u;
    const float ts = (cfg->ts > 0.0f) ? cfg->ts : 0.00005f;

    mc->Motor.RunState = ADC_CALIB;

    mc->Motor.ErrorCode = NONE_ERR;

    mc->Motor.RunMode = SPEED_CURRENT_LOOP;

    mc->Sample.CurrentDir = 1;

    mc->Sample.CurrentFactor = cfg->current_scale;

    mc->Sample.BusFactor = cfg->bus_scale;

    mc->EAngle.Dir = CCW;

    mc->EAngle.PolePairs = pole_pairs;

    mc->EAngle.EncoderValMax = encoder_max;

    mc->EAngle.Ts = ts;

    mc->Foc.IdLPFFactor = 0.1f;
    mc->Foc.IqLPFFactor = 0.1f;

    mc->Foc.PwmCycle = (u16)((uint32_t)cfg->pwm_period * 2u);

    mc->Foc.PwmLimit = cfg->pwm_limit;

    mc->Position.ElectricalValMax = encoder_max;

    mc->TAccDec.PolePairs = pole_pairs;
    mc->TAccDec.Ts = ts;
    mc->TAccDec.AccSpeed = cfg->acceleration;

    mc->Speed.PolePairs = pole_pairs;

    mc->Speed.SpeedDivisionFactor = speed_div;

    mc->Speed.ElectricalValMax = encoder_max;

    mc->Speed.ElectricalSpeedLPFFactor = 0.02f;

    mc->Speed.ElectricalSpeedFactor = (1.0f / (ts * (float)speed_div)) * 60.0f;

    mc->Identify.CurMax = 0.6f;

    mc->IqPid.Kp = 0.2f;
    mc->IqPid.Ki = 0.002f;
    mc->IqPid.OutMax = 10;
    mc->IqPid.OutMin = -10;

    mc->IdPid.Kp = 0.2f;
    mc->IdPid.Ki = 0.002f;
    mc->IdPid.OutMax = 10;
    mc->IdPid.OutMin = -10;

    mc->SpdPid.Kp = 0.001f;
    mc->SpdPid.KpMax = 0.005f;
    mc->SpdPid.KpMin = 0.001f;
    mc->SpdPid.Ki = 0.000002f;
    mc->SpdPid.OutMax = 6;
    mc->SpdPid.OutMin = -6;

    mc->PosPid.Kp = 0.5f;
    mc->PosPid.Ki = cfg->pos_ki;
    mc->PosPid.Kd = 0;
    mc->PosPid.OutMax = 14000;
    mc->PosPid.OutMin = -14000;
}
