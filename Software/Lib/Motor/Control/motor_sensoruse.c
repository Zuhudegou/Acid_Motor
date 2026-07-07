#include "motor_sensoruse.h"
#include "motor_internal.h"

#define SENSORUSE_SPEED_IQ_LIMIT_A             (2.0f)
#define SENSORUSE_SPEED_SOFT_CURRENT_RATIO     (0.55f)

static void Current_Close_Loop(motor_t *m);
static void Speed_Current_Loop(motor_t *m);
static void Pos_Speed_Current_Loop(motor_t *m);
static void Sensoruse_Update_Position(motor_t *m);
static void Sensoruse_Current_Inner_Loop(motor_t *m);
static void Sensoruse_Disable_Output(motor_t *m);
static void Sensoruse_Reset_Speed_Estimator(motor_t *m);

void Sensoruse_Control(motor_t *m)
{
    Calculate_Sin_Cos(m->mc.EAngle.ElectricalAnglePU, &m->mc.Foc.SinVal, &m->mc.Foc.CosVal);
    if (m->mc.Motor.RunMode != POS_SPEED_CURRENT_LOOP)
    {
        Sensoruse_Update_Position(m);
    }

    switch (m->mc.Motor.RunMode)
    {
        case CURRENT_CLOSE_LOOP:
            Current_Close_Loop(m);
            break;
        case SPEED_CURRENT_LOOP:
            Speed_Current_Loop(m);
            break;
        case POS_SPEED_CURRENT_LOOP:
            Pos_Speed_Current_Loop(m);
            break;
        default:
            Sensoruse_Disable_Output(m);
            break;
    }
}

static void Sensoruse_Disable_Output(motor_t *m)
{
    m->mc.Foc.DutyCycleA = 0U;
    m->mc.Foc.DutyCycleB = 0U;
    m->mc.Foc.DutyCycleC = 0U;
}

static void Sensoruse_Update_Position(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;
    POSITION_STRUCT * const pos = &mc->Position;
    const u16 pos_div = (m->cfg.pos_div > 0u) ? m->cfg.pos_div : 1u;

    pos->PosCalculateCnt++;
    if (pos->PosCalculateCnt >= pos_div)
    {
        const s32 mechanical_counts = (s32)pos->ElectricalValMax + 1;
        pos->PosCalculateCnt = 0;
        pos->ElectricalPosThis = (s32)(mc->EAngle.ElectricalAnglePU * (float)mechanical_counts);
        Calculate_Position(pos, mc->EAngle.PolePairs);
    }
}

static void Current_Close_Loop(motor_t *m)
{
    Sensoruse_Current_Inner_Loop(m);
}

static void Sensoruse_Reset_Speed_Estimator(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;

    mc->Speed.SpeedCalculateCnt = 0U;
    mc->Speed.ElectricalPosThis = mc->EAngle.ElectricalAnglePU;
    mc->Speed.ElectricalPosLast = mc->EAngle.ElectricalAnglePU;
    mc->Speed.ElectricalPosChange = 0.0f;
    mc->Speed.ElectricalSpeedRaw = 0.0f;
    mc->Speed.ElectricalSpeedLPF = 0.0f;
    mc->Speed.MechanicalSpeed = 0.0f;

    mc->TAccDec.TargetSpeed = 0.0f;
    mc->TAccDec.SpeedTargetIncrement = 0.0f;
    mc->TAccDec.SpeedIncrement = 0.0f;
    mc->TAccDec.SpeedChangeIncrement = 0.0f;
    mc->TAccDec.SpeedIncrementDelta = 0.0f;
    mc->TAccDec.SpeedIncrementLast = 0.0f;
    mc->TAccDec.SpeedOut = 0.0f;

    PID_Clear(&mc->SpdPid);
    PID_Clear(&mc->IqPid);
    PID_Clear(&mc->IdPid);
    mc->IqPid.Ref = 0.0f;
    mc->IdPid.Ref = 0.0f;
}

static void Speed_Current_Loop(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;
    SPEED_STRUCT * const spd = &mc->Speed;
    TSHAPEDACCDEC_STRUCT * const acc = &mc->TAccDec;
    float phase_abs = mc->Sample.IuReal;

    if (phase_abs < 0.0f) phase_abs = -phase_abs;
    if (mc->Sample.IvReal > phase_abs) phase_abs = mc->Sample.IvReal;
    if (-mc->Sample.IvReal > phase_abs) phase_abs = -mc->Sample.IvReal;
    if (mc->Sample.IwReal > phase_abs) phase_abs = mc->Sample.IwReal;
    if (-mc->Sample.IwReal > phase_abs) phase_abs = -mc->Sample.IwReal;

    if (m->cfg.over_current > 0.0f &&
        phase_abs >= (m->cfg.over_current * SENSORUSE_SPEED_SOFT_CURRENT_RATIO))
    {
        Sensoruse_Reset_Speed_Estimator(m);
        Sensoruse_Current_Inner_Loop(m);
        return;
    }

    if (mc->Speed.MechanicalSpeedSet <= 5.0f && mc->Speed.MechanicalSpeedSet >= -5.0f)
    {
        Sensoruse_Reset_Speed_Estimator(m);
        Sensoruse_Current_Inner_Loop(m);
        return;
    }

    spd->SpeedCalculateCnt++;
    acc->TargetSpeed = mc->Speed.MechanicalSpeedSet;
    T_Shaped_Acc_Dec(acc);

    if (spd->SpeedCalculateCnt >= spd->SpeedDivisionFactor)
    {
        spd->SpeedCalculateCnt = 0;
        spd->ElectricalPosThis = mc->EAngle.ElectricalAnglePU;
        Calculate_Speed(spd);
        mc->SpdPid.Ref = acc->SpeedOut;
        mc->SpdPid.Fbk = spd->ElectricalSpeedLPF;
        if (mc->SpdPid.Fbk > -2000.0f && mc->SpdPid.Fbk < 2000.0f)
        {
            mc->SpdPid.Kp = mc->SpdPid.KpMax;
        }
        else
        {
            mc->SpdPid.Kp = mc->SpdPid.KpMin;
        }
        PID_Control(&mc->SpdPid);
        mc->IqPid.Ref = mc->SpdPid.Out;
        if (mc->IqPid.Ref > SENSORUSE_SPEED_IQ_LIMIT_A)
        {
            mc->IqPid.Ref = SENSORUSE_SPEED_IQ_LIMIT_A;
        }
        else if (mc->IqPid.Ref < -SENSORUSE_SPEED_IQ_LIMIT_A)
        {
            mc->IqPid.Ref = -SENSORUSE_SPEED_IQ_LIMIT_A;
        }
        mc->IdPid.Ref = 0.0f;
    }

    Sensoruse_Current_Inner_Loop(m);
}

static void Pos_Speed_Current_Loop(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;
    POSITION_STRUCT * const pos = &mc->Position;
    SPEED_STRUCT * const spd = &mc->Speed;
    const u16 pos_div = (m->cfg.pos_div > 0u) ? m->cfg.pos_div : 1u;
    const float pole_pairs = (mc->EAngle.PolePairs > 0u) ? (float)mc->EAngle.PolePairs : 1.0f;

    pos->PosCalculateCnt++;
    if (pos->PosCalculateCnt >= pos_div)
    {
        const s32 mechanical_counts = (s32)pos->ElectricalValMax + 1;
        pos->PosCalculateCnt = 0;
        pos->ElectricalPosThis = (s32)(mc->EAngle.ElectricalAnglePU * (float)mechanical_counts);
        Calculate_Position(pos, mc->EAngle.PolePairs);
        s32 target = Position_NearestMechanicalSetpoint(pos->MechanicalPosRaw,
                                                        pos->MechanicalPosSet,
                                                        mechanical_counts);
        mc->PosPid.Fbk = (float)pos->MechanicalPosRaw * pole_pairs;
        mc->PosPid.Ref = (float)target * pole_pairs;
        PID_Control(&mc->PosPid);
    }

    spd->SpeedCalculateCnt++;
    if (spd->SpeedCalculateCnt >= spd->SpeedDivisionFactor)
    {
        spd->SpeedCalculateCnt = 0;
        spd->ElectricalPosThis = mc->EAngle.ElectricalAnglePU;
        Calculate_Speed(spd);
        mc->SpdPid.Ref = mc->PosPid.Out;
        mc->SpdPid.Fbk = spd->ElectricalSpeedLPF;
        if (mc->SpdPid.Fbk > -2000.0f && mc->SpdPid.Fbk < 2000.0f)
        {
            mc->SpdPid.Kp = mc->SpdPid.KpMax;
        }
        else
        {
            mc->SpdPid.Kp = mc->SpdPid.KpMin;
        }
        PID_Control(&mc->SpdPid);
        mc->IqPid.Ref = mc->SpdPid.Out;
    }

    Sensoruse_Current_Inner_Loop(m);
}

static void Sensoruse_Current_Inner_Loop(motor_t *m)
{
    MOTORCONTROL_STRUCT * const mc = &m->mc;
    FOC_STRUCT * const foc = &mc->Foc;
    SAMPLE_STRUCT * const smp = &mc->Sample;

    foc->Iu = smp->IuReal;
    foc->Iv = smp->IvReal;
    Clark_Transform(foc);
    Park_Transform(foc);
    foc->IdLPF = foc->Id * foc->IdLPFFactor + foc->IdLPF * (1.0f - foc->IdLPFFactor);
    foc->IqLPF = foc->Iq * foc->IqLPFFactor + foc->IqLPF * (1.0f - foc->IqLPFFactor);
    mc->IqPid.Fbk = foc->IqLPF;
    mc->IdPid.Fbk = foc->IdLPF;
    PID_Control(&mc->IqPid);
    PID_Control(&mc->IdPid);
    foc->Uq = mc->IqPid.Out;
    foc->Ud = mc->IdPid.Out;
    IPark_Transform(foc);
    foc->Ubus = smp->BusReal;
    Calculate_SVPWM(foc);
}
