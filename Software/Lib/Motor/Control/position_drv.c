#include "position_drv.h"
#include "motor_internal.h"

static s32 Position_SafeCounts(s32 counts)
{
    return counts > 0 ? counts : 1;
}

s32 Position_NormalizeMechanical(s32 pos, s32 counts)
{
    counts = Position_SafeCounts(counts);
    s32 wrapped = pos % counts;

    if (wrapped < 0)
    {
        wrapped += counts;
    }
    return wrapped;
}

s32 Position_WrapMechanicalDelta(s32 delta, s32 counts)
{
    counts = Position_SafeCounts(counts);
    s32 wrapped = delta % counts;
    s32 half = counts / 2;

    if (wrapped > half)
    {
        wrapped -= counts;
    }
    else if (wrapped < -half)
    {
        wrapped += counts;
    }
    return wrapped;
}

s32 Position_NearestMechanicalSetpoint(s32 current, s32 target, s32 counts)
{
    s32 current_mod = Position_NormalizeMechanical(current, counts);
    s32 target_mod = Position_NormalizeMechanical(target, counts);

    return current + Position_WrapMechanicalDelta(target_mod - current_mod, counts);
}

void Position_SetHome(struct motor_s *m)
{
    if (m == NULL)
    {
        return;
    }

    MOTORCONTROL_STRUCT * const mc = &m->mc;
    POSITION_STRUCT * const p = &mc->Position;
    const s32 counts = (s32)p->ElectricalValMax + 1;

    p->ElectricalPosThis = (s32)(mc->EAngle.ElectricalAnglePU * (float)counts);
    p->ElectricalPosLast = p->ElectricalPosThis;
    p->ElectricalPosChange = 0;
    p->ElectricalPosSum = 0;
    p->MechanicalPosRaw = 0;
    p->PosCalculateCnt = 0;
    p->MechanicalPosSet = Position_NormalizeMechanical(p->MechanicalPosSet, counts);

    PID_Clear(&mc->PosPid);
    PID_Clear(&mc->SpdPid);
}

void Calculate_Position(POSITION_STRUCT *p, u8 pole_pairs)
{
    const s32 pole_pair_count = (pole_pairs > 0u) ? (s32)pole_pairs : 1;
    const s32 counts = Position_SafeCounts((s32)p->ElectricalValMax + 1);
    const s32 half_counts = counts / 2;

    p->ElectricalPosChange = p->ElectricalPosThis - p->ElectricalPosLast;
    p->ElectricalPosLast = p->ElectricalPosThis;

    if (p->ElectricalPosChange >= half_counts)
    {
        p->ElectricalPosChange -= counts;
    }
    if (p->ElectricalPosChange <= -half_counts)
    {
        p->ElectricalPosChange += counts;
    }

    p->ElectricalPosSum += p->ElectricalPosChange;

    p->MechanicalPosRaw = p->ElectricalPosSum / pole_pair_count;
}
