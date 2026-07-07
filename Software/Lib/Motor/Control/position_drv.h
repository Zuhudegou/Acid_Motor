#ifndef __POSITION_DRV_H
#define __POSITION_DRV_H

#include "motor_types.h"

typedef struct
{
    u16   PosCalculateCnt;
    u16   ElectricalValMax;
    s32   ElectricalPosThis;
    s32   ElectricalPosLast;
    s32   ElectricalPosChange;
    s32   ElectricalPosSum;
    s32   MechanicalPosRaw;
    s32   MechanicalPosSet;
} POSITION_STRUCT;

struct motor_s;

void Calculate_Position(POSITION_STRUCT *p, u8 pole_pairs);
void Position_SetHome(struct motor_s *m);
s32  Position_NormalizeMechanical(s32 pos, s32 counts);
s32  Position_WrapMechanicalDelta(s32 delta, s32 counts);
s32  Position_NearestMechanicalSetpoint(s32 current, s32 target, s32 counts);

#endif
