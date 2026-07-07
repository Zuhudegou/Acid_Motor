#ifndef __SAMPLE_DRV_H
#define __SAMPLE_DRV_H

#include "motor_types.h"

typedef struct
{
  s8    CurrentDir;
    u8    CalibEndFlag;
    u16   OffsetCnt;
    s32   BusOffset;
    s32   IuOffset;
    s32   IvOffset;
    s32   IwOffset;
    s32   BusRaw;
    s32   IuRaw;
    s32   IvRaw;
    s32   IwRaw;
    float IuReal;
    float IvReal;
    float IwReal;
    float BusReal;
    float BusCalibReal;
    float BusChange;
    float BusFactor;
    float CurrentFactor;
    u16   AdcBuff[3];
}SAMPLE_STRUCT;

void Calculate_Adc_Offset(SAMPLE_STRUCT *p);
void Calculate_Phase_Current(SAMPLE_STRUCT *p);
void Calculate_Bus_Voltage(SAMPLE_STRUCT *p);

#endif
