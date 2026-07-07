#include "sample_drv.h"

void Calculate_Adc_Offset(SAMPLE_STRUCT *p)
{

    if (p->OffsetCnt == 0)
    {
        p->CalibEndFlag = 0;
        p->IuOffset = 0;
        p->IwOffset = 0;
        p->BusOffset = 0;
        p->OffsetCnt = 0;
    }

    if (p->OffsetCnt < 1024)
    {
        p->IuOffset += p->IuRaw;
        p->IwOffset += p->IwRaw;
        p->BusOffset += p->BusRaw;
        p->OffsetCnt++;
    }
    else
    {

        p->IuOffset = p->IuOffset >> 10;
        p->IwOffset = p->IwOffset >> 10;
        p->BusOffset = p->BusOffset >> 10;

        p->BusCalibReal = p->BusOffset * p->BusFactor;

        p->OffsetCnt = 0;
        p->CalibEndFlag = 1;
    }
}

void Calculate_Phase_Current(SAMPLE_STRUCT *p)
{

    p->IuReal = p->CurrentDir * (p->IuRaw - p->IuOffset) * p->CurrentFactor;

    p->IwReal = p->CurrentDir * (p->IwRaw - p->IwOffset) * p->CurrentFactor;

    p->IvReal = -p->IuReal - p->IwReal;
}

void Calculate_Bus_Voltage(SAMPLE_STRUCT *p)
{

    p->BusReal = p->BusRaw * p->BusFactor;

    p->BusChange = p->BusReal - p->BusCalibReal;
}
