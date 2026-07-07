#include "pid_drv.h"

void PID_Control(PID_STRUCT *p)
{
    p->Err = p->Ref - p->Fbk;

    float kp_active = p->Kp;

    float pd_term = kp_active * p->Err + p->Kd * (p->Err - p->ErrLast);
    float out_provisional = pd_term + p->Integrate;

    if (out_provisional < p->OutMax && out_provisional > p->OutMin)
    {
        p->Integrate = p->Integrate + p->Ki * p->Err;
    }

    p->Out = pd_term + p->Integrate;

    if (p->Out >= p->OutMax)
    {
        p->Out = p->OutMax;
    }

    if (p->Out <= p->OutMin)
    {
        p->Out = p->OutMin;
    }

    p->ErrLast = p->Err;
}

void PID_Clear(PID_STRUCT *p)
{
    p->Ref = 0;
    p->Fbk = 0;
    p->Out = 0;
    p->Err = 0;
    p->ErrLast = 0;
    p->Integrate = 0;
}
