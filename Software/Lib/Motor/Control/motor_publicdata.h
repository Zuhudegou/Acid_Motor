#ifndef __MOTOR_PUBLICDATA_H
#define __MOTOR_PUBLICDATA_H

#include "motor_types.h"
#include "foc_drv.h"
#include "pid_drv.h"
#include "math_drv.h"
#include "speed_drv.h"
#include "sample_drv.h"
#include "eangle_drv.h"
#include "position_drv.h"

#define INV_SQRT3          0.57735f

typedef enum{
    RESISTANCE_IDENTIFICATION = 0,
    INDUCTANCE_IDENTIFICATION,
    ENCODER_ROTOR_ALIGN
}IDENTIFY_STATE;

typedef enum{
    ADC_CALIB = 0,
    MOTOR_STOP,
    MOTOR_ERROR,
    MOTOR_IDENTIFY,
    MOTOR_SENSORUSE
}MOTOR_RUN_STATE;

#define CURRENT_CLOSE_LOOP                    0X02
#define SPEED_CURRENT_LOOP                  0X03
#define POS_SPEED_CURRENT_LOOP                0X04

typedef enum{
    NONE_ERR = 0,
    ADC_CALIB_ERR,
    ENCODER_ERR,
    POWER_VOLT_ERR,
    OVER_CURRENT_ERR,
    TEMPERATURE_ERR
}MOTOR_ERROR_CODE;

#define HALF_PI  1.5707963f
#define ONE_PI   3.1415926f
#define TWO_PI   6.2831853f

#define CW  0
#define CCW 1

typedef struct
{
    MOTOR_RUN_STATE    RunState;
    MOTOR_ERROR_CODE    ErrorCode;
    u8    RunMode;
}MOTOR_STRUCT;

typedef struct
{
    IDENTIFY_STATE    State;
    u8    Flag;
    u8    EndFlag;
    u16   Count;
    u16   WaitTim;
    float Rs;
    float Ls;
    float Ld;
    float LsSum;
    float CurMax;
    float CurSum;
    float CurAverage[2];
    float VoltageSet[2];
}IDENTIFY_STRUCT;

typedef struct
{
    MOTOR_STRUCT                         Motor;
  IDENTIFY_STRUCT              Identify;
    E_ANGLE_STRUCT                       EAngle;
    SAMPLE_STRUCT                       Sample;
    FOC_STRUCT                           Foc;
    PID_STRUCT                            IqPid;
    PID_STRUCT                          IdPid;
    PID_STRUCT                          SpdPid;
    PID_STRUCT                           PosPid;
    SPEED_STRUCT                        Speed;
    TSHAPEDACCDEC_STRUCT      TAccDec;
    POSITION_STRUCT                      Position;
}MOTORCONTROL_STRUCT;

struct motor_s;

void Motor_Struct_Init(struct motor_s *m);

#endif
