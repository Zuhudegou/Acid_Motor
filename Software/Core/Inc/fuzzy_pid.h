#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp_base;
  float ki_base;
  float kd_base;
  float kp_scale;
  float ki_scale;
  float kd_scale;
  float ke;
  float kec;
  float out_min;
  float out_max;
  float integral_limit;
} FuzzyPID_Config_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float kp_base;
  float ki_base;
  float kd_base;
  float kp_scale;
  float ki_scale;
  float kd_scale;
  float ke;
  float kec;
  float out_min;
  float out_max;
  float integral;
  float integral_limit;
  float prev_error;
} FuzzyPID_t;

void FuzzyPID_Init(FuzzyPID_t *pid, const FuzzyPID_Config_t *cfg);
void FuzzyPID_Reset(FuzzyPID_t *pid);
float FuzzyPID_Update(FuzzyPID_t *pid, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif