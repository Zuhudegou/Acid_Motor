#include "fuzzy_pid.h"

#include <stddef.h>
#include <stdint.h>

enum
{
  FUZZY_NB = -3,
  FUZZY_NM = -2,
  FUZZY_NS = -1,
  FUZZY_ZO = 0,
  FUZZY_PS = 1,
  FUZZY_PM = 2,
  FUZZY_PB = 3
};

static const float kFuzzyDomainCenter[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};

static const int8_t kRuleKp[7][7] = {
    {FUZZY_PB, FUZZY_PB, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZO, FUZZY_ZO},
    {FUZZY_PB, FUZZY_PB, FUZZY_PM, FUZZY_PS, FUZZY_PS, FUZZY_ZO, FUZZY_NS},
    {FUZZY_PM, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZO, FUZZY_NS, FUZZY_NS},
    {FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZO, FUZZY_NS, FUZZY_NM, FUZZY_NM},
    {FUZZY_PS, FUZZY_PS, FUZZY_ZO, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_NM},
    {FUZZY_PS, FUZZY_ZO, FUZZY_NS, FUZZY_NM, FUZZY_NM, FUZZY_NM, FUZZY_NB},
    {FUZZY_ZO, FUZZY_ZO, FUZZY_NM, FUZZY_NM, FUZZY_NM, FUZZY_NB, FUZZY_NB}};

static const int8_t kRuleKi[7][7] = {
    {FUZZY_NB, FUZZY_NB, FUZZY_NM, FUZZY_NM, FUZZY_NS, FUZZY_ZO, FUZZY_ZO},
    {FUZZY_NB, FUZZY_NB, FUZZY_NM, FUZZY_NS, FUZZY_NS, FUZZY_ZO, FUZZY_ZO},
    {FUZZY_NM, FUZZY_NM, FUZZY_NS, FUZZY_NS, FUZZY_ZO, FUZZY_PS, FUZZY_PS},
    {FUZZY_NM, FUZZY_NS, FUZZY_NS, FUZZY_ZO, FUZZY_PS, FUZZY_PS, FUZZY_PM},
    {FUZZY_NS, FUZZY_NS, FUZZY_ZO, FUZZY_PS, FUZZY_PS, FUZZY_PM, FUZZY_PM},
    {FUZZY_ZO, FUZZY_ZO, FUZZY_PS, FUZZY_PS, FUZZY_PM, FUZZY_PB, FUZZY_PB},
    {FUZZY_ZO, FUZZY_ZO, FUZZY_PS, FUZZY_PM, FUZZY_PM, FUZZY_PB, FUZZY_PB}};

static const int8_t kRuleKd[7][7] = {
    {FUZZY_PS, FUZZY_NS, FUZZY_NB, FUZZY_NB, FUZZY_NB, FUZZY_NM, FUZZY_PS},
    {FUZZY_PS, FUZZY_NS, FUZZY_NB, FUZZY_NM, FUZZY_NM, FUZZY_NS, FUZZY_ZO},
    {FUZZY_ZO, FUZZY_NS, FUZZY_NM, FUZZY_NM, FUZZY_NS, FUZZY_NS, FUZZY_ZO},
    {FUZZY_ZO, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_ZO},
    {FUZZY_ZO, FUZZY_ZO, FUZZY_ZO, FUZZY_ZO, FUZZY_ZO, FUZZY_ZO, FUZZY_ZO},
    {FUZZY_PB, FUZZY_NS, FUZZY_PS, FUZZY_PS, FUZZY_PS, FUZZY_PS, FUZZY_PB},
    {FUZZY_PB, FUZZY_PM, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_PS, FUZZY_PB}};

static float clampf(float v, float lo, float hi)
{
  if (v < lo)
  {
    return lo;
  }
  if (v > hi)
  {
    return hi;
  }
  return v;
}

static float absf(float v)
{
  return (v < 0.0f) ? -v : v;
}

static void fuzzify(float x, float mu[7])
{
  uint8_t i;

  x = clampf(x, -3.0f, 3.0f);
  for (i = 0; i < 7; ++i)
  {
    float m = 1.0f - absf(x - kFuzzyDomainCenter[i]);
    mu[i] = (m > 0.0f) ? m : 0.0f;
  }
}

static float infer_and_defuzz(const int8_t rule_table[7][7], float e_norm, float ec_norm)
{
  float mu_e[7] = {0.0f};
  float mu_ec[7] = {0.0f};
  float numerator = 0.0f;
  float denominator = 0.0f;
  uint8_t i;
  uint8_t j;

  fuzzify(e_norm, mu_e);
  fuzzify(ec_norm, mu_ec);

  for (i = 0; i < 7; ++i)
  {
    for (j = 0; j < 7; ++j)
    {
      float w = mu_e[i] * mu_ec[j];
      if (w > 0.0f)
      {
        numerator += w * (float)rule_table[i][j];
        denominator += w;
      }
    }
  }

  if (denominator <= 0.0f)
  {
    return 0.0f;
  }

  return numerator / denominator;
}

void FuzzyPID_Init(FuzzyPID_t *pid, const FuzzyPID_Config_t *cfg)
{
  if ((pid == NULL) || (cfg == NULL))
  {
    return;
  }

  pid->kp_base = cfg->kp_base;
  pid->ki_base = cfg->ki_base;
  pid->kd_base = cfg->kd_base;
  pid->kp_scale = cfg->kp_scale;
  pid->ki_scale = cfg->ki_scale;
  pid->kd_scale = cfg->kd_scale;
  pid->ke = cfg->ke;
  pid->kec = cfg->kec;
  pid->out_min = cfg->out_min;
  pid->out_max = cfg->out_max;
  pid->integral_limit = cfg->integral_limit;

  FuzzyPID_Reset(pid);
}

void FuzzyPID_Reset(FuzzyPID_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->kp = pid->kp_base;
  pid->ki = pid->ki_base;
  pid->kd = pid->kd_base;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

float FuzzyPID_Update(FuzzyPID_t *pid, float setpoint, float measurement, float dt_s)
{
  float error;
  float error_rate;
  float e_norm;
  float ec_norm;
  float dkp;
  float dki;
  float dkd;
  float unsat_out;
  float out;

  if ((pid == NULL) || (dt_s <= 0.0f))
  {
    return 0.0f;
  }

  error = setpoint - measurement;
  error_rate = (error - pid->prev_error) / dt_s;
  e_norm = clampf(error * pid->ke, -3.0f, 3.0f);
  ec_norm = clampf(error_rate * pid->kec, -3.0f, 3.0f);

  dkp = infer_and_defuzz(kRuleKp, e_norm, ec_norm);
  dki = infer_and_defuzz(kRuleKi, e_norm, ec_norm);
  dkd = infer_and_defuzz(kRuleKd, e_norm, ec_norm);

  pid->kp = pid->kp_base + dkp * pid->kp_scale;
  pid->ki = pid->ki_base + dki * pid->ki_scale;
  pid->kd = pid->kd_base + dkd * pid->kd_scale;

  pid->kp = clampf(pid->kp, 0.0f, 1000.0f);
  pid->ki = clampf(pid->ki, 0.0f, 1000.0f);
  pid->kd = clampf(pid->kd, 0.0f, 1000.0f);

  pid->integral += error * dt_s;
  pid->integral = clampf(pid->integral, -pid->integral_limit, pid->integral_limit);

  unsat_out = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * error_rate);
  out = clampf(unsat_out, pid->out_min, pid->out_max);

  if (((unsat_out > pid->out_max) && (error > 0.0f)) ||
      ((unsat_out < pid->out_min) && (error < 0.0f)))
  {
    pid->integral -= error * dt_s;
  }

  pid->prev_error = error;
  return out;
}