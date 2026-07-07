/*
 * motor.c — encapsulated FOC motor control library: instance lifecycle, target
 * management, telemetry, and the real-time control tick (motor_tick).
 */

#include "motor_internal.h"
#include "motor_control.h"

/* The single motor instance wired by this firmware. */
motor_t g_motor;

static int32_t motor_counts_per_rev(const motor_t *m)
{
    int32_t counts = (int32_t)m->cfg.encoder_lines * 4;
    return counts > 0 ? counts : 1;
}

static int32_t motor_wrap_counts(int32_t value, int32_t modulo)
{
    int32_t wrapped;

    if (modulo <= 0)
    {
        return value;
    }

    wrapped = value % modulo;
    if (wrapped < 0)
    {
        wrapped += modulo;
    }
    return wrapped;
}

static int32_t motor_degrees_to_counts(const motor_t *m, float degrees)
{
    int32_t counts = motor_counts_per_rev(m);
    float raw = degrees * (float)counts / 360.0f;
    int32_t rounded = (raw >= 0.0f) ? (int32_t)(raw + 0.5f) : (int32_t)(raw - 0.5f);

    return motor_wrap_counts(rounded, counts);
}

static float motor_counts_to_degrees(const motor_t *m, int32_t counts_value)
{
    int32_t counts = motor_counts_per_rev(m);
    int32_t wrapped = motor_wrap_counts(counts_value, counts);

    return (float)wrapped * 360.0f / (float)counts;
}

static float motor_clampf(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
    }
    return value;
}

static float motor_current_ref_limit(const motor_t *m)
{
    float limit = m->mc.SpdPid.OutMax;

    if (limit <= 0.0f)
    {
        limit = m->cfg.over_current * 0.5f;
    }
    if (limit <= 0.0f)
    {
        limit = 3.0f;
    }
    return limit;
}

static uint8_t motor_is_sensored_mode(uint8_t mode)
{
    return (mode == CURRENT_CLOSE_LOOP ||
            mode == SPEED_CURRENT_LOOP ||
            mode == POS_SPEED_CURRENT_LOOP) ? 1U : 0U;
}

static void motor_reset_control_loops(motor_t *m, uint8_t mode)
{
    (void)mode;

    PID_Clear(&m->mc.IqPid);
    PID_Clear(&m->mc.IdPid);
    PID_Clear(&m->mc.SpdPid);
    PID_Clear(&m->mc.PosPid);

    m->mc.Speed.SpeedCalculateCnt = 0U;
    m->mc.Speed.ElectricalPosThis = m->mc.EAngle.ElectricalAnglePU;
    m->mc.Speed.ElectricalPosLast = m->mc.EAngle.ElectricalAnglePU;
    m->mc.Speed.ElectricalPosChange = 0.0f;
    m->mc.Speed.ElectricalSpeedRaw = 0.0f;
    m->mc.Speed.ElectricalSpeedLPF = 0.0f;
    m->mc.Speed.MechanicalSpeed = 0.0f;
    m->mc.Position.PosCalculateCnt = 0U;
    m->mc.TAccDec.TargetSpeed = 0.0f;
    m->mc.TAccDec.SpeedTargetIncrement = 0.0f;
    m->mc.TAccDec.SpeedIncrement = 0.0f;
    m->mc.TAccDec.SpeedChangeIncrement = 0.0f;
    m->mc.TAccDec.SpeedIncrementDelta = 0.0f;
    m->mc.TAccDec.SpeedIncrementLast = 0.0f;
    m->mc.TAccDec.SpeedOut = 0.0f;
    m->mc.TAccDec.FinishFlag = 0U;

    m->mc.Foc.IdLPF = 0.0f;
    m->mc.Foc.IqLPF = 0.0f;
    m->mc.Foc.DutyCycleA = 0U;
    m->mc.Foc.DutyCycleB = 0U;
    m->mc.Foc.DutyCycleC = 0U;
}

/* ===================== Lifecycle ===================== */

void motor_init(motor_t *m, const motor_config_t *cfg)
{
    if (m == NULL || cfg == NULL)
    {
        return;
    }

    /* The control state is seeded from the board config, so copy cfg first. */
    m->cfg = *cfg;
    motor_control(m, MOTOR_CONTROL_INIT);
    m->speed_dir       = 1;
    m->speed_target_rpm    = 0.0f;
    m->current_target_amp  = 0.0f;
    m->position_target     = 0;
    m->use_adc_target  = 1U;           /* former static default */
    m->output_enable  = 0U;

    /* Derive the FOC SVPWM cycle from the config. pwm_period is the TIM1 ARR;
     * for center-aligned PWM the SVPWM operates in 0..2*period space. */
    m->mc.Foc.PwmCycle = (uint16_t)((uint32_t)cfg->pwm_period * 2u);
    m->mc.Foc.PwmLimit = cfg->pwm_limit;
}

void motor_start(motor_t *m)
{
    m->mc.Motor.RunState = ADC_CALIB;
}

void motor_stop(motor_t *m)
{
    m->mc.Motor.RunState = MOTOR_STOP;
    m->output_enable = 0U;
    m->mc.Foc.DutyCycleA = 0U;
    m->mc.Foc.DutyCycleB = 0U;
    m->mc.Foc.DutyCycleC = 0U;
}

void motor_clear_error(motor_t *m)
{
    m->mc.Motor.ErrorCode = NONE_ERR;
    if (m->mc.Motor.RunState == MOTOR_ERROR)
    {
        m->mc.Motor.RunState = ADC_CALIB;
    }
}

void motor_enter_mode(motor_t *m, MOTOR_RUN_STATE state, uint8_t mode)
{
    int32_t current_position;
    uint8_t mode_changed;

    if (state == MOTOR_SENSORUSE && motor_is_sensored_mode(mode) == 0U)
    {
        motor_stop(m);
        return;
    }

    if (state == MOTOR_SENSORUSE && motor_sensored_ready(m) == 0U)
    {
        m->mc.Motor.RunMode = mode;
        return;
    }

    mode_changed = (m->mc.Motor.RunState != state || m->mc.Motor.RunMode != mode) ? 1U : 0U;
    motor_clear_error(m);
    m->mc.Motor.RunState = state;
    m->mc.Motor.RunMode = mode;
    if (mode_changed != 0U)
    {
        motor_reset_control_loops(m, mode);
    }
    if (mode == POS_SPEED_CURRENT_LOOP)
    {
        current_position = motor_wrap_counts(m->mc.Position.MechanicalPosRaw, motor_counts_per_rev(m));
        m->position_target = current_position;
        m->mc.Position.MechanicalPosSet = current_position;
        PID_Clear(&m->mc.PosPid);
        PID_Clear(&m->mc.SpdPid);
    }
}

void motor_home_position(motor_t *m)
{
    m->position_target = 0;
    m->mc.Position.MechanicalPosSet = 0;
    motor_control(m, MOTOR_CONTROL_HOME);
}

uint8_t motor_output_enabled(const motor_t *m)
{
    return m->output_enable;
}

uint8_t motor_sensored_ready(const motor_t *m)
{
    if (m == NULL)
    {
        return 0U;
    }

    return (m->mc.Sample.CalibEndFlag != 0U &&
            m->mc.Identify.EndFlag != 0U) ? 1U : 0U;
}

/* ===================== Setters (foreground) ===================== */

void motor_set_state(motor_t *m, MOTOR_RUN_STATE s)            { m->mc.Motor.RunState = s; }
void motor_set_mode(motor_t *m, uint8_t mode)                  { m->mc.Motor.RunMode = mode; }
void motor_set_target_speed(motor_t *m, float rpm)             { m->speed_target_rpm = rpm; }
void motor_set_target_current(motor_t *m, float amp)           { m->current_target_amp = motor_clampf(amp, -motor_current_ref_limit(m), motor_current_ref_limit(m)); }
void motor_set_target_position(motor_t *m, int32_t pos)        { m->position_target = motor_wrap_counts(pos, motor_counts_per_rev(m)); }
void motor_set_target_angle(motor_t *m, float degrees)         { m->position_target = motor_degrees_to_counts(m, degrees); }
void motor_use_adc_target(motor_t *m, uint8_t enable)          { m->use_adc_target = enable; }
void motor_set_speed_dir(motor_t *m, int8_t dir)               { m->speed_dir = dir; }

/* ===================== Getters / telemetry ===================== */

MOTOR_RUN_STATE   motor_get_state(const motor_t *m)            { return m->mc.Motor.RunState; }
uint8_t           motor_get_mode(const motor_t *m)             { return m->mc.Motor.RunMode; }
uint8_t           motor_is_adc_target(const motor_t *m)        { return m->use_adc_target; }
float             motor_get_target_speed(const motor_t *m)     { return m->speed_target_rpm; }
float             motor_get_target_current(const motor_t *m)   { return m->current_target_amp; }
int32_t           motor_get_target_position(const motor_t *m)  { return m->position_target; }
float             motor_get_target_angle(const motor_t *m)     { return motor_counts_to_degrees(m, m->position_target); }
int8_t            motor_get_speed_dir(const motor_t *m)        { return m->speed_dir; }

void motor_get_telemetry(const motor_t *m, motor_telemetry_t *t)
{
    t->state          = m->mc.Motor.RunState;
    t->error_code     = (uint8_t)m->mc.Motor.ErrorCode;
    t->run_mode       = m->mc.Motor.RunMode;
    t->bus_real       = m->mc.Sample.BusReal;
    t->iu_real        = m->mc.Sample.IuReal;
    t->iv_real        = m->mc.Sample.IvReal;
    t->iw_real        = m->mc.Sample.IwReal;
    t->iq             = m->mc.IqPid.Fbk;
    t->current_set    = m->mc.IqPid.Ref;
    t->speed_fb       = m->mc.Speed.MechanicalSpeed;
    t->speed_set      = m->mc.Speed.MechanicalSpeedSet;
    t->encoder        = m->mc.EAngle.EncoderVal;
    t->position_raw   = m->mc.Position.MechanicalPosRaw;
    t->position_target = m->position_target;
    t->position_angle_deg = motor_counts_to_degrees(m, m->mc.Position.MechanicalPosRaw);
    t->position_target_deg = motor_counts_to_degrees(m, m->position_target);
    t->use_adc_target = m->use_adc_target;
    t->speed_dir      = m->speed_dir;
}

/* ===================== Target apply (per tick) =====================
 * Replaces the former Motor_Driver_ApplyAdcTarget / ApplySetTarget. Operates on
 * the instance; logic is identical to the original (including the literal scale
 * factors 0.002f / 0.5f, to be cleaned up in a later stage). */

static void motor_apply_adc_target(motor_t *m)
{
    switch (m->mc.Motor.RunMode)
    {
        case CURRENT_CLOSE_LOOP:
            m->mc.IqPid.Ref = motor_clampf(m->mc.Sample.AdcBuff[1] * 0.002f,
                                           -motor_current_ref_limit(m),
                                           motor_current_ref_limit(m));
            m->mc.IdPid.Ref = 0.0f;
            break;

        case SPEED_CURRENT_LOOP:
            m->mc.Speed.MechanicalSpeedSet = m->speed_dir * m->mc.Sample.AdcBuff[1] * 0.5f;
            if (m->mc.Speed.MechanicalSpeedSet <= 5.0f && m->mc.Speed.MechanicalSpeedSet >= -5.0f)
            {
                m->mc.Speed.MechanicalSpeedSet = 0.0f;
            }
            break;

        case POS_SPEED_CURRENT_LOOP:
            m->position_target = motor_wrap_counts(-(int32_t)m->mc.Sample.AdcBuff[1], motor_counts_per_rev(m));
            m->mc.Position.MechanicalPosSet = m->position_target;
            break;

        default:
            break;
    }
}

static void motor_apply_set_target(motor_t *m)
{
    switch (m->mc.Motor.RunMode)
    {
        case CURRENT_CLOSE_LOOP:
            m->mc.IqPid.Ref = motor_clampf(m->current_target_amp,
                                           -motor_current_ref_limit(m),
                                           motor_current_ref_limit(m));
            m->mc.IdPid.Ref = 0.0f;
            break;

        case SPEED_CURRENT_LOOP:
            m->mc.Speed.MechanicalSpeedSet = m->speed_dir * m->speed_target_rpm;
            if (m->mc.Speed.MechanicalSpeedSet <= 5.0f && m->mc.Speed.MechanicalSpeedSet >= -5.0f)
            {
                m->mc.Speed.MechanicalSpeedSet = 0.0f;
            }
            break;

        case POS_SPEED_CURRENT_LOOP:
            m->mc.Position.MechanicalPosSet = m->position_target;
            break;

        default:
            break;
    }
}

static void motor_update_target(motor_t *m)
{
    if (m->use_adc_target != 0U)
    {
        motor_apply_adc_target(m);
    }
    else
    {
        motor_apply_set_target(m);
    }
}

static void motor_control_tick(motor_t *m)
{
    motor_update_target(m);
    motor_control(m, MOTOR_CONTROL_RUN);
}

/* ===================== Real-time control tick ===================== */

void motor_tick(motor_t *m, const motor_hw_inputs_t *in)
{
    /* 1. Raw hardware inputs (read once by the facade). */
    m->mc.Sample.IuRaw = in->iu_raw;
    m->mc.Sample.IwRaw = in->iw_raw;
    m->mc.Sample.BusRaw = in->bus_raw;
    m->mc.EAngle.EncoderVal = in->encoder_val;

    /* 2. All motor control decisions live in one place: target apply plus the
     *    FOC state machine. */
    motor_control_tick(m);
}
