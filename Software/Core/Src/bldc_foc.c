#include "bldc_foc.h"
#include "uart_debug.h"
#include <string.h>

#define PWM_FREQ_HZ             40000U
#define DT_LOOP                 (1.0f / PWM_FREQ_HZ)  
#define INV_DT_LOOP             ((float)PWM_FREQ_HZ) 

#define SECTOR_COUNT            6
#define TWO_PI                  6.283185307f
#define ONE_OVER_SQRT3          0.577350269f
#define INV_TWO_PI              0.159154943f
#define SECTOR_ANGLE            (TWO_PI / SECTOR_COUNT)
#define INV_SECTOR_ANGLE        (SECTOR_COUNT / TWO_PI)
#define FOC_ISR_DEBUG_ENABLE    0

FOCControl_t g_foc_ctrl = {0};
CurrentSensor_t g_current_sensor = {0};

static uint16_t g_adc_offset_u = ADC_OFFSET_U_DEFAULT;
static uint16_t g_adc_offset_v = ADC_OFFSET_V_DEFAULT;

static void FOC_ReadADCValues(void);
static void FOC_ApplyOffsetCompensation(void);
static void FOC_ConvertADCToCurrents(void);
static void FOC_ExecuteClarkeTransform(void);
static void FOC_ExecuteParkTransform(void);
static void FOC_ExecuteCurrentControl(void);
static void FOC_ExecuteInverseParkTransform(void);
static void FOC_ExecuteSVPWM(void);
static void FOC_LimitAndUpdatePWM(void);
static inline float FOC_UpdatePIDController(PIDController_t *pid, float error, float dt, float inv_dt);
static inline void FOC_ResetPIDState(PIDController_t *pid);

void FOC_Init(void)
{
    memset(&g_foc_ctrl, 0, sizeof(g_foc_ctrl));
    memset(&g_current_sensor, 0, sizeof(g_current_sensor));

    g_foc_ctrl.pid_id.kp = ID_KP;
    g_foc_ctrl.pid_id.ki = ID_KI;
    g_foc_ctrl.pid_id.kd = ID_KD;
    g_foc_ctrl.pid_id.output_max = UD_MAX;

    g_foc_ctrl.pid_iq.kp = IQ_KP;
    g_foc_ctrl.pid_iq.ki = IQ_KI;
    g_foc_ctrl.pid_iq.kd = IQ_KD;
    g_foc_ctrl.pid_iq.output_max = UQ_MAX;

    g_foc_ctrl.pid_vel.kp = VEL_KP;
    g_foc_ctrl.pid_vel.ki = VEL_KI;
    g_foc_ctrl.pid_vel.kd = VEL_KD;
    g_foc_ctrl.pid_vel.output_max = IQ_REF_MAX;

    g_foc_ctrl.pid_pos.kp = POS_KP;
    g_foc_ctrl.pid_pos.ki = POS_KI;
    g_foc_ctrl.pid_pos.kd = POS_KD;
    g_foc_ctrl.pid_pos.output_max = VEL_REF_MAX;

    g_foc_ctrl.mode = FOC_MODE_TORQUE;
    g_foc_ctrl.enable = false;
    g_foc_ctrl.fault = false;
    g_foc_ctrl.loop_counter = 0;

    g_foc_ctrl.id_ref = 0.0f;
    g_foc_ctrl.iq_ref = 0.0f;
    g_foc_ctrl.theta_e = 0.0f;
}

void FOC_CurrentLoopCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;

    if (!g_foc_ctrl.enable) {

        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        return;
    }

    FOC_ReadADCValues();

    FOC_ApplyOffsetCompensation();

    FOC_ConvertADCToCurrents();

#if FOC_ISR_DEBUG_ENABLE
    if ((g_foc_ctrl.loop_counter % 100) == 0) {
        DebugUART_Printf("CUR: Iu=%.3f,Iv=%.3f,Iw=%.3f\r\n",
            g_current_sensor.i_u, g_current_sensor.i_v, g_current_sensor.i_w);
    }
#endif

    FOC_ExecuteClarkeTransform();

    FOC_ExecuteParkTransform();

#if FOC_ISR_DEBUG_ENABLE
    if ((g_foc_ctrl.loop_counter % 100) == 0) {
        DebugUART_Printf("DQ: Id=%.3f(ref=%.3f),Iq=%.3f(ref=%.3f)\r\n",
            g_foc_ctrl.i_dq_meas.d, g_foc_ctrl.i_dq_ref.d,
            g_foc_ctrl.i_dq_meas.q, g_foc_ctrl.i_dq_ref.q);
    }
#endif

    FOC_ExecuteCurrentControl();

#if FOC_ISR_DEBUG_ENABLE
    if ((g_foc_ctrl.loop_counter % 100) == 0) {
        DebugUART_Printf("VOL: Ud=%.3f,Uq=%.3f,theta=%.3f\r\n",
            g_foc_ctrl.u_dq.d, g_foc_ctrl.u_dq.q, g_foc_ctrl.theta_e);
    }
#endif

    FOC_ExecuteInverseParkTransform();

    FOC_ExecuteSVPWM();

    FOC_LimitAndUpdatePWM();

#if FOC_ISR_DEBUG_ENABLE
    if ((g_foc_ctrl.loop_counter % 500) == 0) {
        DebugUART_Printf("PWM: CCR3=%d,CCR2=%d,CCR1=%d,loop=%ld\r\n",
            g_foc_ctrl.duty_u, g_foc_ctrl.duty_v, g_foc_ctrl.duty_w, g_foc_ctrl.loop_counter);
    }
#endif

    g_foc_ctrl.loop_counter++;
}

void FOC_SetReferences(float id_ref, float target, float theta_e)
{
    g_foc_ctrl.id_ref = (id_ref > ID_REF_MAX) ? ID_REF_MAX :
                        (id_ref < -ID_REF_MAX) ? -ID_REF_MAX : id_ref;

    if (g_foc_ctrl.mode == FOC_MODE_TORQUE || g_foc_ctrl.mode == FOC_MODE_OPEN_LOOP) {
        g_foc_ctrl.iq_ref = (target > IQ_REF_MAX) ? IQ_REF_MAX :
                            (target < -IQ_REF_MAX) ? -IQ_REF_MAX : target;
    } else if (g_foc_ctrl.mode == FOC_MODE_VELOCITY) {
        g_foc_ctrl.vel_ref = (target > VEL_REF_MAX) ? VEL_REF_MAX :
                             (target < -VEL_REF_MAX) ? -VEL_REF_MAX : target;
    } else if (g_foc_ctrl.mode == FOC_MODE_POSITION || g_foc_ctrl.mode == FOC_MODE_HOMING) {
        g_foc_ctrl.pos_ref = target;
    }

    while (theta_e >= TWO_PI) theta_e -= TWO_PI;
    while (theta_e < 0) theta_e += TWO_PI;

    g_foc_ctrl.theta_e = theta_e;

    g_foc_ctrl.theta_e_angle = (uint16_t)(theta_e * INV_TWO_PI * ELECTRO_ANGLE_SCALE);
}

void FOC_SetEnable(bool enable)
{
    if (!enable) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        FOC_ResetPIDState(&g_foc_ctrl.pid_id);
        FOC_ResetPIDState(&g_foc_ctrl.pid_iq);
        FOC_ResetPIDState(&g_foc_ctrl.pid_vel);
        FOC_ResetPIDState(&g_foc_ctrl.pid_pos);
    }
    g_foc_ctrl.enable = enable;
}

const FOCControl_t* FOC_GetState(void)
{
    return &g_foc_ctrl;
}

static void FOC_ReadADCValues(void)
{
    g_current_sensor.adc_u_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    g_current_sensor.adc_v_raw = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
}

static void FOC_ApplyOffsetCompensation(void)
{

    g_current_sensor.adc_u_offset = (int16_t)((int32_t)g_current_sensor.adc_u_raw - (int32_t)g_adc_offset_u);
    g_current_sensor.adc_v_offset = (int16_t)((int32_t)g_current_sensor.adc_v_raw - (int32_t)g_adc_offset_v);
}

static void FOC_ConvertADCToCurrents(void)
{
    g_current_sensor.i_u = (float)g_current_sensor.adc_u_offset * ADC_TO_CURRENT_SCALE;
    g_current_sensor.i_v = (float)g_current_sensor.adc_v_offset * ADC_TO_CURRENT_SCALE;

    g_current_sensor.i_w = -(g_current_sensor.i_u + g_current_sensor.i_v);
}

static void FOC_ExecuteClarkeTransform(void)
{
    float i_a = g_current_sensor.i_u;
    float i_b = g_current_sensor.i_v;

    AlphaBeta_t i_ab;
    i_ab.alpha = i_a;
    i_ab.beta = (i_a + 2.0f * i_b) * ONE_OVER_SQRT3;

    g_current_sensor.i_u = i_ab.alpha;  
    g_current_sensor.i_v = i_ab.beta;    
}

static void FOC_ExecuteParkTransform(void)
{
    float theta = g_foc_ctrl.theta_e;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    float i_alpha = g_current_sensor.i_u;
    float i_beta = g_current_sensor.i_v;

    g_foc_ctrl.i_dq_meas.d = i_alpha * cos_theta + i_beta * sin_theta;
    g_foc_ctrl.i_dq_meas.q = -i_alpha * sin_theta + i_beta * cos_theta;

    g_foc_ctrl.i_dq_ref.d = g_foc_ctrl.id_ref;
    g_foc_ctrl.i_dq_ref.q = g_foc_ctrl.iq_ref;

    g_foc_ctrl.i_dq_error.d = g_foc_ctrl.i_dq_ref.d - g_foc_ctrl.i_dq_meas.d;
    g_foc_ctrl.i_dq_error.q = g_foc_ctrl.i_dq_ref.q - g_foc_ctrl.i_dq_meas.q;
}

static void FOC_ExecuteCurrentControl(void)
{
    g_foc_ctrl.u_dq.d = FOC_UpdatePIDController(
        &g_foc_ctrl.pid_id,
        g_foc_ctrl.i_dq_error.d,
        DT_LOOP,
        INV_DT_LOOP
    );

    g_foc_ctrl.u_dq.q = FOC_UpdatePIDController(
        &g_foc_ctrl.pid_iq,
        g_foc_ctrl.i_dq_error.q,
        DT_LOOP,
        INV_DT_LOOP
    );
}

static void FOC_ExecuteInverseParkTransform(void)
{
    float theta = g_foc_ctrl.theta_e;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    float u_d = g_foc_ctrl.u_dq.d;
    float u_q = g_foc_ctrl.u_dq.q;

    g_foc_ctrl.u_ab.alpha = u_d * cos_theta - u_q * sin_theta;
    g_foc_ctrl.u_ab.beta = u_d * sin_theta + u_q * cos_theta;
}

static void FOC_ExecuteSVPWM(void)
{
    float u_alpha = g_foc_ctrl.u_ab.alpha;
    float u_beta = g_foc_ctrl.u_ab.beta;

    float magnitude = sqrtf(u_alpha * u_alpha + u_beta * u_beta);
    float angle = atan2f(u_beta, u_alpha);

    if (angle < 0.0f) angle += TWO_PI;

    uint32_t sector = (uint32_t)(angle * INV_SECTOR_ANGLE);
    if (sector >= SECTOR_COUNT) sector = SECTOR_COUNT - 1;

    float angle_sector = angle - ((float)sector * SECTOR_ANGLE);

    float Ts = 1.0f; 
    float T1 = (SQRT3 * Ts * 0.5f) * magnitude * sinf(SECTOR_ANGLE - angle_sector);
    float T2 = (SQRT3 * Ts * 0.5f) * magnitude * sinf(angle_sector);
    float T0 = Ts - T1 - T2;

    if (T0 < 0.0f) {
        T1 = Ts * magnitude * ONE_OVER_SQRT3;
        T2 = T1;
        T0 = 0.0f;
    }

    float T0_half = T0 * 0.5f;

    SVPWM_Duty_t duty;

    switch (sector) {
        case 0: 
            duty.ua = T0_half + T1 + T2;
            duty.ub = T0_half + T2;
            duty.uc = T0_half;
            break;
        case 1:  
            duty.ua = T0_half + T1;
            duty.ub = T0_half + T1 + T2;
            duty.uc = T0_half;
            break;
        case 2:  
            duty.ua = T0_half;
            duty.ub = T0_half + T1 + T2;
            duty.uc = T0_half + T2;
            break;
        case 3: 
            duty.ua = T0_half;
            duty.ub = T0_half + T1;
            duty.uc = T0_half + T1 + T2;
            break;
        case 4:  
            duty.ua = T0_half + T2;
            duty.ub = T0_half;
            duty.uc = T0_half + T1 + T2;
            break;
        case 5:  
            duty.ua = T0_half + T1 + T2;
            duty.ub = T0_half;
            duty.uc = T0_half + T1;
            break;
        default:
            duty.ua = 0.5f;
            duty.ub = 0.5f;
            duty.uc = 0.5f;
            break;
    }

    g_foc_ctrl.duty_u = (uint16_t)(duty.ua * PWM_PERIOD);  
    g_foc_ctrl.duty_v = (uint16_t)(duty.ub * PWM_PERIOD); 
    g_foc_ctrl.duty_w = (uint16_t)(duty.uc * PWM_PERIOD); 
}

static void FOC_LimitAndUpdatePWM(void)
{

    if (g_foc_ctrl.duty_u < DUTY_MIN) {
        g_foc_ctrl.duty_u = DUTY_MIN;
    } else if (g_foc_ctrl.duty_u > DUTY_MAX) {
        g_foc_ctrl.duty_u = DUTY_MAX;
    }

    if (g_foc_ctrl.duty_v < DUTY_MIN) {
        g_foc_ctrl.duty_v = DUTY_MIN;
    } else if (g_foc_ctrl.duty_v > DUTY_MAX) {
        g_foc_ctrl.duty_v = DUTY_MAX;
    }

    if (g_foc_ctrl.duty_w < DUTY_MIN) {
        g_foc_ctrl.duty_w = DUTY_MIN;
    } else if (g_foc_ctrl.duty_w > DUTY_MAX) {
        g_foc_ctrl.duty_w = DUTY_MAX;
    }

    TIM1->CCR3 = g_foc_ctrl.duty_u;  
    TIM1->CCR2 = g_foc_ctrl.duty_v;  
    TIM1->CCR1 = g_foc_ctrl.duty_w;  

    __ISB();
}

static inline float FOC_UpdatePIDController(PIDController_t *pid, float error, float dt, float inv_dt)
{
    float p_term = pid->kp * error;

    float i_term_delta = pid->ki * dt * error;

    float d_term = pid->kd * (error - pid->error_prev) * inv_dt;
    pid->error_prev = error;

    float output = p_term + pid->integral + d_term + i_term_delta;

    if (output > pid->output_max) {
        output = pid->output_max;
        if (error < 0.0f) {
            pid->integral += i_term_delta;
        }
    } else if (output < -pid->output_max) {
        output = -pid->output_max;
        if (error > 0.0f) {
            pid->integral += i_term_delta;
        }
    } else {
        pid->integral += i_term_delta;
    }

    return output;
}

static inline void FOC_ResetPIDState(PIDController_t *pid)
{
    pid->error = 0.0f;
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

void FOC_CascadeLoopCallback(void)
{
    if (!g_foc_ctrl.enable) {
        return;
    }

    const float dt_cascade = 0.001f;
    const float inv_dt_cascade = 1000.0f;

    if (g_foc_ctrl.mode == FOC_MODE_POSITION || g_foc_ctrl.mode == FOC_MODE_HOMING) {
        float pos_error = g_foc_ctrl.pos_ref - g_foc_ctrl.theta_m;

        g_foc_ctrl.vel_ref = FOC_UpdatePIDController(
            &g_foc_ctrl.pid_pos, 
            pos_error, 
            dt_cascade, 
            inv_dt_cascade
        );

        if (g_foc_ctrl.vel_ref > VEL_REF_MAX) g_foc_ctrl.vel_ref = VEL_REF_MAX;
        if (g_foc_ctrl.vel_ref < -VEL_REF_MAX) g_foc_ctrl.vel_ref = -VEL_REF_MAX;
    }

    if (g_foc_ctrl.mode == FOC_MODE_VELOCITY || g_foc_ctrl.mode == FOC_MODE_POSITION || g_foc_ctrl.mode == FOC_MODE_HOMING) {
        float vel_error = g_foc_ctrl.vel_ref - g_foc_ctrl.velocity_m;

        g_foc_ctrl.iq_ref = FOC_UpdatePIDController(
            &g_foc_ctrl.pid_vel, 
            vel_error, 
            dt_cascade, 
            inv_dt_cascade
        );

        if (g_foc_ctrl.iq_ref > IQ_REF_MAX) g_foc_ctrl.iq_ref = IQ_REF_MAX;
        if (g_foc_ctrl.iq_ref < -IQ_REF_MAX) g_foc_ctrl.iq_ref = -IQ_REF_MAX;
    }
}

void FOC_SetControlMode(FOC_ControlMode_t mode)
{
    if (mode == FOC_MODE_POSITION || mode == FOC_MODE_HOMING) {
        g_foc_ctrl.pos_ref = g_foc_ctrl.theta_m;
        g_foc_ctrl.vel_ref = 0.0f;
    } else if (mode == FOC_MODE_VELOCITY) {
        g_foc_ctrl.vel_ref = g_foc_ctrl.velocity_m;
    }

    g_foc_ctrl.mode = mode;
}

void FOC_UpdateFeedback(float mech_angle, float mech_vel)
{
    g_foc_ctrl.theta_m = mech_angle;
    g_foc_ctrl.velocity_m = mech_vel;

    float theta_e = fmodf(mech_angle * POLE_PAIRS, TWO_PI);
    if (theta_e < 0.0f) theta_e += TWO_PI;
    g_foc_ctrl.theta_e = theta_e;

    g_foc_ctrl.theta_e_angle = (uint16_t)(theta_e * INV_TWO_PI * ELECTRO_ANGLE_SCALE);
}
