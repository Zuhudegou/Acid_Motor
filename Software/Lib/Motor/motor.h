#ifndef MOTOR_H
#define MOTOR_H

/*
 * motor.h — public API of the encapsulated FOC motor control library.
 *
 * The instance handle motor_t is opaque: callers depend only on the functions
 * below, not on the internal layout (defined in motor_internal.h, visible only
 * to Lib/Motor source files). The library operates on a motor_t* instance; this build
 * wires a single instance g_motor, but the API supports multiple.
 *
 * NOTE (transitional): this header includes motor_control.h to reuse the existing
 * MOTOR_RUN_STATE / MOTOR_ERROR_CODE types and the RunMode #define values. A
 * later stage can extract clean motor_state_t / motor_mode_t enums here.
 */

#include <stdint.h>
#include <stddef.h>
#include "motor_control.h"  /* MOTOR_RUN_STATE, MOTOR_ERROR_CODE, RunMode #defines */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motor_s motor_t;   /* opaque instance handle */

/* Hardware-derived configuration. Single source of truth for values that used
 * to be scattered macros. pwm_period is the TIM1 ARR (the sole truth); the FOC
 * SVPWM PwmCycle is derived as 2*pwm_period (center-aligned). */
typedef struct {
    uint8_t  pole_pairs;
    uint16_t encoder_lines;
    uint16_t pwm_period;
    uint16_t pwm_limit;
    float    ts;
    float    current_scale;
    float    bus_scale;
    float    over_current;
    float    bus_min;
    float    bus_max;
    uint16_t speed_div;
    uint16_t pos_div;
    float    acceleration;
    /* Position loop integral gain. Default 0 preserves the original pure-P
     * behavior (which leaves a steady-state position error under load). Set
     * nonzero on the bench to trim out the static error; the position PID's
     * conditional-integration anti-windup bounds the integrator to OutMax/OutMin. */
    float    pos_ki;
} motor_config_t;

/* Raw hardware inputs filled by the HAL facade each tick (read once). */
typedef struct {
    int32_t  iu_raw;            /* ADC2 injected rank 1 (PA6) */
    int32_t  iw_raw;            /* ADC2 injected rank 2 (PA4) */
    uint16_t bus_raw;           /* ADC2 regular DMA slot 0 (PC5) */
    uint16_t encoder_val;       /* TIM3 counter */
} motor_hw_inputs_t;

/* Telemetry snapshot — replaces direct MC.* reads from UI/command. */
typedef struct {
    MOTOR_RUN_STATE   state;
    uint8_t           error_code;   /* MOTOR_ERROR_CODE */
    uint8_t           run_mode;     /* RunMode #define */
    float             bus_real;
    float             iu_real, iv_real, iw_real;
    float             iq;
    float             current_set;
    float             speed_fb;
    float             speed_set;
    int32_t           encoder;
    int32_t           position_raw;
    int32_t           position_target;
    float             position_angle_deg;
    float             position_target_deg;
    uint8_t           use_adc_target;
    int8_t            speed_dir;
} motor_telemetry_t;

/* ---------- Lifecycle ---------- */
void     motor_init(motor_t *m, const motor_config_t *cfg);
void     motor_start(motor_t *m);          /* set run state to ADC_CALIB */
void     motor_stop(motor_t *m);           /* set MOTOR_STOP + zero duty */
void     motor_clear_error(motor_t *m);    /* clear error, return to ADC_CALIB if was error */
void     motor_enter_mode(motor_t *m, MOTOR_RUN_STATE state, uint8_t mode);
void     motor_home_position(motor_t *m);
uint8_t  motor_output_enabled(const motor_t *m);
uint8_t  motor_sensored_ready(const motor_t *m);

/* ---------- Setters (foreground) ---------- */
void     motor_set_state(motor_t *m, MOTOR_RUN_STATE s);
void     motor_set_mode(motor_t *m, uint8_t mode);
void     motor_set_target_speed(motor_t *m, float rpm);
void     motor_set_target_current(motor_t *m, float amp);
void     motor_set_target_position(motor_t *m, int32_t pos);
void     motor_set_target_angle(motor_t *m, float degrees);
void     motor_use_adc_target(motor_t *m, uint8_t enable);
void     motor_set_speed_dir(motor_t *m, int8_t dir);

/* ---------- Getters / telemetry ---------- */
void              motor_get_telemetry(const motor_t *m, motor_telemetry_t *t);
MOTOR_RUN_STATE   motor_get_state(const motor_t *m);
uint8_t           motor_get_mode(const motor_t *m);
uint8_t           motor_is_adc_target(const motor_t *m);
float             motor_get_target_speed(const motor_t *m);
float             motor_get_target_current(const motor_t *m);
int32_t           motor_get_target_position(const motor_t *m);
float             motor_get_target_angle(const motor_t *m);
int8_t            motor_get_speed_dir(const motor_t *m);

/* ---------- ISR control entry ----------
 * Reads raw HW inputs into the instance, applies the foreground target, runs the
 * FOC pipeline, and leaves the phase duty cycles in m->mc.Foc.DutyCycleA/B/C.
 * The PWM commit to TIM1 stays in the facade (motor_hal_write_pwm), called by
 * the ISR after this. */
void     motor_tick(motor_t *m, const motor_hw_inputs_t *in);

/* ---------- Single-instance handle ----------
 * This build wires one motor. The instance is declared here (public) so the
 * application layers (UI/command/main) reach it as an opaque handle without
 * depending on the private layout in motor_internal.h. The API itself supports
 * multiple instances. */
extern motor_t g_motor;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
