/*
 * motor_hal.c — thin HAL-free orchestration layer between the motor library
 * and the board port (board.h). It sequences lifecycle (init/start/stop),
 * applies the library's output-enable / duty-clamp policy, and provides the
 * per-tick FOC ISR entry. All actual hardware access goes through board_*;
 * this file no longer references the STM32 HAL or main.h.
 */

#include "motor_hal.h"
#include "motor_internal.h"   /* g_motor, struct motor_s */
#include "motor_board.h"
#include "board.h"

static uint16_t motor_hal_clamp_compare(uint16_t compare)
{
    /* Clamp to the PWM period (single source of truth = cfg.pwm_period). */
    if (compare > g_motor.cfg.pwm_period)
    {
        return g_motor.cfg.pwm_period;
    }
    return compare;
}

/* ===================== Lifecycle ===================== */

void motor_hal_init(void)
{
    motor_init(&g_motor, motor_board_default_config());
    motor_hal_write_pwm(&g_motor);     /* commit zero duty */
    board_motor_powerstage(0U);        /* power stage off until start */
}

int motor_hal_start(motor_t *m)
{
    board_delay_ms(100U);

    if (board_motor_adc_calibrate() != 0)
    {
        return -1;
    }

    board_delay_ms(10U);

    if (board_motor_start_io(m->mc.Sample.AdcBuff, 3U) != 0)
    {
        return -1;
    }

    board_motor_powerstage(1U);

    return 0;
}

void motor_hal_stop(motor_t *m)
{
    motor_stop(m);
    motor_hal_write_pwm(m);
    motor_hal_enable_output(m, 0U);
}

void motor_hal_enable_output(motor_t *m, uint8_t enable)
{
    m->output_enable = enable;
    board_motor_powerstage(enable);
}

/* ===================== Per-tick HW access ===================== */

void motor_hal_read_inputs(motor_t *m, motor_hw_inputs_t *in)
{
    board_motor_read_phase(&in->iu_raw, &in->iw_raw);
    in->bus_raw     = m->mc.Sample.AdcBuff[0];
    in->encoder_val = board_motor_read_encoder();
}

void motor_hal_write_pwm(motor_t *m)
{
    MOTOR_RUN_STATE state = motor_get_state(m);

    if ((state != ADC_CALIB && state != MOTOR_IDENTIFY && motor_output_enabled(m) == 0U) ||
        state == MOTOR_STOP || state == MOTOR_ERROR)
    {
        board_motor_write_pwm(0U, 0U, 0U);
        return;
    }

    board_motor_write_pwm(motor_hal_clamp_compare(m->mc.Foc.DutyCycleA),
                          motor_hal_clamp_compare(m->mc.Foc.DutyCycleB),
                          motor_hal_clamp_compare(m->mc.Foc.DutyCycleC));
}

/* ===================== FOC control tick =====================
 * Called from the board ADC injected-complete ISR (board_stm32g431.c) once per
 * control period: read raw inputs, run the library tick, commit duty cycles. */
void motor_hal_isr_tick(void)
{
    motor_hw_inputs_t in;
    motor_hal_read_inputs(&g_motor, &in);   /* fills encoder ONCE */
    motor_tick(&g_motor, &in);              /* library: target + FOC pipeline */
    motor_hal_write_pwm(&g_motor);          /* commit duty via board */
}
