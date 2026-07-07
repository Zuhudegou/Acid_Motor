#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

/*
 * motor_hal.h — thin orchestration layer over the board port (board.h).
 * HAL-free: sequences lifecycle and provides the FOC ISR tick entry. All
 * hardware access is delegated to board_* functions.
 */

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Init the motor instance (g_motor) from the default config, zero PWM, and
 * disable the power stage. Replaces Motor_Driver_Init. */
void motor_hal_init(void);

/* Calibrate + start the FOC I/O peripherals and enable the power stage.
 * Returns 0 on success (nonzero on failure). MUST be the last init step (after
 * UI/command init) so the 20 kHz ISR does not run before the foreground layers
 * are ready. Replaces Motor_Driver_Start. */
int motor_hal_start(motor_t *m);

/* Force stop: MOTOR_STOP, zero duty, commit PWM, disable the power stage. */
void motor_hal_stop(motor_t *m);

/* Enable/disable the power stage and record the state in m->output_enable. */
void motor_hal_enable_output(motor_t *m, uint8_t enable);

/* Read the raw HW inputs for one tick into `in` (phase currents, bus, encoder).
 * Called by the FOC ISR before motor_tick. */
void motor_hal_read_inputs(motor_t *m, motor_hw_inputs_t *in);

/* Commit m->mc.Foc.DutyCycleA/B/C to the PWM peripheral, clamped to
 * cfg.pwm_period. */
void motor_hal_write_pwm(motor_t *m);

/* One FOC control tick: read inputs -> motor_tick -> write PWM. Called from the
 * board's ADC injected-complete ISR. */
void motor_hal_isr_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HAL_H */
