#include "motor_board.h"

#define MOTOR_BOARD_LOW_RESISTOR       (4.7f)
#define MOTOR_BOARD_HIGH_RESISTOR      (100.0f)
#define MOTOR_BOARD_SAMPLING_RESISTOR  (0.005f)
#define MOTOR_BOARD_MAGNIFICATION      (10.0f)
#define MOTOR_BOARD_ADC_RESOLUTION     (4096.0f)
#define MOTOR_BOARD_ADC_VREF           (3.3f)

#define MOTOR_BOARD_PWM_PERIOD         (4250U)
#define MOTOR_BOARD_PWM_LIMIT          (7800U)
#define MOTOR_BOARD_CONTROL_TS         (0.00005f)

#define MOTOR_BOARD_OVER_CURRENT       (12.0f)
#define MOTOR_BOARD_BUS_VOLTAGE_MIN    (10.0f)
#define MOTOR_BOARD_BUS_VOLTAGE_MAX    (40.0f)

#define MOTOR_BOARD_POLE_PAIRS         (7U)
#define MOTOR_BOARD_ENCODER_LINES      (1024U)
#define MOTOR_BOARD_ACCELERATION       (4800.0f)
#define MOTOR_BOARD_SPEED_DIV          (2U)
#define MOTOR_BOARD_POS_DIV            (4U)

/* Position loop integral gain. Default 0 keeps the original pure-P behavior
 * (a real steady-state position error under load). Raise this on the bench to
 * trim the static error; the position PID's anti-windup bounds the integrator. */
#define MOTOR_BOARD_POS_KI             (0.0f)

#define MOTOR_BOARD_VBUS_FACTOR \
    ((MOTOR_BOARD_ADC_VREF / MOTOR_BOARD_ADC_RESOLUTION) * \
     (MOTOR_BOARD_LOW_RESISTOR + MOTOR_BOARD_HIGH_RESISTOR) / MOTOR_BOARD_LOW_RESISTOR)

#define MOTOR_BOARD_PHASE_CURRENT_FACTOR \
    ((MOTOR_BOARD_ADC_VREF / MOTOR_BOARD_ADC_RESOLUTION) / \
     (MOTOR_BOARD_MAGNIFICATION * MOTOR_BOARD_SAMPLING_RESISTOR))

static const motor_config_t g_motor_board_config = {
    .pole_pairs = MOTOR_BOARD_POLE_PAIRS,
    .encoder_lines = MOTOR_BOARD_ENCODER_LINES,
    .pwm_period = MOTOR_BOARD_PWM_PERIOD,
    .pwm_limit = MOTOR_BOARD_PWM_LIMIT,
    .ts = MOTOR_BOARD_CONTROL_TS,
    .current_scale = MOTOR_BOARD_PHASE_CURRENT_FACTOR,
    .bus_scale = MOTOR_BOARD_VBUS_FACTOR,
    .over_current = MOTOR_BOARD_OVER_CURRENT,
    .bus_min = MOTOR_BOARD_BUS_VOLTAGE_MIN,
    .bus_max = MOTOR_BOARD_BUS_VOLTAGE_MAX,
    .speed_div = MOTOR_BOARD_SPEED_DIV,
    .pos_div = MOTOR_BOARD_POS_DIV,
    .acceleration = MOTOR_BOARD_ACCELERATION,
    .pos_ki = MOTOR_BOARD_POS_KI,
};

const motor_config_t *motor_board_default_config(void)
{
    return &g_motor_board_config;
}
