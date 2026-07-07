#ifndef MOTOR_INTERNAL_H
#define MOTOR_INTERNAL_H

/*
 * motor_internal.h — PRIVATE header. Defines the full layout of motor_t and the
 * single instance g_motor. Included ONLY by Lib/Motor source files that need
 * the private motor_t layout.
 * App/ code must include only motor.h / motor_hal.h.
 */

#include "motor.h"
#include "motor_control.h"   /* MOTORCONTROL_STRUCT + sub-struct defs */

#ifdef __cplusplus
extern "C" {
#endif

struct motor_s {
    MOTORCONTROL_STRUCT mc;          /* the former global MC, now an instance */
    motor_config_t      cfg;
    int8_t              speed_dir;
    float               speed_target_rpm;
    float               current_target_amp;
    int32_t             position_target;
    uint8_t             use_adc_target;
    uint8_t             output_enable;   /* library decision; facade applies to GPIO */
};

/* Single instance for this build. The API supports multiple motors; one is
 * wired here. The public declaration lives in motor.h (extern motor_t g_motor)
 * so application layers reach it as an opaque handle; the definition is in
 * motor.c. */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_INTERNAL_H */
