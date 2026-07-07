#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/*
 * motor_control.h - public bridge for the FOC control pipeline internals.
 *
 * The detailed control structs and mode enums still live in Control/ for now,
 * but the rest of the project depends on this single header.
 */

#include "Control/motor_publicdata.h"

#ifdef __cplusplus
extern "C" {
#endif

struct motor_s;

typedef enum {
    MOTOR_CONTROL_INIT = 0,
    MOTOR_CONTROL_HOME,
    MOTOR_CONTROL_RUN
} motor_control_cmd_t;

void motor_control(struct motor_s *m, motor_control_cmd_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
