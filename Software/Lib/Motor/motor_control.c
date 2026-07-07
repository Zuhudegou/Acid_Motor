#include "motor_control.h"

#include "Control/motor_system.h"
#include "Control/position_drv.h"

/*
 * Build the former MotorControl modules as one unit. They remain split into
 * small private files under Lib/Motor/Control for readability, while CMake and
 * outside code see this single motor_control.c/.h pair.
 */
#include "Control/math_drv.c"
#include "Control/pid_drv.c"
#include "Control/sample_drv.c"
#include "Control/eangle_drv.c"
#include "Control/foc_drv.c"
#include "Control/speed_drv.c"
#include "Control/position_drv.c"
#include "Control/motor_publicdata.c"
#include "Control/motor_identify.c"
#include "Control/motor_sensoruse.c"
#include "Control/motor_system.c"

void motor_control(struct motor_s *m, motor_control_cmd_t cmd)
{
    switch (cmd)
    {
        case MOTOR_CONTROL_INIT:
            Motor_Struct_Init(m);
            break;

        case MOTOR_CONTROL_HOME:
            Position_SetHome(m);
            break;

        case MOTOR_CONTROL_RUN:
            Motor_System_Run(m);
            break;

        default:
            break;
    }
}
