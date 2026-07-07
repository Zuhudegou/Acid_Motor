#ifndef MOTOR_UI_H
#define MOTOR_UI_H

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

void Motor_UI_Init(void);
void Motor_UI_Task(void);
void Motor_UI_ForceRefresh(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_UI_H */
