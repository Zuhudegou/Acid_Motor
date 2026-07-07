#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Command handler: return non-zero to send "OK" reply, 0 on error. */
typedef uint8_t (*cmd_handler_t)(motor_t *m, const char *arg);

/* A single entry in the command dispatch table. */
typedef struct {
    const char *name;       /* uppercase command name, e.g. "RUN" */
    cmd_handler_t handler;  /* function to call */
    const char *help;       /* short help string */
} cmd_entry_t;

void Motor_Command_Init(void);
void Motor_Command_Task(void);

/* Called from the board UART RX ISR for each received byte. */
void Motor_Command_OnRxByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_COMMAND_H */
