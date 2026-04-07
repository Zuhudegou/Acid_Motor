#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include <stdint.h>
#include <stdio.h>
#include "circular_buffer.h"
#include "stm32g4xx_hal.h"

typedef enum {
    DEBUG_FOC_STATE = 0x01,      
    DEBUG_CURRENT_VALUES = 0x02, 
    DEBUG_VOLTAGE_VALUES = 0x03, 
    DEBUG_ROTOR_STATE = 0x04,    
    DEBUG_PID_DEBUG = 0x05,      
    DEBUG_ERROR = 0xFF
} DebugMessageType_t;

typedef struct {
    uint8_t type;          
    uint8_t reserved;
    union {
        struct {
            float Ia;      
            float Ib;      
        } currents;
        struct {
            float theta;   
            float omega;   
        } rotor;
        struct {
            float Ud;      
            float Uq;      
        } voltages;
    } data;
} DebugMessage_t;

void DebugUART_Init(UART_HandleTypeDef *huart);

uint32_t DebugUART_WriteMessage(uint8_t type, void *data);

uint32_t DebugUART_Printf(const char *format, ...);

void DebugUART_Process(void);

uint32_t DebugUART_GetFreeSpace(void);

uint32_t DebugUART_GetAvailable(void);

void DebugUART_Flush(void);

uint32_t DebugUART_IsBusy(void);

#endif
