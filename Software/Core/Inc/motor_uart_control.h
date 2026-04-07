#ifndef __MOTOR_UART_CONTROL_H__
#define __MOTOR_UART_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "usart.h"

#define UART_PACKET_SIZE        7U          
#define UART_PACKET_HEADER      0xAAU       
#define UART_RX_BUFFER_SIZE     64U         

typedef enum {
    MOTOR_CMD_DISABLE = 0x00,              
    MOTOR_CMD_ENABLE_TORQUE  = 0x01,       
    MOTOR_CMD_ENABLE_VELOCITY = 0x02,      
    MOTOR_CMD_ENABLE_POSITION = 0x03,      
    MOTOR_CMD_ENABLE_HOMING   = 0x04,      
} MotorCommand_t;

typedef struct {
    uint8_t   header;                      
    uint8_t   cmd;                         
    int16_t   iq_ref;                      
    uint16_t  theta_e;                     
    uint8_t   crc;                         
} MotorUartPacket_t;

typedef enum {
    UART_STATE_WAIT_HEADER,                
    UART_STATE_COLLECTING,                 
    UART_STATE_COMPLETE,                   
} UartRxState_t;

typedef struct {
    uint8_t         rx_byte;                        

    MotorUartPacket_t current_packet;              
    MotorUartPacket_t pending_packet;              

    uint8_t         byte_count;                    
    UartRxState_t   state;                        

    uint32_t        valid_packets;                 
    uint32_t        crc_errors;                    
    uint32_t        timeout_errors;                
} UartControlBlock_t;

void MotorUart_Init(void);

void MotorUart_RxCpltCallback(UART_HandleTypeDef *huart);

bool MotorUart_ProcessPacket(void);

void MotorUart_ProcessForward(void);

void MotorUart_SendTelemetry(void);

const UartControlBlock_t* MotorUart_GetStats(void);

void MotorUart_SendPacket(uint8_t cmd, float iq_ref, float theta_e);

#ifdef __cplusplus
}
#endif

#endif
