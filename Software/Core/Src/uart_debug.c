#include "uart_debug.h"
#include <stdarg.h>
#include <string.h>

#define DEBUG_FIFO_SIZE 2048
static uint8_t debug_fifo_buffer[DEBUG_FIFO_SIZE];
static CircularBuffer_t debug_fifo;
static UART_HandleTypeDef *debug_huart = NULL;

#define DEBUG_TX_BUFFER_SIZE 256
static uint8_t debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];

void DebugUART_Init(UART_HandleTypeDef *huart)
{
    debug_huart = huart;
    CircularBuffer_Init(&debug_fifo, debug_fifo_buffer, DEBUG_FIFO_SIZE);
}

uint32_t DebugUART_WriteMessage(uint8_t type, void *data)
{
    if (debug_huart == NULL)
        return 0;

    DebugMessage_t msg;
    msg.type = type;
    msg.reserved = 0;

    if (data != NULL) {
        memcpy(&msg.data, data, sizeof(msg.data));
    }

    return CircularBuffer_Write(&debug_fifo, (uint8_t *)&msg, sizeof(DebugMessage_t));
}

uint32_t DebugUART_Printf(const char *format, ...)
{
    if (debug_huart == NULL)
        return 0;

    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0)
        return 0;

    uint32_t write_len = (len >= (int)sizeof(buffer)) ? (sizeof(buffer) - 1U) : (uint32_t)len;

    return CircularBuffer_Write(&debug_fifo, (uint8_t *)buffer, write_len);
}

void DebugUART_Process(void)
{
    if (debug_huart == NULL)
        return;

    if (debug_huart->gState != HAL_UART_STATE_READY)
        return;

    uint32_t available = CircularBuffer_Available(&debug_fifo);
    if (available == 0)
        return;

    uint32_t to_send = (available > DEBUG_TX_BUFFER_SIZE) ? DEBUG_TX_BUFFER_SIZE : available;
    uint32_t read_len = CircularBuffer_Read(&debug_fifo, debug_tx_buffer, to_send);

    if (read_len > 0) {
        (void)HAL_UART_Transmit_IT(debug_huart, debug_tx_buffer, (uint16_t)read_len);
    }
}

uint32_t DebugUART_GetFreeSpace(void)
{
    return CircularBuffer_Free(&debug_fifo);
}

uint32_t DebugUART_GetAvailable(void)
{
    return CircularBuffer_Available(&debug_fifo);
}

void DebugUART_Flush(void)
{
    CircularBuffer_Flush(&debug_fifo);
}

uint32_t DebugUART_IsBusy(void)
{
    if (debug_huart == NULL)
        return 0;
    return (debug_huart->gState != HAL_UART_STATE_READY);
}
