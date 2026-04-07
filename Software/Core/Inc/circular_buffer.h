#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    volatile uint32_t write_index;
    volatile uint32_t read_index;
} CircularBuffer_t;

void CircularBuffer_Init(CircularBuffer_t *cbuf, uint8_t *buffer_ptr, uint32_t buffer_size);

uint32_t CircularBuffer_Write(CircularBuffer_t *cbuf, const uint8_t *data, uint32_t length);

uint32_t CircularBuffer_Read(CircularBuffer_t *cbuf, uint8_t *data, uint32_t length);

uint32_t CircularBuffer_Available(CircularBuffer_t *cbuf);

uint32_t CircularBuffer_Free(CircularBuffer_t *cbuf);

uint32_t CircularBuffer_IsEmpty(CircularBuffer_t *cbuf);

uint32_t CircularBuffer_IsFull(CircularBuffer_t *cbuf);

void CircularBuffer_Flush(CircularBuffer_t *cbuf);

#endif
