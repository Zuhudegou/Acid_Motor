#include "circular_buffer.h"

void CircularBuffer_Init(CircularBuffer_t *cbuf, uint8_t *buffer_ptr, uint32_t buffer_size)
{
    cbuf->buffer = buffer_ptr;
    cbuf->size = buffer_size;
    cbuf->write_index = 0;
    cbuf->read_index = 0;
}

uint32_t CircularBuffer_Write(CircularBuffer_t *cbuf, const uint8_t *data, uint32_t length)
{
    uint32_t free_space = CircularBuffer_Free(cbuf);
    uint32_t to_write = (length < free_space) ? length : free_space;

    if (to_write == 0)
        return 0;

    uint32_t write_idx = cbuf->write_index % cbuf->size;
    uint32_t space_until_end = cbuf->size - write_idx;

    if (to_write <= space_until_end) {

        memcpy(&cbuf->buffer[write_idx], data, to_write);
    } else {

        memcpy(&cbuf->buffer[write_idx], data, space_until_end);
        memcpy(cbuf->buffer, data + space_until_end, to_write - space_until_end);
    }

    cbuf->write_index += to_write;
    return to_write;
}

uint32_t CircularBuffer_Read(CircularBuffer_t *cbuf, uint8_t *data, uint32_t length)
{
    uint32_t available = CircularBuffer_Available(cbuf);
    uint32_t to_read = (length < available) ? length : available;

    if (to_read == 0)
        return 0;

    uint32_t read_idx = cbuf->read_index % cbuf->size;
    uint32_t space_until_end = cbuf->size - read_idx;

    if (to_read <= space_until_end) {

        memcpy(data, &cbuf->buffer[read_idx], to_read);
    } else {

        memcpy(data, &cbuf->buffer[read_idx], space_until_end);
        memcpy(data + space_until_end, cbuf->buffer, to_read - space_until_end);
    }

    cbuf->read_index += to_read;
    return to_read;
}

uint32_t CircularBuffer_Available(CircularBuffer_t *cbuf)
{
    return (cbuf->write_index - cbuf->read_index);
}

uint32_t CircularBuffer_Free(CircularBuffer_t *cbuf)
{
    return cbuf->size - CircularBuffer_Available(cbuf);
}

uint32_t CircularBuffer_IsEmpty(CircularBuffer_t *cbuf)
{
    return (cbuf->write_index == cbuf->read_index);
}

uint32_t CircularBuffer_IsFull(CircularBuffer_t *cbuf)
{
    return (CircularBuffer_Available(cbuf) == cbuf->size);
}

void CircularBuffer_Flush(CircularBuffer_t *cbuf)
{
    cbuf->write_index = 0;
    cbuf->read_index = 0;
}
