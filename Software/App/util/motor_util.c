#include "motor_util.h"

/* ---------- Clamping ---------- */

float motor_util_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ---------- Bounded number parsing ---------- */

int motor_util_parse_int(const char *s, int32_t *out)
{
    const char *p = s;
    int neg = 0;
    int32_t v = 0;
    int digits = 0;

    if (p[0] == '+') { p++; }
    else if (p[0] == '-') { neg = 1; p++; }

    while (p[0] >= '0' && p[0] <= '9')
    {
        int32_t d = p[0] - '0';
        /* Overflow guard: INT32_MAX/10 == 214748364, last digit 7 */
        if (v > 214748364 || (v == 214748364 && d > 7))
        {
            v = (neg ? (int32_t)0x80000000 : 0x7FFFFFFF); /* saturate */
            while (p[0] >= '0' && p[0] <= '9') p++;        /* consume rest */
            *out = v;
            return (int)(p - s);
        }
        v = v * 10 + d;
        digits++;
        p++;
    }

    if (digits == 0) return 0;
    *out = neg ? -v : v;
    return (int)(p - s);
}

int motor_util_parse_float(const char *s, float *out)
{
    const char *p = s;
    int neg = 0;
    float v = 0.0f;
    float scale = 0.1f;
    int digits = 0;
    int frac = 0;

    if (p[0] == '+') { p++; }
    else if (p[0] == '-') { neg = 1; p++; }

    while (p[0] >= '0' && p[0] <= '9')
    {
        v = v * 10.0f + (float)(p[0] - '0');
        digits++;
        p++;
    }

    if (p[0] == '.')
    {
        p++;
        while (p[0] >= '0' && p[0] <= '9')
        {
            v += (float)(p[0] - '0') * scale;
            scale *= 0.1f;
            digits++;
            frac++;
            p++;
        }
    }

    if (digits == 0) return 0;
    (void)frac;
    *out = neg ? -v : v;
    return (int)(p - s);
}

/* ---------- SPSC ring buffer ---------- */

void motor_ring_init(motor_ring_t *r, uint8_t *buf, uint16_t size)
{
    r->buf = buf;
    r->size = size;
    r->mask = (uint16_t)(size - 1u);
    r->head = 0;
    r->tail = 0;
}

int motor_ring_push(motor_ring_t *r, uint8_t b)
{
    uint16_t head = r->head;
    uint16_t tail = r->tail;
    if ((uint16_t)(head - tail) >= r->size)
    {
        return 0; /* full */
    }
    r->buf[head & r->mask] = b;
    r->head = (uint16_t)(head + 1u);
    return 1;
}

int motor_ring_pop(motor_ring_t *r, uint8_t *b)
{
    uint16_t head = r->head;
    uint16_t tail = r->tail;
    if (head == tail)
    {
        return 0; /* empty */
    }
    *b = r->buf[tail & r->mask];
    r->tail = (uint16_t)(tail + 1u);
    return 1;
}
