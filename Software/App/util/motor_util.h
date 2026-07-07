#ifndef MOTOR_UTIL_H
#define MOTOR_UTIL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Clamping ---------- */
float    motor_util_clampf(float v, float lo, float hi);

/* ---------- Bounded number parsing ----------
 * Parse a number from the start of s. Stop at the first non-numeric character.
 * On success return the number of characters consumed (> 0) and write *out.
 * On failure (no digits) return 0 and leave *out unchanged. Overflow-safe. */
int motor_util_parse_int(const char *s, int32_t *out);
int motor_util_parse_float(const char *s, float *out);

/* ---------- SPSC ring buffer ----------
 * Single-producer / single-consumer ring for ISR->foreground byte streaming
 * (e.g. UART RX). `size` must be a power of two and <= 32768.
 * Producer (ISR) calls motor_ring_push; consumer (foreground) calls
 * motor_ring_pop. head/tail are free-running u16 counters; the index into buf
 * is counter & mask. No disabling of interrupts is required for correctness. */
typedef struct {
    uint8_t         *buf;
    uint16_t         size;   /* power of two */
    uint16_t         mask;   /* size - 1 */
    volatile uint16_t head;  /* producer index (ISR) */
    volatile uint16_t tail;  /* consumer index (foreground) */
} motor_ring_t;

void     motor_ring_init(motor_ring_t *r, uint8_t *buf, uint16_t size);
int      motor_ring_push(motor_ring_t *r, uint8_t b);   /* ISR: 1 ok, 0 full */
int      motor_ring_pop(motor_ring_t *r, uint8_t *b);   /* fg: 1 ok, 0 empty */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_UTIL_H */
