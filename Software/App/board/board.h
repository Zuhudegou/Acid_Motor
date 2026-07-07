#ifndef BOARD_H
#define BOARD_H

/*
 * board.h — board port layer (HAL-free interface).
 *
 * Every hardware operation the firmware needs is declared here as a board_*
 * function. The ONLY implementation that touches the STM32 HAL is
 * board_stm32g431.c. The motor library (Lib/Motor) and the facades
 * (motor_hal / lcd_drv / motor_ui / motor_command) call these and never
 * reference HAL handles, pins, or main.h directly.
 *
 * PORTING CONTRACT: to bring this firmware up on a different board or MCU,
 * provide a new board_<target>.c implementing every function below, and edit
 * the pin/handle map in board_config.h. Nothing else needs to change.
 *
 * These are plain link-time functions (not a function-pointer table): the FOC
 * I/O runs in the 20 kHz ADC ISR, so we keep the call path direct (zero
 * indirection) and let the linker bind the board implementation.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== Motor power stage + FOC I/O =====================
 * The read/write/encoder functions are on the real-time hot path (called from
 * the FOC ISR every control tick). */

/* Calibrate the phase-current ADC. Returns 0 on success, nonzero on failure. */
int      board_motor_adc_calibrate(void);

/* Start the FOC I/O peripherals: bus-voltage ADC DMA into adc_dma_buf[n], the
 * encoder timer, the PWM timer + its update interrupt, and the FOC timebase.
 * Returns 0 on success, nonzero on failure. */
int      board_motor_start_io(uint16_t *adc_dma_buf, uint16_t n);

/* Enable (1) or disable (0) the half-bridge gate-driver power stage. */
void     board_motor_powerstage(uint8_t enable);

/* Read the two measured phase currents (raw ADC counts) for this tick. */
void     board_motor_read_phase(int32_t *iu, int32_t *iw);

/* Read the encoder counter. */
uint16_t board_motor_read_encoder(void);

/* Commit the three phase PWM compare values (already clamped by the caller). */
void     board_motor_write_pwm(uint16_t a, uint16_t b, uint16_t c);

/* ===================== LCD (ST7735 over SPI) ===================== */
void     board_lcd_reset(uint8_t level);          /* RES pin: 0=low, 1=high */
void     board_lcd_dc(uint8_t level);             /* DC pin:  0=command, 1=data */
void     board_lcd_cs(uint8_t level);             /* CS pin:  0=select, 1=deselect */
void     board_lcd_spi_write(const uint8_t *data, uint16_t len);

/* ===================== Buttons / time base ===================== */
uint8_t  board_key_next_pressed(void);            /* 1 if NEXT key is pressed */
uint8_t  board_key_enter_pressed(void);           /* 1 if ENTER key is pressed */
uint32_t board_millis(void);                      /* monotonic ms tick */
void     board_delay_ms(uint32_t ms);             /* blocking delay */

/* ===================== Command UART ===================== */
void     board_uart_tx(const uint8_t *data, uint16_t len);  /* blocking TX */
void     board_uart_rx_arm(void);                 /* arm one-byte RX interrupt */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
