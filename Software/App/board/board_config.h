#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * board_config.h — pin / peripheral-handle map for THIS board
 * (STM32G431RBT6 FOC controller).
 *
 * This is the single place that names physical pins and CubeMX peripheral
 * handles. To retarget a different board of the same MCU family, edit only the
 * defines here; to retarget a different MCU, replace board_stm32g431.c (which
 * is the only translation unit that includes this header and main.h).
 *
 * NOTE: this header pulls in main.h (CubeMX) for the HAL handle externs and
 * GPIO/pin types. It must only be included by the board implementation TU,
 * never by the HAL-free facades or the library.
 */

#include "main.h"   /* HAL handles (hadc2/htim1/htim3/htim6/hspi3/huart3), GPIO types */

/* ---------------- Motor power stage (half-bridge SD / enable) ---------------- */
#define BOARD_SD1_PORT     GPIOB
#define BOARD_SD1_PIN      GPIO_PIN_0
#define BOARD_SD2_PORT     GPIOA
#define BOARD_SD2_PIN      GPIO_PIN_1
#define BOARD_SD3_PORT     GPIOA
#define BOARD_SD3_PIN      GPIO_PIN_2

/* ---------------- FOC peripherals ---------------- */
#define BOARD_ADC_PHASE        hadc2          /* phase-current ADC (injected ranks) */
#define BOARD_ADC_PHASE_INST   ADC2           /* instance for ISR identity check */
#define BOARD_TIM_PWM          htim1          /* 3-phase PWM (CCR1/2/3) */
#define BOARD_TIM_ENCODER      htim3          /* quadrature encoder counter */
#define BOARD_TIM_TIMEBASE     htim6          /* FOC timebase */
#define BOARD_ADC_RANK_IU      ADC_INJECTED_RANK_1
#define BOARD_ADC_RANK_IW      ADC_INJECTED_RANK_2

/* ---------------- LCD (ST7735 over SPI) ---------------- */
#define BOARD_LCD_SPI          hspi3
#define BOARD_LCD_RES_PORT     GPIOC
#define BOARD_LCD_RES_PIN      GPIO_PIN_11
#define BOARD_LCD_DC_PORT      GPIOD
#define BOARD_LCD_DC_PIN       GPIO_PIN_2
#define BOARD_LCD_CS_PORT      GPIOA
#define BOARD_LCD_CS_PIN       GPIO_PIN_15
#define BOARD_LCD_SPI_TIMEOUT  10U

/* ---------------- Buttons (active-low) ---------------- */
#define BOARD_KEY_NEXT_PORT    GPIOC
#define BOARD_KEY_NEXT_PIN     GPIO_PIN_9
#define BOARD_KEY_ENTER_PORT   GPIOB
#define BOARD_KEY_ENTER_PIN    GPIO_PIN_12

/* ---------------- Command UART ---------------- */
#define BOARD_CMD_UART         huart3
#define BOARD_CMD_UART_INST    USART3
#define BOARD_CMD_UART_TIMEOUT 30U

#endif /* BOARD_CONFIG_H */
