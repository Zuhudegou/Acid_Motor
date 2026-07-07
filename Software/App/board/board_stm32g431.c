/*
 * board_stm32g431.c — STM32G431RBT6 board implementation of the board_* port.
 *
 * This is the ONLY translation unit in App/ that includes main.h and calls the
 * STM32 HAL. Every board_* function declared in board.h is implemented here in
 * terms of the pin/handle map from board_config.h. The weak HAL ISR callbacks
 * (TIM update, ADC injected-complete, UART RX/error) also live here, because
 * CubeMX dispatches them by HAL handle and they must see the HAL types.
 *
 * To port to another MCU: copy this file to board_<target>.c, reimplement each
 * board_* function with the target HAL, and adjust board_config.h.
 */

#include "board.h"
#include "board_config.h"

/* Hot-path FOC ISR entry (implemented in motor_hal.c) and command RX sink
 * (implemented in motor_command.c). Declared here so the HAL weak callbacks
 * below can call into the HAL-free layers. */
void motor_hal_isr_tick(void);
void Motor_Command_OnRxByte(uint8_t byte);

/* One-byte RX target for the command UART interrupt. */
static uint8_t board_uart_rx_byte;

/* ===================== Motor power stage + FOC I/O ===================== */

int board_motor_adc_calibrate(void)
{
    return (HAL_ADCEx_Calibration_Start(&BOARD_ADC_PHASE, ADC_SINGLE_ENDED) == HAL_OK) ? 0 : -1;
}

int board_motor_start_io(uint16_t *adc_dma_buf, uint16_t n)
{
    if (HAL_ADC_Start_DMA(&BOARD_ADC_PHASE, (uint32_t *)adc_dma_buf, n) != HAL_OK) return -1;
    if (HAL_TIM_Encoder_Start(&BOARD_TIM_ENCODER, TIM_CHANNEL_ALL) != HAL_OK)     return -1;

    __HAL_TIM_SET_COMPARE(&BOARD_TIM_PWM, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&BOARD_TIM_PWM, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&BOARD_TIM_PWM, TIM_CHANNEL_3, 0U);

    if (HAL_TIM_PWM_Start(&BOARD_TIM_PWM, TIM_CHANNEL_1) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(&BOARD_TIM_PWM, TIM_CHANNEL_2) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(&BOARD_TIM_PWM, TIM_CHANNEL_3) != HAL_OK) return -1;
    if (HAL_TIM_Base_Start_IT(&BOARD_TIM_PWM) != HAL_OK)           return -1;
    if (HAL_TIM_Base_Start(&BOARD_TIM_TIMEBASE) != HAL_OK)         return -1;

    return 0;
}

void board_motor_powerstage(uint8_t enable)
{
    GPIO_PinState s = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(BOARD_SD1_PORT, BOARD_SD1_PIN, s);
    HAL_GPIO_WritePin(BOARD_SD2_PORT, BOARD_SD2_PIN, s);
    HAL_GPIO_WritePin(BOARD_SD3_PORT, BOARD_SD3_PIN, s);
}

void board_motor_read_phase(int32_t *iu, int32_t *iw)
{
    *iu = (int32_t)HAL_ADCEx_InjectedGetValue(&BOARD_ADC_PHASE, BOARD_ADC_RANK_IU);
    *iw = (int32_t)HAL_ADCEx_InjectedGetValue(&BOARD_ADC_PHASE, BOARD_ADC_RANK_IW);
}

uint16_t board_motor_read_encoder(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&BOARD_TIM_ENCODER);
}

void board_motor_write_pwm(uint16_t a, uint16_t b, uint16_t c)
{
    __HAL_TIM_SET_COMPARE(&BOARD_TIM_PWM, TIM_CHANNEL_1, a);
    __HAL_TIM_SET_COMPARE(&BOARD_TIM_PWM, TIM_CHANNEL_2, b);
    __HAL_TIM_SET_COMPARE(&BOARD_TIM_PWM, TIM_CHANNEL_3, c);
}

/* ===================== LCD ===================== */

void board_lcd_reset(uint8_t level)
{
    HAL_GPIO_WritePin(BOARD_LCD_RES_PORT, BOARD_LCD_RES_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void board_lcd_dc(uint8_t level)
{
    HAL_GPIO_WritePin(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void board_lcd_cs(uint8_t level)
{
    HAL_GPIO_WritePin(BOARD_LCD_CS_PORT, BOARD_LCD_CS_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void board_lcd_spi_write(const uint8_t *data, uint16_t len)
{
    (void)HAL_SPI_Transmit(&BOARD_LCD_SPI, (uint8_t *)data, len, BOARD_LCD_SPI_TIMEOUT);
}

/* ===================== Buttons / time base ===================== */

uint8_t board_key_next_pressed(void)
{
    return (HAL_GPIO_ReadPin(BOARD_KEY_NEXT_PORT, BOARD_KEY_NEXT_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t board_key_enter_pressed(void)
{
    return (HAL_GPIO_ReadPin(BOARD_KEY_ENTER_PORT, BOARD_KEY_ENTER_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

uint32_t board_millis(void)
{
    return HAL_GetTick();
}

void board_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ===================== Command UART ===================== */

void board_uart_tx(const uint8_t *data, uint16_t len)
{
    (void)HAL_UART_Transmit(&BOARD_CMD_UART, (uint8_t *)data, len, BOARD_CMD_UART_TIMEOUT);
}

void board_uart_rx_arm(void)
{
    (void)HAL_UART_Receive_IT(&BOARD_CMD_UART, &board_uart_rx_byte, 1U);
}

/* ===================== Real-time ISR callbacks =====================
 * CubeMX dispatches these weak HAL callbacks by handle. They live here (the
 * only HAL-aware TU) and forward into the HAL-free layers.
 *   TIM_PWM update (prio 0) -> software-trigger the phase-current ADC injection
 *   ADC injected-complete (prio 1) -> one FOC control tick
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BOARD_TIM_PWM.Instance)
    {
        (void)HAL_ADCEx_InjectedStart_IT(&BOARD_ADC_PHASE);
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != BOARD_ADC_PHASE_INST)
    {
        return;
    }
    motor_hal_isr_tick();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != BOARD_CMD_UART_INST)
    {
        return;
    }
    Motor_Command_OnRxByte(board_uart_rx_byte);
    (void)HAL_UART_Receive_IT(&BOARD_CMD_UART, &board_uart_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BOARD_CMD_UART_INST)
    {
        (void)HAL_UART_Receive_IT(&BOARD_CMD_UART, &board_uart_rx_byte, 1U);
    }
}
