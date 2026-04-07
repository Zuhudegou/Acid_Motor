#include "ma600_encoder.h"
#include <math.h>

#define TWO_PI 6.283185307f
#define MA600_SPI_TIMEOUT 1U 

static uint16_t last_valid_angle = 0;

void MA600_Init(void)
{
    HAL_GPIO_WritePin(MA600_CS_GPIO_Port, MA600_CS_Pin, GPIO_PIN_SET);
}

uint16_t MA600_ReadAngleRaw(void)
{
    uint16_t tx_data = 0x0000;
    uint16_t rx_data = 0;

    HAL_GPIO_WritePin(MA600_CS_GPIO_Port, MA600_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1, MA600_SPI_TIMEOUT);

    HAL_GPIO_WritePin(MA600_CS_GPIO_Port, MA600_CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK)
    {
        last_valid_angle = rx_data;
    }

    return last_valid_angle;
}

float MA600_ReadAngleRad(void)
{
    uint16_t raw_val = MA600_ReadAngleRaw();
    return ((float)raw_val / 65536.0f) * TWO_PI;
}
