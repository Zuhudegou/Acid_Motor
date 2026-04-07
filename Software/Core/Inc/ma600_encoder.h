#ifndef __MA600_ENCODER_H__
#define __MA600_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;

#ifndef MA600_CS_GPIO_Port
#define MA600_CS_GPIO_Port GPIOA
#endif
#ifndef MA600_CS_Pin
#define MA600_CS_Pin GPIO_PIN_4
#endif

void MA600_Init(void);

uint16_t MA600_ReadAngleRaw(void);

float MA600_ReadAngleRad(void);

#ifdef __cplusplus
}
#endif

#endif
