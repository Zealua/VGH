/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HX711_H__
#define __HX711_H__
#include "stm32f4xx_hal.h"
#include "Stdint.h"

	// PB3 - sck (output)
	// PB4 - data (input)
#define sck_PIN GPIO_PIN_3
#define sck_PORT GPIOB

#define data_PIN GPIO_PIN_4
#define data_PORT GPIOB



//#include "Variables.c"


uint32_t nyl=0;

/* Private variables ---------------------------------------------------------*/
void scalesCalibration(void);
uint32_t getWeight(void);
uint32_t GetWeight(void);

#endif /* __HX711_H__ */

/************************ (C) COPYRIGHT Zakaz.UA *****END OF FILE****/
