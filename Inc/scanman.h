#ifndef __scanman_H__
#define __scanman_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_adc.h"

#include "stm32f4_discovery_lcd.h"
#include "HX711.h"
#include "US30.h"

#define MESSAGE1   "       READY     " 
#define MESSAGE3   "     zakaz.ua      " 

void ScanMan(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1);
void showParamLCD(void);
//void sendParamUart(UART_HandleTypeDef huart6);
void startLoad(UART_HandleTypeDef huart6);
void checkCommand(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1);
void calibrationAndShow(UART_HandleTypeDef huart6);
void pressSB3(UART_HandleTypeDef huart6);
void pressSB2(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1);
void getAndShowParam(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1);
#endif /* __scanman_H__ */