#include "US30.h"

extern uint32_t data_buffer[3];

void getWHL(ADC_HandleTypeDef hadc1){
// US30 --==ON==--
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);

  HAL_Delay(500);
                
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedPollForConversion(&hadc1,20);
  data_buffer[0]=100+HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)/1.35;        //read ADC value 
  data_buffer[1]=100+HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)/1.35;
  data_buffer[2]=100+HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3)/1.35;
  HAL_ADCEx_InjectedStop(&hadc1);  
  HAL_Delay(100);
//  US30   --==OFF==--      
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

}