
#include "HX711.h"
//#include "Variables.h"



uint32_t GetWeight(void) {     // for HX711
	

	uint32_t adc_value = 0;
	
	// PB3 - sck (output)
	// PB4 - data (input)
	
	HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_RESET);
	// wait when data end
	while(HAL_GPIO_ReadPin(data_PORT, data_PIN) > 0);
	
	for(uint8_t i=0; i<24; i++)
	{
		HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_SET);
		adc_value <<= 1;
		HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(data_PORT, data_PIN) >0)
		{
			adc_value++;
		}	
	}

	HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	
	
	return adc_value;
}

uint32_t getWeight(void){
  
  uint32_t weightData=0;
  
  for(uint8_t i=0;i<10;i++){
    weightData+=GetWeight();
  }
  weightData/=10;
  weightData-=nyl;
  weightData/=(43*1.7984);

  if (weightData>1677216) {weightData=0;}
  
  return weightData;
}

void scalesCalibration(){

    HAL_GPIO_WritePin(data_PORT, data_PIN, GPIO_PIN_RESET);
		
		nyl=0;
		
		for (uint8_t i=0; i<10; i++) {
		nyl+=GetWeight();
		}
		nyl/=10;
		
		HAL_GPIO_WritePin(data_PORT, data_PIN, GPIO_PIN_SET);
                
//    sprintf(part,"--------------------------\r\n");          
//    HAL_UART_Transmit(&huart6, (uint8_t*) part, 28,80);
//    sprintf(part,"-=Scale Calibration=-\r\n");          
//    HAL_UART_Transmit(&huart6, (uint8_t*) part, 23,80);	
//    sprintf(part,"--------------------------\r\n\r\n");          
//    HAL_UART_Transmit(&huart6, (uint8_t*) part, 30,80);
}