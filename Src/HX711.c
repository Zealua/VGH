
#include "HX711.h"
//#include "Variables.h"

uint32_t nyl=0;

uint32_t GetWeight(void) {     // for HX711
	
	uint32_t adc_value = 0;
	
	HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_RESET);
	// wait when data end
	while(HAL_GPIO_ReadPin(data_PORT, data_PIN) > 0) ;
	
	for(uint8_t i=0; i<24; i++)	{
		HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_SET);
		adc_value <<= 1;
		HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(data_PORT, data_PIN) >0) {
			adc_value++;
		}	
	}

	HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(sck_PORT, sck_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
        
	return adc_value;

}

int32_t getWeight(void)
{

    int32_t weightData = 0;

    for (uint8_t i = 0; i < 3; i++) {
        weightData += GetWeight();
    }

    weightData /= 3;
    weightData -= nyl;
    weightData /= 15.27;//(28.2);

    if (weightData > 16077216){
        weightData = 0;
    }

    return weightData;
}

int32_t getWeightShot(void)
{

    int32_t weightData = 0;

    weightData += GetWeight();

    weightData -= nyl;
    weightData /= 15.27;//(28.2);

    if (weightData > 16077216){
        weightData = 0;
    }

    return weightData;
}

void scalesCalibration(){

    HAL_GPIO_WritePin(data_PORT, data_PIN, GPIO_PIN_RESET);
		
    nyl=0;
    for (uint8_t i=0; i<3; i++) {
	nyl+=GetWeight();
    }
    nyl/=3;
		
    HAL_GPIO_WritePin(data_PORT, data_PIN, GPIO_PIN_SET);
}