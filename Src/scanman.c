#include "scanman.h"

//dimensions - rozmiry
//weights - vaga
// colibrations


/* Private variables ---------------------------------------------------------*/
uint8_t it=0;
char part[65]={0};
char StrLCD[65]={0};  
int k=430;
int32_t w=0,tempWeight=0;
uint32_t data_buffer[3]={0};
char getUart[15]={0};
uint32_t RAM_LCD_Read=0;

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
void ScanMan(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1){
  
  /* Initialize all configured peripherals */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_3, GPIO_PIN_SET);
    STM32f4_Discovery_LCD_Init();
    RAM_LCD_Read=LCD_ReadReg(0x00);
    
    startLoad(huart6);
    
    scalesCalibration();
    HAL_UART_Receive_IT(&huart6,(uint8_t*) getUart,5);

    while(1){

        checkCommand(huart6,hadc1);           // find command from COM-port
        pressSB3(huart6);                     //  ------------___wait_push_SB3__-----------------
        pressSB2(huart6,hadc1);               // ------------___wait_push_SB2__-----------------

        k++;
        if(k>50000){
          tempWeight+=getWeightShot();
          tempWeight/=2;
          if(tempWeight>-5&&tempWeight<5)
             {w=0;}
          else if((tempWeight-w)<-2||(tempWeight-w)>2)
              {w=tempWeight;}
           showParamLCD();
           k=0;
        }
            
    }
}
//--------------------------------------------------------------------------------
//===========================___END___============================================
//--------------------------------------------------------------------------------

void showParamLCD(void){
    LCD_Clear(LCD_COLOR_MAGENTA);
    LCD_SetBackColor(LCD_COLOR_MAGENTA);
    sprintf(StrLCD,"Product # 00004587");	
    LCD_DisplayStringLine(LINE(1), (uint8_t *)StrLCD);
    sprintf(StrLCD,"Weight: %d;",w);	
    LCD_DisplayStringLine(LINE(3), (uint8_t *)StrLCD);
    sprintf(StrLCD,"Length: %d;",data_buffer[0]);
    LCD_DisplayStringLine(LINE(5), (uint8_t *)StrLCD);	
    sprintf(StrLCD,"Width: %d;",data_buffer[1]);
    LCD_DisplayStringLine(LINE(6), (uint8_t *)StrLCD);	
    sprintf(StrLCD,"Height: %d.",data_buffer[2]);
    LCD_DisplayStringLine(LINE(7), (uint8_t *)StrLCD);	
}

void startLoad(UART_HandleTypeDef huart6){
    LCD_Clear(LCD_COLOR_BLUE);
    LCD_SetBackColor(LCD_COLOR_BLUE);
    LCD_SetTextColor(LCD_COLOR_WHITE);
    LCD_DisplayStringLine(LINE(3), (uint8_t *)"     START .");
    HAL_Delay(500);

    for(uint8_t i=0; i<7;i++){
      LCD_DisplayChar(LINE(3),184+i*8,46);
      HAL_Delay(500);
    }
    HAL_Delay(750);
    LCD_Clear(LCD_COLOR_GREEN);
    LCD_SetBackColor(LCD_COLOR_GREEN);
    LCD_SetTextColor(LCD_COLOR_BLUE);
    LCD_DisplayStringLine(LINE(4), (uint8_t *)MESSAGE1);
    LCD_DisplayStringLine(LINE(8), (uint8_t *)MESSAGE3);

      /**  send to UART imformation that device ready 
    */
    sprintf(part,"--------------------------\r\n");          
    HAL_UART_Transmit(&huart6, (uint8_t*) part, 28,80);
    sprintf(part,"====================\r\n");          
    HAL_UART_Transmit(&huart6, (uint8_t*) part, 22,80);
    sprintf(part,"__-==START WORK==-__\r\n");          
    HAL_UART_Transmit(&huart6, (uint8_t*) part, 22,80);	
    sprintf(part,"====================\r\n");          
    HAL_UART_Transmit(&huart6, (uint8_t*) part, 22,80);
    sprintf(part,"--------------------------\r\n\r\n");          
    HAL_UART_Transmit(&huart6, (uint8_t*) part, 30,80);


}
/**
* @brief This function look on duffer UART and analyze command
*/
void checkCommand(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1){
  if(getUart[0]!=0){
     HAL_Delay(100);  
     if(getUart[0]=='-'){
        switch (getUart[1]){
             case 'g': 
               getAndShowParam(huart6,hadc1);    
               break;
              case 'c':
                calibrationAndShow(huart6);
                break;
           }
        }
     for(uint8_t gu=0; gu<15;gu++)
         getUart[gu]=0;
     it=0;
  }
}
/**
* @brief This function set weight zero and show it on TFT + send UART6
*/
void calibrationAndShow(UART_HandleTypeDef huart6){
               
     scalesCalibration();
     LCD_Clear(LCD_COLOR_YELLOW);
     LCD_DisplayStringLine(LINE(3), (uint8_t *)" Calibration Done !  ");
                    
     w=getWeight();
                
     showParamLCD();
     sprintf(part,"\r\n\r\n Calibration done !!!");	
     HAL_UART_Transmit(&huart6, (uint8_t*) part, 25,80);
                    
}
/**
* @brief This function get param (weight and dimansion), show it in TFT and send UART_6
*/
void getAndShowParam(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1){
           
     w=getWeight();
     getWHL(hadc1);
           
//           sendParamUart(huart6);
     showParamLCD();
     sprintf(part,"\r\n\r\nWeight: %d;\r\n Length: %d;\r\n Width: %d;\r\n Height: %d\r\n\r\n",w,data_buffer[0],data_buffer[1],data_buffer[2]);	
     HAL_UART_Transmit(&huart6, (uint8_t*) part, 59,80);	

     w=0;
}
/**
* @brief This function check press button SB3, if press, do clibrations.
*/
void pressSB3(UART_HandleTypeDef huart6){
     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) < 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) < 1) {
            calibrationAndShow(huart6);
        }
     }
}
/**
* @brief This function check press button SB2, if press, do clibrations.
*/
void pressSB2(UART_HandleTypeDef huart6,ADC_HandleTypeDef hadc1){
     if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) < 1) {
        HAL_Delay(45);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) < 1) {
          getAndShowParam(huart6,hadc1);        
        }
     }
}