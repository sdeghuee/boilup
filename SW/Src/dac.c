/**
  ******************************************************************************
  * File Name          : DAC.c
  * Description        : This file provides code for the configuration
  *                      of the DAC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dac.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
int max[] = {152,114,90,76,57,45,38,45,144,114,96,72,57,48,36,48,128,102,85,64,51,43,32,0,32,0,32,0,32,28};
int duty[] = {76,57,45,38,28,22,19,22,72,67,48,36,28,24,18,24,64,51,42,32,25,21,16,0,16,0,16,0,16,14};
int maxFF[] = {57,0,57,0,57,0,57,71,64,57,0,64,57,76,85,76,85,64,0,64,68,64,68,0,68,76,85,91,85,102,0,
             76,85,76,85,64,0,64,68,64,68,0,68,76,85,76,64,57,0};
int dutyFF[]= {28,0,28,0,28,0,28,35,32,28,0,32,28,38,42,38,42,32,0,32,34,32,34,0,34,38,42,45,42,51,0,
             38,42,38,42,32,0,32,34,32,34,0,34,38,42,38,32,28,0};
int rate[] = {80,120,160,200,240,280,320,360};
int rate2[] = {92,109,182,146,164,218};
int dir = 0;
int stateDac = 0;
int doneWave;
extern int cycles;
extern int updateDACEnable;
extern int dacStage;
int repeat = 0;
/* USER CODE END 0 */

DAC_HandleTypeDef hdac1;

/* DAC1 init function */
void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* dacHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(dacHandle->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspInit 0 */

  /* USER CODE END DAC_MspInit 0 */
    /* DAC clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();
  
    /**DAC1 GPIO Configuration    
    PA4     ------> DAC1_OUT1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DAC interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN DAC_MspInit 1 */

  /* USER CODE END DAC_MspInit 1 */
  }
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* dacHandle)
{

  if(dacHandle->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspDeInit 0 */

  /* USER CODE END DAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC1_CLK_DISABLE();
  
    /**DAC1 GPIO Configuration    
    PA4     ------> DAC1_OUT1 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* DAC interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN DAC_MspDeInit 1 */

  /* USER CODE END DAC_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void DoneCheckWave(){
	if(doneWave > max[stateDac])
		doneWave = 0;
}
void FFCheckWave(){
	if(doneWave > maxFF[stateDac])
		doneWave = 0;
}
void FFSetWave(){
	int val;
	if(doneWave++ > dutyFF[stateDac])
		val = 0;
	else val = 255;
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_8B_R,val);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
}
void DoneSetWave(){
	int val;
	if(doneWave++ > duty[stateDac])
		val = 0;
//	else val = 255;
    else val = 7;
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_8B_R,val);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
}
void DoneGetState(){
		  if(stateDac != 6 && stateDac != 7 && stateDac != 14 && stateDac != 15 && stateDac != 22
				  && stateDac != 29 && stateDac != 23 && stateDac != 25 && stateDac != 27){
			  if(cycles > 138){
				  stateDac++;
				  cycles = 0;
			  }
		  }else if(stateDac == 29){
			  if(cycles > 828){
				  stateDac++;
				  cycles = 0;
			  }
		  }
		  else if(stateDac == 23 || stateDac == 25 || stateDac == 27){
			  if(cycles > 1)
			  {
				  stateDac++;
				  cycles = 0;
			  }
		  }else if(stateDac == 24 || stateDac == 26 || stateDac == 28){
			  if(cycles > 137){
				  stateDac++;
				  cycles = 0;
			  }
		  }
		  else{
			  if(cycles > 414){
				  stateDac++;
				  cycles = 0;
			  }
		  }
		  if(stateDac > 29)
		  {
			  stateDac = 0;
			  updateDACEnable = 0;
		  }
}
int triSetWave(int val){
	switch(dacStage){
        case 1:
            if(!dir){
                val+=rate[stateDac];
            }
            else{
                val -= rate[stateDac];
            }
            break;
        case 2:
            if(!dir){
                val+=rate2[stateDac];
            }
            else{
                val-=rate2[stateDac];
            }
            break;
	}
	if(val >= 4095){
		val = 4095;
		dir = 1;
	}
	if(val <= 0){
		val = 0;
		dir = 0;
	}
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,val);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
	return val;
}
void triGetState(int stage){
	switch(stage){
	case 1:
		if(cycles > 40){
			stateDac++;
			cycles = 0;
		}
		 if(stateDac == 8){

			 stateDac = 0;
            repeat++;
		 }
         if(repeat > 1)
         {
             repeat = 0;
             updateDACEnable = 0;
         }
		break;
	case 2:
		if(cycles > 160){
			stateDac++;
			cycles = 0;
		}
		if(stateDac == 6){
			stateDac = 0;
           updateDACEnable = 0;
		}
	}
}
void FFGetstateDac(){
          if(stateDac != 0 && stateDac != 1 && stateDac != 2 && stateDac != 3 && stateDac != 4
           && stateDac != 5 && stateDac != 9 && stateDac != 10 && stateDac != 11 && stateDac != 12
		   && stateDac != 16 && stateDac != 17 && stateDac != 18 && stateDac != 19 && stateDac != 21
            && stateDac != 22 && stateDac != 23 && stateDac != 24 && stateDac != 28
            && stateDac != 29 && stateDac != 34 && stateDac != 35 && stateDac != 36
            && stateDac != 37 && stateDac != 39 && stateDac != 40 && stateDac != 41
            && stateDac != 42 && stateDac != 46 && stateDac != 47){ //quarter
           if(cycles > 360){//0,1,2,3,4,5,9,10,11,12,16,17,18,19,21,22,23,24,28,29,34,35,36,37,39,40,41,42,46,47
               stateDac++;
               cycles = 0;
           }
         }else if(stateDac == 12){ //dotted half
           if(cycles > 1080){
               stateDac++;
               cycles = 0;
           }
         }else if(stateDac == 0 || stateDac == 2 || stateDac == 4){ //triplets shortened
               if(cycles > 119){
                   stateDac++;
                   cycles = 0;
           }

           }else if(stateDac == 9 || stateDac == 10 || stateDac == 11){ //triplets normal
           if(cycles > 120){
               stateDac++;
               cycles = 0;
           }
           }else if(stateDac == 17 || stateDac == 22 || stateDac == 35 || stateDac == 40){ //quarter shortened
               if(cycles > 359){
                   stateDac++;
                   cycles = 0;
               }

           }else if(stateDac == 1 || stateDac == 3 || stateDac == 5 || stateDac == 18 || stateDac == 23
           || stateDac == 36 ||stateDac == 41){ //spaces
           if(cycles > 1)
           {
               stateDac++;
               cycles = 0;
           }
         }else if(stateDac == 16 || stateDac == 19 || stateDac == 21 || stateDac == 24 || stateDac == 28
               || stateDac == 34 || stateDac == 37 || stateDac == 39 || stateDac == 42 || stateDac == 46){ //eighth
           if(cycles > 180){
               stateDac++;
               cycles = 0;
           }
         }          else{ //eighth+dotted half 29, 47
           if(cycles > 1260){
               stateDac++;
               cycles = 0;
           }
          }
          if(stateDac > 48)
         {
           stateDac = 0;
           updateDACEnable = 0;
          }
           }
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
