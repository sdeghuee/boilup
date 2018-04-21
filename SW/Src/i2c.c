/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t display_commandBits[20] = {0x41, 0x42, 0x45, 0x46, 0x47, 0x48, 0x49,
                                   0x4A, 0x4B, 0x4C, 0x4E, 0x51, 0x52, 0x53,
                                   0x54, 0x55, 0x56, 0x62, 0x70, 0x72};
uint8_t display_delays[20] = {0, 0, 0, 2, 2, 2, 0, 0, 0, 0, 0, 2, 1, 0, 1, 0,
                              0, 3, 4, 4};
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303EFD;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    /**I2C2 GPIO Configuration    
    PF6     ------> I2C2_SCL
    PF7     ------> I2C2_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C2 GPIO Configuration    
    PF6     ------> I2C2_SCL
    PF7     ------> I2C2_SDA 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6|GPIO_PIN_7);

    /* I2C2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void i2cDisplaySendCommand(I2C_HandleTypeDef * hi2c, uint8_t command, uint8_t param) {
    if (command == 2 || command == 12 || command == 13 || command == 17) {
        uint8_t pData[3] = {0xFE, display_commandBits[command], param};
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 3);
        HAL_Delay(display_delays[command]);
    }
    else if (command != DISPLAY_LOADCUSTOM) {
        uint8_t pData[2] = {0xFE, display_commandBits[command]};
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 2);
        HAL_Delay(display_delays[command]);
    }
    else {
        uint8_t slash[11] = {0xFE, 0x54, 0x01, 0x00, 0x10, 0x08, 0x04,
                             0x02, 0x01, 0x00, 0x00};
        uint8_t tripleApostrophe[11] = {0xFE, 0x54, 0x02, 0x15, 0x15, 0x15, 0x00,
                             0x00, 0x00, 0x00, 0x00};
        uint8_t kettleLeft[11] = {0xFE, 0x54, 0x03, 0x01, 0x03, 0x0F, 0x0B,
                             0x0B, 0x0F, 0x03, 0x00};
        uint8_t kettleRight[11] = {0xFE, 0x54, 0x04, 0x10, 0x18, 0x18, 0x1B,
                             0x1E, 0x1C, 0x18, 0x00};
        uint8_t pData[11] = {0xFE, 0x54, 0x01, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00};
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 11);
        HAL_Delay(display_delays[command]);
        pData[2] = 0x02;
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 11);
        HAL_Delay(display_delays[command]);
        pData[2] = 0x03;
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 11);
        HAL_Delay(display_delays[command]);
        pData[2] = 0x04;
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 11);
        HAL_Delay(display_delays[command]);
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &slash, 11);
        HAL_Delay(display_delays[command]+2);
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &tripleApostrophe, 11);
        HAL_Delay(display_delays[command]+2);
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &kettleLeft, 11);
        HAL_Delay(display_delays[command]+2);
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &kettleRight, 11);
        HAL_Delay(display_delays[command]+2);
    }
}


void i2cDisplayString(I2C_HandleTypeDef * hi2c, unsigned char * str) {
    HAL_I2C_Master_Transmit_IT(hi2c, 80, str, strlen(str));
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
