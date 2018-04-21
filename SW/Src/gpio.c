/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
int debounce = 0;
uint32_t btnPress[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t btnCurrent[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t btnPrevious[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PCPin PCPin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void matrixDebounce(uint32_t column){
    switch (column) {
        case 1:
//            GPIOC->ODR |= 0x00001000;//col3
//            GPIOD->ODR |= 0x00000004;//col2
//            GPIOB->ODR &= 0xFFFFFFF7;//col1
            HAL_GPIO_WritePin(COL2_PORT, COL2_PIN, 1);
            HAL_GPIO_WritePin(COL3_PORT, COL3_PIN, 1);
            HAL_GPIO_WritePin(COL1_PORT, COL1_PIN, 0);
            break;
        case 2:
//            GPIOB->ODR |= 0x00000008;
//            GPIOC->ODR |= 0x00001000;
//            GPIOD->ODR &= 0xFFFFFFFB;
            HAL_GPIO_WritePin(COL1_PORT, COL1_PIN, 1);
            HAL_GPIO_WritePin(COL3_PORT, COL3_PIN, 1);
            HAL_GPIO_WritePin(COL2_PORT, COL2_PIN, 0);
            break;
        case 3:
//            GPIOB->ODR |= 0x00000008;
//            GPIOD->ODR |= 0x00000004;
//            GPIOC->ODR &= 0xFFFFEFFF;
            HAL_GPIO_WritePin(COL1_PORT, COL1_PIN, 1);
            HAL_GPIO_WritePin(COL2_PORT, COL2_PIN, 1);
            HAL_GPIO_WritePin(COL3_PORT, COL3_PIN, 0);
            break;
    }
    for (uint32_t row = 0; row < 4; row++) {
//        btnCurrent[(column - 1)*4 + row] = !(GPIOB->IDR & (0x10 << (3-row)));
        switch (row) {
            case 0:
                btnCurrent[(column - 1)*4 + row] = !(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));
                break;
            case 1:
                btnCurrent[(column - 1)*4 + row] = !(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11));
                break;
            case 2:
                btnCurrent[(column - 1)*4 + row] = !(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
                break;
            case 3:
                btnCurrent[(column - 1)*4 + row] = !(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
                break;
        }
//        uint32_t a = !(GPIOB->IDR & (0x400 << row));
//        btnCurrent[(column - 1)*4 + row] = a;
        matrixButtonDebounce((column - 1)*4 + row);
    }
}

void matrixButtonDebounce(uint32_t index) {
    btnPress[index] = !btnPrevious[index] && btnCurrent[index];
    btnPrevious[index] = btnCurrent[index];
}
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
