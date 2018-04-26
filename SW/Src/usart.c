/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
char rxData = 0x00;
unsigned char buffer[100];
unsigned char received[100];
unsigned char time[9];
Time currentTime;
char hoursChar[3];
char minutesChar[3];
uint8_t length;
uint8_t buffer_i = 0;
uint8_t carriageReturn = 0;
uint8_t requestingTime = 0;
uint8_t timeReady = 0;
uint8_t receivedO = 0;
uint8_t receivedC = 0;
uint32_t result = 0;
uint32_t operand = 0;
uint32_t txWait = 0;
uint32_t rxWait = 0;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA10     ------> USART1_RX
    PB6     ------> USART1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA10     ------> USART1_RX
    PB6     ------> USART1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void transmitString(UART_HandleTypeDef * huart, unsigned char * str) {
    HAL_UART_Transmit_IT(huart, str, strlen(str));
}

void receiveString() {
    length = buffer_i;
    buffer_i = 0;
    for (int i = 0; i < length; i++){
        received[i] = buffer[i];
    }
    for (int i = 0; i < 100; i++) {
        buffer[i] = 0;
    }
}

uint32_t testRxString(unsigned char * test) {
    if (strlen(test) != length) {
        return 0;
    }
    for (int i = 0; i < length; i++) {
        if (test[i] != received[i]) {
            return 0;
        }
    }
    return 1;
}

void wifiConnect() {
    //unsigned char command[37];
    //sprintf(command, "AT+CWJAP=\"LZMedia_24\",\"SUBterm3575\"\r\n");
    //unsigned char command[37] = "AT+CWJAP=\"LZMedia_24\",\"SUBterm3575\"\r\n";
    //sprintf(command, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);

    //transmitString(&huart1, "AT+CWJAP=\"LZMedia_24\",\"SUBterm3575\"\r\n");
    transmitString(&huart1, "AT+CWJAP=\"Hai-Fi\",\"0123456789\"\r\n");
}

void requestTime() {
    requestingTime = 1;
    transmitString(&huart1, "AT+CIPSTART=\"TCP\",\"time.nist.gov\",13\r\n");
}

void parseTime(unsigned char * rawTime) {
        uint8_t foundFirst = 0;
        uint32_t i = 0;
        uint32_t offset = 0;
        while (i < strlen(rawTime)) {
            if (rawTime[i] == 0x3A) {
                if (foundFirst) {
                    offset = i - 2;
                    break;
                }
                else {
                    foundFirst = 1;
                }
            }
            i++;
        }
        if (!foundFirst) {
            receiveString();
        }

        for (i = 0; i < 5; i++) {
            if (i < 2) {
                hoursChar[i] = rawTime[i + offset];
            }
            else if (i > 2) {
                minutesChar[i - 3] = rawTime[i + offset];
            }
        }
        hoursChar[2] = '\0';
        minutesChar[2] = '\0';
        currentTime.hours = atoi(hoursChar);
        currentTime.minutes = atoi(minutesChar);
        if (currentTime.hours < 4) {
            currentTime.hours += 24;
        }
        currentTime.hours -= 4;
        if (currentTime.hours == 0) {
            currentTime.hours += 12;
        }
        currentTime.alarmEnabled = 0;
        formatTime(&currentTime);
}

uint8_t testEndString(unsigned char * str, unsigned char * end) {
    if (strlen(str) < strlen(end)) {
        return 0;
    }
    uint32_t start = strlen(str) - strlen(end);
    for (int i = start; i < strlen(str); i++) {
        if (str[i] != end[i]) {
            return 0;
        }
    }
    return 1;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
