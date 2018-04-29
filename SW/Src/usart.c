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
Time alarm;
Time currentTime;
char hoursChar[3];
char minutesChar[3];
uint8_t length;
uint8_t buffer_i = 0;
uint8_t carriageReturn = 0;
uint8_t requestingTime = 0;
uint8_t requestingAlarm = 0;
uint8_t timeReady = 0;
uint8_t alarmTimeReady = 0;
uint8_t receivedO = 0;
uint8_t receivedC = 0;
uint32_t result = 0;
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
    // function to send a string over USART
    HAL_UART_Transmit_IT(huart, str, strlen(str));
}

void receiveString() {
    // function to copy everything in buffer into received
    // and also clear the buffer array
    length = buffer_i;
    buffer_i = 0;
    for (int i = 0; i < length; i++){
        received[i] = buffer[i];
    }
    for (int i = 0; i < 100; i++) {
        buffer[i] = 0;
    }
}

void requestTime() {
    // function to tell the ESP to get time data from time server
    requestingTime = 1;     // set flag to indicate time data is expected
    transmitString(&huart1, "AT+CIPSTART=0,\"TCP\",\"time.nist.gov\",13\r\n");
}

void parseAlarm(unsigned char * alarmTime) {
    // function to store slack command data into usable variables
    cups = alarmTime[0] - '0';  // cups is the first characters
    pumpCups = cups;
    if (alarmTime[3] == ':') {
        // happens if there was no tens digit for hours, ie /teatime 2 4:00PM
        alarm.hours = alarmTime[2] - '0';   // save the hours
        unsigned char minStr[2];    // substring for minutes
        for (int i = 0; i < 2; i++) {
            minStr[i] = alarmTime[i + 4];   // copy minutes into substring
        }
        alarm.minutes = atoi(minStr);   // save minutes
        alarm.pm = 0;       // reset pm variable
        if (alarmTime[6] == 0x50) {     // 0x50 == P, so if PM was entered:
            alarm.hours += 12;  // add 12 to hours to fix 24 hour conversion
            alarm.pm = 1;       // set pm flag
        }
    }
    else {
        // happens if there was a tens digit for hours, ie /teatime 2 11:00AM
        unsigned char hoursStr[2];  // substring for hours
        for (int i = 0; i < 2; i++) {
            hoursStr[i] = alarmTime[i + 2];     // copy hours into substring
        }
        alarm.hours = atoi(hoursStr);       // save hours
        unsigned char minStr[2];        // substring for minutes
        for (int i = 0; i < 2; i++) {
            minStr[i] = alarmTime[i + 5];   // copy minutes into substring
        }
        alarm.minutes = atoi(minStr);       // save minutes
        alarm.pm = 0;       // reset pm flag
        if (alarmTime[7] == 0x50) {  // 0x50 == P, so if PM was entered:
            alarm.hours += 12;      // add 12 to hours to fix 24 hour conversion
            alarm.pm = 1;           // set pm flag
        }
    }
}

void parseTime(unsigned char * rawTime) {
    // function to parse time data from time server into usable variables
    uint8_t foundFirst = 0;     // flag for if the first ':' was found, which should be ignored
    uint32_t offset = 0;        // variable for storing index of where the hours, tens digit starts
    for (int i = 0; i < strlen(rawTime); i++) { // loop through the rawTime string
        if (rawTime[i] == 0x3A) {   // 0x3A == :, which is how the time is located, the second ':' found is HH:MM:SS
            //                                                                                              ^
            if (foundFirst) {   // if the first ':' has already been skipped
                offset = i - 2;     // set offset to store the place of the hours, tens digit
                break;
            }
            else {      // otherwise this was the first ':', and set the flag
                foundFirst = 1;
            }
        }
    }
    if (!foundFirst) {
        // if ':' wasn't found, copy buffer into received again
        receiveString();
    }

    for (int i = 0; i < 5; i++) {   // now loop through the hours and minutes characters
        if (i < 2) {    // looking at the hours
            hoursChar[i] = rawTime[i + offset];
        }
        else if (i > 2) {   // looking at the minutes
            minutesChar[i - 3] = rawTime[i + offset];
        }
    }
    hoursChar[2] = '\0';    // add escape char
    minutesChar[2] = '\0';  // add escape char
    currentTime.hours = atoi(hoursChar);        // store hours
    currentTime.minutes = atoi(minutesChar);    // store minutes
    if (currentTime.hours < 4) {        // don't underflow when doing time zone conversion
        currentTime.hours += 24;
    }
    currentTime.hours -= 4; // time zone conversion
    if (currentTime.hours == 0) {   // if it's midnight or anything 12:XX AM
        currentTime.hours += 12;    // add 12 to convert from 24 hour clock
    }
    currentTime.alarmEnabled = 0;   // reset alarm enable flag
    formatTime(&currentTime);       // use function to format time into a string
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
