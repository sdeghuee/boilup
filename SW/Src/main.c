/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t msCount = 1000;
uint32_t buttonPress = 0;
uint32_t buttonState0 = 0;

char rxData = 0x00;
unsigned char buffer[100];
unsigned char received[100];
uint8_t length;
uint8_t buffer_i = 0;
uint8_t carriageReturn = 0;
uint8_t requestingTime = 0;
uint8_t receivedO = 0;
uint8_t receivedC = 0;
uint32_t result = 0;
uint32_t operand = 0;
uint32_t txWait = 0;
uint32_t rxWait = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void transmitString(UART_HandleTypeDef * huart, unsigned char * str);
void receiveString();
uint32_t testRxString(unsigned char * test);

void wifiConnect(unsigned char * ssid, unsigned char * password);
void requestTime();
uint8_t testEndString(unsigned char * a, unsigned char * b);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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

void wifiConnect(unsigned char * ssid, unsigned char * password) {
    //unsigned char command[37];
    //sprintf(command, "AT+CWJAP=\"LZMedia_24\",\"SUBterm3575\"\r\n");
    //unsigned char command[37] = "AT+CWJAP=\"LZMedia_24\",\"SUBterm3575\"\r\n";
    //sprintf(command, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    transmitString(&huart1, "AT+CWJAP=\"LZMedia_24\",\"SUBterm3575\"\r\n");
}

void requestTime() {
    requestingTime = 1;
    transmitString(&huart1, "AT+CIPSTART=\"TCP\",\"time.nist.gov\",13\r\n");
}

uint8_t testEndString(unsigned char * str, unsigned char * end) {
    uint32_t start = strlen(str) - strlen(end);
    for (int i = start; i < strlen(str); i++) {
        if (str[i] != str[end]) {
            return 0;
        }
    }
    return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
    if (rxData == 0x4B && receivedO) {
        carriageReturn = 1;
    }
    else if (requestingTime && rxData == 0x4C && receivedC){
        carriageReturn = 1;
    }
    else if (rxData == 0x4F) {
        receivedO = 1;
    }
    else if (rxData == 0x43) {
        receivedC = 1;
    }
    else if (rxData != 0x0D && rxData != 0x0A) {
        if (receivedO) {
            receivedO = 0;
            buffer[buffer_i++] = 0x4F;
        }
        if (receivedC) {
            receivedC = 0;
            buffer[buffer_i++] = 0x43;
        }
        uint8_t pData = rxData;
        buffer[buffer_i++] = rxData;
    }
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) {
    for (int i = 0; i < 100; i++) {
        buffer[i] = 0;
    }
    txWait = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
    if (htim->Instance == TIM3) {
        msCount--;
        if (msCount <= 0) {
            msCount = 1000;
        }
        if (msCount % 5 == 0) {
            buttonDebounce();
        }
        if (msCount % 2 == 0) {
            debounce = 1;
        }
    }
}

void buttonDebounce() {
    uint32_t current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) & 1;
    buttonPress = !buttonState0 && current;
    buttonState0 = current;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  uint8_t pData;
  uint8_t column = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      if (buttonPress) {
          buttonPress = 0;
          HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
//          receiveString();
          i2cDisplayString(&hi2c2, received);
//          uint8_t cat[12] = {0x28, 0x02, 0x01, 0x28, 0x5E, 0x2E, 0x5E,
//                             0x29, 0x2F, 0x02, 0x29};
//          HAL_I2C_Master_Transmit_IT(&hi2c2, 80, cat, strlen(cat));
      }
      if (debounce) {
          debounce = 0;
          matrixDebounce(column++);
          if (column > 3) {
              column = 1;
          }
      }
      if (carriageReturn) {
          carriageReturn = 0;
          receivedO = 0;
          if (requestingTime) {
              requestingTime = 1;
              i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN,0x00);
          }
          receiveString();
          i2cDisplayString(&hi2c2, received);
      }
      for (uint32_t index = 0; index < 12; index++) {
          if (btnPress[index] == 1) {
              btnPress[index] = 0;
              switch (index) {
                  case 0:   // 1
                      pData = '1';
                      wifiConnect("LZMedia", "SUBterm3575");
                      break;
                  case 1:   // 4
                      pData = '4';
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_MOVELEFT, 0x00);
                      break;
                  case 2:   // 7
                      pData = '7';
                      break;
                  case 3:   // *
                      pData = '*';
                      break;
                  case 4:   // 2
                      pData = '2';
                      transmitString(&huart1, "AT\r\n");
                      break;
                  case 5:   // 5
                      pData = '5';
                      break;
                  case 6:   // 8
                      pData = '8';
                      break;
                  case 7:   // 0
                      pData = '0';
                      break;
                  case 8:   // 3
                      pData = '3';
                      requestTime();
                      break;
                  case 9:   // 6
                      pData = '6';
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_MOVELEFT, 0x00);
                      break;
                  case 10:  // 9
                      pData = '9';
                      break;
                  case 11:  // #
                      pData = '#';
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      break;
              }
              //HAL_I2C_Master_Transmit_IT(&hi2c2, 80, &pData, 1);
          }
      }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
