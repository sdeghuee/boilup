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
#include "displaycommands.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char rxData = 0x00;
unsigned char buffer[100];
unsigned char received[100];
char tmp[100];
uint8_t length;
uint8_t buffer_i = 0;
uint8_t carriage_return = 0;
//uint8_t custom = 0;
uint8_t store = 0;
uint8_t value = 0;
uint32_t txWait = 0;
uint32_t rxWait = 0;
uint32_t msCount = 1000;
uint32_t buttonPress = 0;
uint32_t buttonState0 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void transmitUARTString(UART_HandleTypeDef * huart, unsigned char * str);
void receiveUARTString();
uint32_t testRxString(unsigned char * test);
void transmitI2CString(I2C_HandleTypeDef * hi2c, unsigned char * str);
void buttonDebounce();

//void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address);
//void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void transmitUARTString(UART_HandleTypeDef * huart, unsigned char * str) {
    HAL_UART_Transmit_IT(huart, str, strlen(str));
}

void receiveUARTString() {
    length = buffer_i;
    buffer_i = 0;
    for (int i = 0; i< length; i++){
        received[i] = buffer[i];
    }
}

uint32_t testRxString(unsigned char * test) {
    if (strlen(test) != length) {
        return 0;
    }
    for (int i = 0; i < length; i++) {
        if (tolower(test[i]) != tolower(received[i])) {
            return 0;
        }
    }
    return 1;
}

void transmitI2CString(I2C_HandleTypeDef * hi2c, unsigned char * str) {
    HAL_I2C_Master_Transmit_IT(hi2c, 80, str, strlen(str));
}

void buttonDebounce() {
    uint32_t current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) & 1;
    buttonPress = !buttonState0 && current;
    buttonState0 = current;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
    if (rxData == 0x0D || rxData == 0x0A) {
        // carriage return/line feed
        carriage_return = 1;
    }
    else {
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

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c) {
    //txWait = 0;
    for (int i = 0; i < 100; i++) {
        received[i] = 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
    msCount -= 5;
    if (msCount <= 0) {
        msCount = 1000;
    }
    buttonDebounce();
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
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
    HAL_TIM_Base_Start_IT(&htim2);
    while(!buttonPress) {}
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    uint8_t pData;
    buttonPress = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (buttonPress) {
            buttonPress = 0;
//            transmitUARTString(&huart1, "Hey there\n\r");
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
            pData = value;
            HAL_I2C_Master_Transmit_IT(&hi2c1, 80, &pData, 1);
        }
        if (carriage_return) {
            rxWait = 0;
            carriage_return = 0;
            receiveUARTString();
            if (store) {
                store = 0;
                value = atoi(received);
                for (int i = 0; i < 100; i++) {
                    received[i] = 0;
                }
            }
            else if (testRxString("On")) {
                transmitDisplayCommand(&hi2c1, 0, 0);
            }
            else if (testRxString("Off")) {
                transmitDisplayCommand(&hi2c1, 1, 0);
            }
            else if (testRxString("Set Cursor")) {
                transmitDisplayCommand(&hi2c1, 2, value);
            }
            else if (testRxString("Cursor Home")) {
                transmitDisplayCommand(&hi2c1, 3, 0);
            }
            else if (testRxString("Underline On")) {
                transmitDisplayCommand(&hi2c1, 4, 0);
            }
            else if (testRxString("Underline Off")) {
                transmitDisplayCommand(&hi2c1, 5, 0);
            }
            else if (testRxString("Cursor Left")) {
                transmitDisplayCommand(&hi2c1, 6, 0);
            }
            else if (testRxString("Cursor Right")) {
                transmitDisplayCommand(&hi2c1, 7, 0);
            }
            else if (testRxString("Blink On")) {
                transmitDisplayCommand(&hi2c1, 8, 0);
            }
            else if (testRxString("Blink Off")) {
                transmitDisplayCommand(&hi2c1, 9, 0);
            }
            else if (testRxString("Backspace")) {
                transmitDisplayCommand(&hi2c1, 10, 0);
            }
            else if (testRxString("Clear")) {
                transmitDisplayCommand(&hi2c1, 11, 0);
            }
            else if (testRxString("Set Contrast")) {
                transmitDisplayCommand(&hi2c1, 12, value);
            }
            else if (testRxString("Set Brightness")) {
                transmitDisplayCommand(&hi2c1, 13, value);
            }
            else if (testRxString("Load Custom")) {
                transmitDisplayCommand(&hi2c1, 14, 0);
            }
            else if (testRxString("Move Left")) {
                transmitDisplayCommand(&hi2c1, 15, 0);
            }
            else if (testRxString("Move Right")) {
                transmitDisplayCommand(&hi2c1, 16, 0);
            }
            else if (testRxString("Change Address")) {
                transmitDisplayCommand(&hi2c1, 17, 80);
            }
            else if (testRxString("Show Firmware")) {
                transmitDisplayCommand(&hi2c1, 18, 0);
            }
            else if (testRxString("Show Address")) {
                transmitDisplayCommand(&hi2c1, 19, 0);
            }
            else if (testRxString("Cat")) {
                uint8_t cat[12] = {0x28, 0x02, 0x01, 0x28, 0x5E, 0x2E, 0x5E,
                                   0x29, 0x2F, 0x02, 0x29};
                HAL_I2C_Master_Transmit_IT(&hi2c1, 80, cat, strlen(cat));
            }
            else if (testRxString("Kettle")) {
                uint8_t kettle[12] = {0x03, 0x04, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                HAL_I2C_Master_Transmit_IT(&hi2c1, 80, kettle, strlen(kettle));
            }
            else if (testRxString("Store")) {
                store = 1;
                for (int i = 0; i < 100; i++) {
                    received[i] = 0;
                }
            }
            else {
                transmitI2CString(&hi2c1, received);
                //HAL_I2C_Master_Transmit_IT(&hi2c1, 80, &received, strlen(received));
//                pData = received[0];
//                transmitI2C(&hi2c1, received[0]);
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

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00101D7C;   //  50kHz
//  hi2c1.Init.Timing = 0x00101D47;   //  75kHz
//  hi2c1.Init.Timing = 0x2000090E; // 100kHz
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
