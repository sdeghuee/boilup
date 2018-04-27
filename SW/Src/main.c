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
#include "time.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int updateDACEnable = 0;
int updateDAC = 0;
int cycles = 0;
int dacStage = 1;
int waveVal = 0;
uint8_t promptDisplayed = 0;    // remove
uint8_t error = 0;
uint8_t digit = 0;
uint8_t servoWait = 0;          // remove
uint8_t workOnce = 0;           // remove
uint8_t secondFour = 0;
// alarm enable check
//uint32_t state = 0;
typedef enum {
    Startup,
    Wait,
    SetCups,
    SetTime,
    Pump,
    StartBoil,
    SwitchReset,
    WaterReady
} stateName;
stateName state = Startup;
typedef enum {
    StateNotRun,
    StateRun
} stateRunYet;
stateRunYet setCupsRunYet = StateNotRun;
stateRunYet errorRunYet = StateNotRun;
stateRunYet setTimeRunYet = StateNotRun;
stateRunYet pumpRunYet = StateNotRun;
stateRunYet startBoilRunYet = StateNotRun;
stateRunYet switchResetRunYet = StateNotRun;
stateRunYet waterReadyRunYet = StateNotRun;
uint32_t cups = 0;
uint32_t pumpCups = 0;       // fix
uint32_t msCount = 1000;
uint32_t secondCount = 10;
uint32_t buttonPress = 1;
uint32_t buttonState0 = 1;

uint32_t count = 0;
uint8_t once = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
//    if (!once) {
//        once = 1;
//        if (testEndString(buffer, "CLOSED")) {
//            carriageReturn = 1;
//            for (int i = 1; i < 7; i++) {
//                buffer[buffer_i - i] = 0;
//            }
//            buffer_i -= 6;
//        }
//        else if (rxData != 0x0D && rxData != 0x0A) {
//            uint8_t pData = rxData;
//            buffer[buffer_i++] = rxData;
//        }
//        if (buffer_i == 5) {
//            count = 0;
//        }
//    }
//    else if (requestingTime) {
//        if (testEndString(buffer, "OK")) {
//            // clear last two chars
//            // and carriageReturn = 1
//            carriageReturn = 1;
//            count = 0;
//            for (int i = 1; i < 3; i++) {
//                buffer[buffer_i - i] = 0;
//            }
//            buffer_i -= 6;
//        }
//        else if (rxData != 0x0D && rxData != 0x0A) {
//            uint8_t pData = rxData;
//            buffer[buffer_i++] = rxData;
//        }
//    }
    if (rxData == 0x4B && receivedO) {
        carriageReturn = 1;
    }
    else if (requestingTime && rxData == 0x4C && receivedC){
        requestingTime = 0;
        timeReady = 1;
        carriageReturn = 1;
    }
    else if (rxData == 0x26 && requestingAlarm) {
        for (int i = 0; i < 100; i++) {
            buffer[i] = 0;
        }
        buffer_i = 0;
        requestingAlarm = 0;
    }
    else if (rxData == 0x26) {
        carriageReturn = 1;
        alarmTimeReady = 1;
    }
    else if (rxData == 0x5E) {  //^
        for (int i = 0; i < 100; i++) {
            buffer[i] = 0;
            received[i] = 0;
        }
        buffer_i = 0;
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
        updateDAC = 1;
        if (msCount <= 0) {
            msCount = 1000;
            secondCount++;
            if (secondCount == 6) {
                //requestAlarm();
                secondFour = 1;
            }
        }
        if (secondCount == 10) {
            secondCount = 0;
            once = 0;
            requestTime();
            if (pumpCups - 1 <= 0 && (state == Pump || state == StartBoil || state == SwitchReset)) {
                if (state == Pump) {
                    state = Wait;
                }
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // question 1
            }
            else {
                if (state == Pump || state == StartBoil || state == SwitchReset) {
                    pumpCups--;
                }
            }
        }
        if (msCount % 5 == 0) {
//            buttonDebounce();
            buttonPress = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) & 1;
        }
        if (msCount % 2 == 0) {
            debounce = 1;
        }
        if(updateDACEnable) {
            cycles++;
        }
    }
    if(htim->Instance == TIM14){
        updateDAC = 1;
    }
}

void buttonDebounce() {
    uint32_t current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) & 1;
//    buttonPress = !buttonState0 && current;
//    buttonState0 = current;
    buttonPress = current;
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
//  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  HAL_Delay(10000);
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  i2cDisplaySendCommand(&hi2c2, DISPLAY_LOADCUSTOM, 0x00);
  i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETBACKLIGHT, 0x08);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // question 1
  uint8_t column = 1;
  updateDACEnable = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      if (secondFour) {
          secondFour = 0;
          requestingAlarm = 1;
//          transmitString(&huart1, "AT+CIPSTART=1,\"TCP\",\"http://sdeghuee.pythonanywhere.com\",80\r\n");
//          HAL_Delay(500);
          transmitString(&huart1, "AT+CIPSEND=1,71\r\n");
          HAL_Delay(500);
          transmitString(&huart1, "GET / HTTP/1.1\r\nHost: sdeghuee.pythonanywhere.com\r\nUser-Agent: test\r\n\r\n");
      }
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, buttonPress);
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
          receivedC = 0;
          receiveString();
          if (timeReady) {
              timeReady = 0;
              if (received[0] == 0x00 || received[0] == '0') {
                  requestTime();
              }
              else {
//                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
//                  i2cDisplayString(&hi2c2, received);
                  parseTime(received);
//                  if (state != 2 && state != 3 && state != 4 && !timeEqual(eight, currentTime)) {
                  if (state != SetCups && state != SetTime && state != Pump) {
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      i2cDisplayString(&hi2c2, currentTime.str);
                  }
                  if (timeEqual(currentTime, alarm)) {
                      if (alarm.alarmEnabled) {
                          alarm.alarmEnabled = 0;
                          state = StartBoil;
                      }
                  }
              }
          }
          else if (alarmTimeReady) {
              alarmTimeReady = 0;
              parseAlarm(received);
              if (state == Wait) {
                  state = Pump;
                  msCount = 1000;
                  secondCount = 0;
                  digit = 0;
                  alarm.alarmEnabled = 1;
              }
              setCupsRunYet = StateNotRun;
              errorRunYet = StateNotRun;
              setTimeRunYet = StateNotRun;
              pumpRunYet = StateNotRun;
              startBoilRunYet = StateNotRun;
              switchResetRunYet = StateNotRun;
              waterReadyRunYet = StateNotRun;
              for (int i = 0; i < 100; i++) {
                  received[i] = 0;
              }
          }
          else {
              for (int i = 0; i < 100; i++) {
                  received[i] = 0;
              }
          }
//          else {
//              i2cDisplayString(&hi2c2, received);
//          }
      }
      unsigned char matrixButton = 0;
      for (uint32_t index = 0; index < 12; index++) {
          if (btnPress[index] == 1) {
              btnPress[index] = 0;
              matrixButton = matrixGetButton(index);
              if (matrixButton == '#') {
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                  if (!error) {
                    state = Wait;
                    alarm.hours = 0;
                    alarm.minutes = 0;
                    alarm.pm = 0;
                    alarm.alarmEnabled = 0;
                    setCupsRunYet = StateNotRun;
                    errorRunYet = StateNotRun;
                    setTimeRunYet = StateNotRun;
                    pumpRunYet = StateNotRun;
                    startBoilRunYet = StateNotRun;
                    switchResetRunYet = StateNotRun;
                    waterReadyRunYet = StateNotRun;
                    cups = 0;
                    pumpCups = 0;
                    digit = 0;
//                    i2cDisplayString(&hi2c2, currentTime.str);
                    requestTime();
                    i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                    i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
                  }
              }
              /*
              switch (matrixButton) {
                  case '*':
                      switch (state) {
                          case Wait:
//                              state = SetCups;
                              break;
                          case SetCups:
//                              if (cups > 0) {
//                                  alarm.hours = 0;
//                                  alarm.minutes = 0;
//                                  alarm.pm = 0;
//                                  state = SetTime;
//                                  digit = 1;
//                                  promptDisplayed = 0;
//                              }
                              break;
                          case SetTime:
//                              if (digit >= 5) {
//                                  state = Pump;
//                                  msCount = 1000;
//                                  secondCount = 0;
//                                  promptDisplayed = 0;
//                                  digit = 0;
//                                  if (alarm.pm) {
//                                      alarm.hours += 12;
//                                  }
//                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
//                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
//                                  i2cDisplayString(&hi2c2, currentTime.str);
//                              }
                              break;
                          default:
                              break;
                      }
                      break;
                  case '#':
                      promptDisplayed = 0;
                      workOnce = 0;
                      digit = 0;
                      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                      if (!error) {
                        state = 1;
                        alarm.hours = 0;
                        alarm.minutes = 0;
                        alarm.pm = 0;
                        setCupsRunYet = StateNotRun;
                        errorRunYet = StateNotRun;
                        setTimeRunYet = StateNotRun;
                        pumpRunYet = StateNotRun;
                        startBoilRunYet = StateNotRun;
                        switchResetRunYet = StateNotRun;
                        waterReadyRunYet = StateNotRun;
                        cups = 0;
                        pumpCups = 0;
                        digit = 0;
                        i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                        i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
                        i2cDisplayString(&hi2c2, currentTime.str);
                      }
                      break;
                  default:
                      break;
              }*/
              if (error) {
                  error = 0;
              }
          }
      }

      // begin state machine
      switch (state) {
          case Startup:   // startup
              transmitString(&huart1, "AT+CIPMUX=1\r\n");
              HAL_Delay(1000);
              transmitString(&huart1, "AT+CIPSTART=1,\"TCP\",\"http://sdeghuee.pythonanywhere.com\",80\r\n");
              updateDACEnable = 1;
              dacStage = 1;
              servoUp();
              state = Wait;
              break;
          case Wait:   // waiting
              if (matrixButton == '*') {
                  state = SetCups;
              }
              break;
          case SetCups:   // set alarm - cups
              if (matrixButton == '*') {
                  if (cups > 0 && !error) {
                      state = SetTime;
                      alarm.hours = 0;
                      alarm.minutes = 0;
                      alarm.pm = 0;
                      digit = 1;
                      promptDisplayed = 0;
                  }
              }
              else if (!error) {
                  if (setCupsRunYet == StateNotRun) {
                      setCupsRunYet = StateRun;
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      i2cDisplayString(&hi2c2, "How many cups: ");
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKON, 0x00);
                  }
                  else {
                      if (matrixButton != '*' && matrixButton != '#' && matrixButton != 0) {
                          if (matrixButton > '0' && matrixButton <= '4') {  // 4 liter kettle
                              if (cups > 0) {
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                              }
                              unsigned char num[2];
                              sprintf(&num, "%c", matrixButton);
                              i2cDisplayString(&hi2c2, num);
                              cups = atoi(num);
                              pumpCups = cups;
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x0F);
                          }
                          else {
                              error = 1;
                              errorRunYet = StateNotRun;
                              setCupsRunYet = StateNotRun;
                              cups = 0;
                          }
                      }
                  }
              }
              else {
                  if (errorRunYet == StateNotRun) {
                      errorRunYet = StateRun;
                      setCupsRunYet = StateNotRun;
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      i2cDisplayString(&hi2c2, "4 Cups Max.");
                      HAL_Delay(5);
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x40);
                      i2cDisplayString(&hi2c2, "1 Cup Min.");
                  }
              }
              break;
          case SetTime:   // set alarm - time
              if (matrixButton == '*') {
                  if (digit >= 5) {
                      state = Pump;
                      msCount = 1000;
                      secondCount = 0;
                      promptDisplayed = 0;
                      digit = 0;
                      alarm.alarmEnabled = 1;
                      if (alarm.pm) {
                          alarm.hours += 12;
                      }
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      i2cDisplayString(&hi2c2, currentTime.str);
                  }
              }
              if (setTimeRunYet == StateNotRun) {
                  setTimeRunYet = StateRun;
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x40);
                  i2cDisplayString(&hi2c2, "Time: 00:00 AM");
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x46);
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKON, 0x00);
              }
              else {
                  if (matrixButton != '*' && matrixButton != '#' && matrixButton != 0) {
                      unsigned char num[2];
                      sprintf(&num, "%c", matrixButton);
                      switch (digit) {
                          case 1:
                              if (matrixButton <= '1') {    // only 0 or 1 for hours tens digit
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  i2cDisplayString(&hi2c2, num);
                                  alarm.hours += 10*atoi(num);
                                  digit++;
                              }
                              break;
                          case 2:
                              if ((alarm.hours < 10 || matrixButton <= '2') && !(alarm.hours == 0 && matrixButton == '0')) {
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  i2cDisplayString(&hi2c2, num);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  alarm.hours += atoi(num);
                                  digit++;
                              }
                              break;
                          case 3:
                              if (matrixButton < '6') {
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  i2cDisplayString(&hi2c2, num);
                                  alarm.minutes += 10*atoi(num);
                                  digit++;
                              }
                              break;
                          case 4:
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                              i2cDisplayString(&hi2c2, num);
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                              alarm.minutes += atoi(num);
                              digit++;
                              break;
                          case 5:
                              if (matrixButton != '*') {
                                  if (alarm.pm) {
                                      alarm.pm = 0;
                                      sprintf(&num, "%c", 'A');
                                  }
                                  else {
                                      alarm.pm = 1;
                                      sprintf(&num, "%c", 'P');
                                  }
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  i2cDisplayString(&hi2c2, num);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORLEFT, 0x00);
                              }
                              break;
                          default:
                              break;
                      }
                  }
              }
              break;
          case Pump:   // pump
              if (pumpRunYet == StateNotRun) {
                  pumpRunYet = StateRun;
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                  unsigned char message[16];
                  sprintf(&message, "%d cups, start at", cups);
                  i2cDisplayString(&hi2c2, message);
                  HAL_Delay(5);
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x40);
                  formatTime(&alarm);
                  i2cDisplayString(&hi2c2, alarm.str);
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 125); // question 1
              }
//              if (cups <= 0) {
//                  state = 1;
//              }
              break;
          case StartBoil:   // start water
              if (startBoilRunYet == StateNotRun) {
                  startBoilRunYet = StateRun;
                  servoDown();
              }
              if (!buttonPress) {
                  state = SwitchReset;
              }
              break;
          case SwitchReset:   // water started
              if (switchResetRunYet == StateNotRun) {
                  switchResetRunYet = StateRun;
                  HAL_Delay(1500);      // fix
                  servoUp();
                  updateDACEnable = 1;
                  dacStage = 2;
              }
              if (buttonPress) {
                  state = WaterReady;
              }
              break;
          case WaterReady:   // water ready
              if (waterReadyRunYet == StateNotRun) {
                  waterReadyRunYet = StateRun;
                  updateDACEnable = 1;
                  dacStage = 3;
                  state = Wait;
                  alarm.hours = 0;
                  alarm.minutes = 0;
                  alarm.pm = 0;
                  alarm.alarmEnabled = 0;
                  setCupsRunYet = StateNotRun;
                  errorRunYet = StateNotRun;
                  setTimeRunYet = StateNotRun;
                  pumpRunYet = StateNotRun;
                  startBoilRunYet = StateNotRun;
                  switchResetRunYet = StateNotRun;
                  waterReadyRunYet = StateNotRun;
                  cups = 0;
                  pumpCups = 0;
                  digit = 0;
              }
              break;
      }

      // end state machine

/* CODE FOR SPEAKER: state machine should control stage variable...
 * dacStage 1 = powerup
 * dacStage 2 = tea start
 * dacStage 3 = tea finish
 */
      if(updateDAC && updateDACEnable){
          updateDAC = 0;
          if(dacStage == 3){
              DoneGetState();
              //FFGetState();
              DoneSetWave();
              DoneCheckWave();
              //FFSetWave();
              //FFCheckWave();
          }
          if(dacStage == 1 || dacStage == 2){
              triGetState(dacStage);
              waveVal = triSetWave(waveVal);
          }
      }
      /*END CODE FOR SPEAKER*/
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
