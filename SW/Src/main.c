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
int updateDACEnable = 1;
int updateDAC = 0;
int cycles = 0;
int dacStage = 1;
int waveVal = 0;
uint8_t error = 0;
uint8_t digit = 0;
uint8_t requestAlarm = 0;
uint8_t column = 1;
uint8_t skipAlarmCheck = 0;
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
uint32_t pumpCups = 0;
uint32_t msCount = 1000;
uint32_t secondCount = 10;
uint32_t buttonPress = 1;
uint32_t buttonState0 = 1;

uint32_t count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
    if (rxData == 0x4B && receivedO) {  // 0x4B == K
        carriageReturn = 1;     // if 'OK' was received, set carriageReturn
    }
    else if (requestingTime && rxData == 0x4C && receivedC){   // 0x4C == L
        // When 'CL' is received, assume the message is ending in 'CLOSED'
        // Which means the time data from server is returned
        requestingTime = 0;     // so reset the flag for waiting for time
        timeReady = 1;          // tell main loop that time can be parsed now
        carriageReturn = 1;
    }
    else if (rxData == 0x26 && requestingAlarm) {   // 0x26 == &
        // slack command is formatted like "&2 2:00PM&"
        // when & is received and requestingAlarm is still set,
        // then I know that alarm data is about to come,
        // so clear any previous data in the buffer
        for (int i = 0; i < 100; i++) {
            buffer[i] = 0;
        }
        buffer_i = 0;
        requestingAlarm = 0;
    }
    else if (rxData == 0x26) {      // 0x26 == &
        // now I know all the alarm data is received, and the main loop
        // can be told to parse it
        carriageReturn = 1;
        alarmTimeReady = 1;
    }
    else if (rxData == 0x5E) {      // 0x5E == ^
        // if slack doesn't have a new alarm to set a '^' is all that is sent
        // so then everything should be cleared, including
        // the buffer and received string
        for (int i = 0; i < 100; i++) {
            buffer[i] = 0;
            received[i] = 0;
        }
        buffer_i = 0;
        carriageReturn = 1;
    }
    else if (rxData == 0x4F) {      // 0x4F == O
        // if an 'O' is received, remember it so that 'OK' can be checked for
        receivedO = 1;
    }
    else if (rxData == 0x43) {      // 0x43 == C
        // if an 'C' is received, remember it so that 'CL' can be checked for
        receivedC = 1;
    }
    else if (rxData != 0x0D && rxData != 0x0A) {    // 0x0D,0x0A == \n,\r
        // if anything else if received, then store it in buffer
        if (receivedO) {
            // but first add back the potentially missing 'O'
            receivedO = 0;
            buffer[buffer_i++] = 0x4F;
        }
        if (receivedC) {
            // and also add back the potentially missing 'C'
            receivedC = 0;
            buffer[buffer_i++] = 0x43;
        }
        // add the latest character to buffer
        buffer[buffer_i++] = rxData;
    }
    HAL_UART_Receive_IT(&huart1, &rxData, 1);   // ask for next character
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) {
    // when sending USART data, make sure receive buffer is cleared
    for (int i = 0; i < 100; i++) {
        buffer[i] = 0;
    }
    buffer_i = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
    if (htim->Instance == TIM3) {   // TIM3 was our general purpose timer
        msCount--;  // TIM3 was configure to fire every 1ms, this counts each
        updateDAC = 1;
        if (msCount <= 0) {
            // keep track of each second
            msCount = 1000;
            secondCount++;
        }
        if (secondCount == 6) {
            // requestAlarm flag tells main loop to check for slack command
            // this triggers 6 seconds after every time check to make sure
            // the time server had enough time to respond
            requestAlarm = 1;
        }
        if (secondCount == 10) {
            // only ten seconds need to be kept track of, so reset secondCount
            secondCount = 0;
            // every ten seconds the current time should be gotten from the time server
            requestTime();
            // ten seconds was also a good time interval for pumping water
            // 1 cup of water was pumped in ten seconds, and the rate was
            // low enough that it didn't splash going into the kettle
            if (state == Pump || state == StartBoil || state == SwitchReset) {
                // water can only be pumping while in the Pump, StartBoil, or SwitchReset states
                if (pumpCups - 1 <= 0) {
                    // occurs when the desired number of cups have been pumped
                    if (state == Pump) {
                        // if it's not currently time to start boiling water
                        // (in that case state would be StartBoil or SwitchReset),
                        // then go to the wait state, until it is time to start boil
                        state = Wait;
                    }
                    // enough water is pumped, so stop PWM to the motor
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                }
                else {
                    // occurs when more cups still need to be pumped
                    // but decrement how many still need to pump
                    pumpCups--;
                }
            }
        }
        if (msCount % 5 == 0) {
            // every 5ms, check the switch
            // no need to debounce, because button is held in one state
            // also need to recognize both state changes, not just pressing
            buttonPress = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) & 1;
        }
        if (msCount % 2 == 0) {
            // every 2ms, debounce the matrix
            debounce = 1;
        }
        if(updateDACEnable) {
            cycles++;
        }
    }
    if(htim->Instance == TIM14){
        // TIM14 is used to time the DAC audio output
        updateDAC = 1;
    }
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
  HAL_Delay(10000); // ten second startup delay added to give the wifi chip to
  // connect to the AP
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  /* at startup:
   * load custom characters for kettle
   * turn blinking cursor off
   * set backlight brightness to max
   * set put PWM duty cycle to 0, to make sure the pump isn't running
   */
  i2cDisplaySendCommand(&hi2c2, DISPLAY_LOADCUSTOM, 0x00);
  i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETBACKLIGHT, 0x08);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      if (requestAlarm) {
          // happens when it is time to check for a command from slack
          requestAlarm = 0; // reset flag
          if (!skipAlarmCheck) {
              // slack command check will be skipped once, when the finished
              // sound is being played, because HAL_Delay(500) ruins it
              requestingAlarm = 1;  // set other flag, that means alarm data is expected
              // connecting and disconnecting to the server everytime proved problematic
              // this should be revised later though
//              transmitString(&huart1, "AT+CIPSTART=1,\"TCP\",\"http://sdeghuee.pythonanywhere.com\",80\r\n");
//              HAL_Delay(500);
              // send AT command to tell ESP 71 bytes will be send
              transmitString(&huart1, "AT+CIPSEND=1,71\r\n");
              HAL_Delay(500);   // wait for ESP to be ready
              // send GET request to server to get slack command data
              transmitString(&huart1, "GET / HTTP/1.1\r\nHost: sdeghuee.pythonanywhere.com\r\nUser-Agent: test\r\n\r\n");
              // disconnect code, that should also be revised later
//              HAL_Delay(500);
//              transmitString(&huart1, "AT+CIPCLOSE=1\r\n");
          }
          else {
              // now that one slack check was skipped, reset the flag
              skipAlarmCheck = 0;
          }
      }
//      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, buttonPress);  // debugging option
      if (debounce) {
          // when debounce flag is set, debounce the next column, like in lab 8
          debounce = 0;
          matrixDebounce(column++);
          if (column > 3) {
              column = 1;
          }
      }
      if (carriageReturn) {
          // carriageReturn flag from USART received data, meaning
          // the buffer should be checked
          carriageReturn = 0;
          // reset 'O' and 'C' flags to be safe
          receivedO = 0;
          receivedC = 0;
          // save the string in buffer into received
          receiveString();
          if (timeReady) {
              // happens when the time data has been received from the server
              timeReady = 0;
              if (received[0] == 0x00 || received[0] == '0') {
                  // occasionally the time server doesn't send back a response,
                  // this checks for that, and request the time again
                  requestTime();
              }
              else {
                  // if the time server gave back a good time:
                  parseTime(received);  // get rid of extraneous data
                  if (state != SetCups && state != SetTime && state != Pump) {
                      // if the SetCups,SetTime, and Pump states, other information is displayed
                      // but otherwise, the current time should be shown on the display
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      i2cDisplayString(&hi2c2, currentTime.str);
                  }
                  if (timeEqual(currentTime, alarm)) {
                      // now that the latest time has been received, the kettle
                      // should check if it's time to start boiling
                      if (alarm.alarmEnabled) {
                          // only start the alarm once, since the time will be
                          // equal 6 times in a minute (new time every 10 seconds)
                          alarm.alarmEnabled = 0;
                          // go to the start for starting the water heating
                          state = StartBoil;
                      }
                  }
              }
          }
          else if (alarmTimeReady) {
              // otherwise, if a slack command is received
              alarmTimeReady = 0;
              // store the received command into the proper variables
              parseAlarm(received);
              if (state == Wait) {
                  state = Pump;
                  // reset the time trackers, so that water pumps for tens seconds
                  msCount = 1000;
                  secondCount = 0;
                  digit = 0;
                  alarm.alarmEnabled = 1;   // enable alarm
              }
              // reset all the first run states
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
              // if time isn't ready and slack command isn't ready, then
              // received data wasn't useful, so clear it
              for (int i = 0; i < 100; i++) {
                  received[i] = 0;
              }
          }
      }
      unsigned char matrixButton = 0;   // stores currently pressed numpad button
      for (uint32_t index = 0; index < 12; index++) {
          // check for button press at every index on numpad
          if (btnPress[index] == 1) {
              // one of the buttons was pressed
              btnPress[index] = 0;
              matrixButton = matrixGetButton(index);
              if (matrixButton == '#') {
                  // '#' is the button for cancel or clearing
                  // can be used as an emergency shutoff for pump
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                  if (!error) {
                    // hitting any button while on the error screen, should only
                    // go back to input, including '#'
                    /*
                     * all of these are reseting any inputs, including:
                     * state reset to wait
                     * clear the currently set alarm
                     * reset all the states to not run yet
                     * clear cups
                     * reset digit count for SetTime state
                     * ask for the current time again, and clear screen
                     */
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
                    requestTime();
                    i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                    i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
                  }
              }
              if (error) {
                  // pressing any button while on the error screen clears the error
                  error = 0;
              }
          }
      }

      // begin state machine
      switch (state) {
          case Startup:   // startup
              // on start, set the ESP to have multiple connections (one for time, one for slac
              transmitString(&huart1, "AT+CIPMUX=1\r\n");
              // wait for setting to take place
              HAL_Delay(1000);
              // then connect to the server for checking slack commands
              transmitString(&huart1, "AT+CIPSTART=1,\"TCP\",\"http://sdeghuee.pythonanywhere.com\",80\r\n");
              updateDACEnable = 1;  // flag for playing a sound
              dacStage = 1; // song to play, in this case the startup beeps
              servoUp();    // reset the servo to be not pressing the on switch
              state = Wait; // go to the wait state
              break;
          case Wait:   // waiting
              if (matrixButton == '*') {
                  // when '*' is pressed, the kettle will prompt for cups (in the next state)
                  state = SetCups;
              }
              break;
          case SetCups:   // set alarm - cups
              // this state prompts for cups
              if (matrixButton == '*') {
                  // when '*' is pressed, the kettle should go to the next state if the number of cups is valid
                  if (cups > 0 && !error) {
                      // checks that a number was pressed first, and that the error screen isn't showing
                      state = SetTime;  // next state
                      // clear alarm time
                      alarm.hours = 0;
                      alarm.minutes = 0;
                      alarm.pm = 0;
                      // reset digit for inputing time in next state
                      digit = 1;
                  }
              }
              else if (!error) {
                  // if not in the error state
                  if (setCupsRunYet == StateNotRun) {
                      // happens if this is the first time this state is run,
                      // so the prompt should be displayed
                      setCupsRunYet = StateRun; // indicate the next time in this state isn't the first run
                      // clear screen first
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      // show prompt for cups
                      i2cDisplayString(&hi2c2, "How many cups: ");
                      // set the cursor to blink so user knows input is expected
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKON, 0x00);
                  }
                  else {
                      // otherwise the state has already been run once, so
                      // don't display again, and wait for a button press
                      if (matrixButton != '*' && matrixButton != '#' && matrixButton != 0) {
                          // if a number was pressed
                          if (matrixButton > '0' && matrixButton <= '4') {  // 1 liter kettle
                              // only accept 1-4 as cups, because 5 cups is more that one liter
                              if (cups > 0) {
                                  // if a number was already given for cups, remove it
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                              }
                              unsigned char num[2];
                              sprintf(&num, "%c", matrixButton);
                              // display the number of cups
                              i2cDisplayString(&hi2c2, num);
                              // save number of cups
                              cups = atoi(num);
                              pumpCups = cups;
                              // move the cursor to on top of the cups
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x0F);
                          }
                          else {
                              // otherwise 0 or 5-9 was selected, so set error
                              error = 1;
                              errorRunYet = StateNotRun;
                              // clear setCups first run flag so the prompt will redisplay after clearing the error
                              setCupsRunYet = StateNotRun;
                              cups = 0;
                          }
                      }
                  }
              }
              else {
                  // otherwise it's in error mode
                  if (errorRunYet == StateNotRun) {
                      // on first time in error mode, the error message should display
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
                  // pressing '*' moves to the next state after selecting AM/PM
                  if (digit >= 5) {
                      // this is when AM/PM selection is done, so all time data input is done
                      state = Pump; // immediately start pumping after getting all time data
                      // reset time counts, so water pumps for ten seconds
                      msCount = 1000;
                      secondCount = 0;
                      digit = 0;
                      alarm.alarmEnabled = 1;   // enable the alarm
                      if (alarm.pm) {
                          // add 12 hours if PM, so that 24 clock conversion works
                          alarm.hours += 12;
                      }
                      // turn off blinking cursor, because user input is done
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKOFF, 0x00);
                      // clear scren, then show current time
                      i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                      i2cDisplayString(&hi2c2, currentTime.str);
                  }
              }
              if (setTimeRunYet == StateNotRun) {
                  // happens when this is the first time this state is run
                  // so, display the prompt
                  setTimeRunYet = StateRun; // set first run flag
                  // move cursor to second line
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x40);
                  // display empty time input
                  i2cDisplayString(&hi2c2, "Time: 00:00 AM");
                  // move cursor to the tens place on hours digit
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x46);
                  // blink cursor to indicated expecting user input
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BLINKON, 0x00);
              }
              else {
                  if (matrixButton != '*' && matrixButton != '#' && matrixButton != 0) {
                      // happens when a number was pressed
                      unsigned char num[2];
                      // store the number that was pressed in a string
                      sprintf(&num, "%c", matrixButton);
                      switch (digit) {
                      // switch handles which digit is receiving input
                      /*
                       * digit == 1 : hours, tens place
                       * digit == 2 : hours, ones place
                       * digit == 3 : minutes, tens place
                       * digit == 4 : minutes, ones place
                       * digit == 5 : AM/PM
                       */
                          case 1:
                              if (matrixButton <= '1') {
                                  // only accept 0 or 1 for hours tens digit
                                  // no error message to reduce annoyance
                                  // then delete the previous number that was shown
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  // and show the new selection
                                  i2cDisplayString(&hi2c2, num);
                                  // save the number into the alarm time
                                  alarm.hours += 10*atoi(num);
                                  // a valid input moves to the next digit
                                  digit++;
                              }
                              break;
                          case 2:
                              if ((alarm.hours < 10 || matrixButton <= '2') && !(alarm.hours == 0 && matrixButton == '0')) {
                                  // for the hours, ones digit, great than 2 if 0 was in the tens,
                                  // and only accept 0 if 1 was in the tens place
                                  // then delete the previous number that was shown
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  // and show the new selection
                                  i2cDisplayString(&hi2c2, num);
                                  // move the cursor over the semi-colon
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  // save the number into the alarm time
                                  alarm.hours += atoi(num);
                                  // a valid input moves to the next digit
                                  digit++;
                              }
                              break;
                          case 3:
                              if (matrixButton < '6') {
                                  // only accept less than 6 in the minutes,
                                  // ten place, because 3:65 isn't valid
                                  // then delete the previous number that was shown
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  // and show the new selection
                                  i2cDisplayString(&hi2c2, num);
                                  // save the number into the alarm time
                                  alarm.minutes += 10*atoi(num);
                                  // a valid input moves to the next digit
                                  digit++;
                              }
                              break;
                          case 4:
                              // any number is accept in the minutes ones place
                              // delete the previous number that was shown
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                              // and show the new selection
                              i2cDisplayString(&hi2c2, num);
                              // move the cursor to skip over the space
                              i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                              // save the number into the alarm time
                              alarm.minutes += atoi(num);
                              // a valid input moves to the next digit
                              digit++;
                              break;
                          case 5:
                              // digit == 5 is the AM/PM selection
                              if (matrixButton != '*') {
                                  // if a number is pressed, AM/PM should flip
                                  if (alarm.pm) {
                                      // PM was chosen & shown before, so change to AM
                                      alarm.pm = 0;
                                      sprintf(&num, "%c", 'A');
                                  }
                                  else {
                                      // AM was chosen & shown before, so change to PM
                                      alarm.pm = 1;
                                      sprintf(&num, "%c", 'P');
                                  }
                                  // delete the previous A/P
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CURSORRIGHT, 0x00);
                                  i2cDisplaySendCommand(&hi2c2, DISPLAY_BACKSPACE, 0x00);
                                  // print the new selection
                                  i2cDisplayString(&hi2c2, num);
                                  // move cursor back to on top of A/P
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
                  // happens if this is the first time the state is entered
                  pumpRunYet = StateRun;    // clear first run flag
                  // clear display, then print how many cups will boil
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_CLEARSCREEN, 0x00);
                  unsigned char message[16];
                  sprintf(&message, "%d cups, start at", cups);
                  i2cDisplayString(&hi2c2, message);
                  HAL_Delay(5);
                  // move to next line
                  i2cDisplaySendCommand(&hi2c2, DISPLAY_SETCURSOR, 0x40);
                  // use function to put alarm time into a string
                  formatTime(&alarm);
                  // print the time
                  i2cDisplayString(&hi2c2, alarm.str);
                  // set PWM duty cycle so pump runs
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 125);
              }
              break;
          case StartBoil:   // start water
              if (startBoilRunYet == StateNotRun) {
                  // happens if this is the first time the state is entered
                  startBoilRunYet = StateRun;   // clear first run flag
                  // move the servo down so that the switch to boil water is flipped
                  servoDown();
              }
              if (!buttonPress) {
                  // wait until the button is released (meaning the switch is down)
                  // then move to the next state
                  state = SwitchReset;
              }
              break;
          case SwitchReset:   // water started
              if (switchResetRunYet == StateNotRun) {
                  // happens if this is the first time the state is entered
                  switchResetRunYet = StateRun;     // clear first run flag
                  // delay to make sure that servo has had time to go down
                  HAL_Delay(1500);
                  // then move the servo back up, so that the switch can reset
                  servoUp();
                  updateDACEnable = 1;  // tell DAC code that a sound should be played
                  dacStage = 2; // select the water started sound
              }
              if (buttonPress) {
                  // wait until the button is pressed again (meaning the switch has reset, and water is ready)
                  // then go to the next state
                  state = WaterReady;
              }
              break;
          case WaterReady:   // water ready
              if (waterReadyRunYet == StateNotRun) {
                  // happens if this is the first time the state is entered
                  waterReadyRunYet = StateRun;
                  updateDACEnable = 1;  // tell DAC code that a sound should be played
                  dacStage = 3;         // select the water ready sound
                  skipAlarmCheck = 1;   // skip the next slack command check so the sound isn't messed up
                  state = Wait;         // now that water is ready, wait for input
                  // also clear the alarm
                  alarm.hours = 0;
                  alarm.minutes = 0;
                  alarm.pm = 0;
                  alarm.alarmEnabled = 0;
                  // and reset the first run flags for all states
                  setCupsRunYet = StateNotRun;
                  errorRunYet = StateNotRun;
                  setTimeRunYet = StateNotRun;
                  pumpRunYet = StateNotRun;
                  startBoilRunYet = StateNotRun;
                  switchResetRunYet = StateNotRun;
                  waterReadyRunYet = StateNotRun;
                  // reset the cups
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
