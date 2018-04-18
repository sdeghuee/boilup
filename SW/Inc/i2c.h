/**
  ******************************************************************************
  * File Name          : I2C.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */
#define DISPLAY_ON              0
#define DISPLAY_OFF             1
#define DISPLAY_SETCURSOR       2
#define DISPLAY_CURSORHOME      3
#define DISPLAY_UNDERLINEON     4
#define DISPLAY_UNDERLINEOFF    5
#define DISPLAY_CURSORLEFT      6
#define DISPLAY_CURSORRIGHT     7
#define DISPLAY_BLINKON         8
#define DISPLAY_BLINKOFF        9
#define DISPLAY_BACKSPACE      10
#define DISPLAY_CLEARSCREEN    11
#define DISPLAY_SETCONTRAST    12
#define DISPLAY_SETBACKLIGHT   13
#define DISPLAY_LOADCUSTOM     14
#define DISPLAY_MOVELEFT       15
#define DISPLAY_MOVERIGHT      16
#define DISPLAY_SETADDRESS     17
#define DISPLAY_SHOWFIRMWARE   18
#define DISPLAY_SHOWADDRESS    19
//uint8_t display_on           =  0;
//uint8_t display_off          =  1;
//uint8_t display_setCursor    =  2;
//uint8_t display_cursorHome   =  3;
//uint8_t display_underlineOn  =  4;
//uint8_t display_underlineOff =  5;
//uint8_t display_cursorLeft   =  6;
//uint8_t display_cursorRight  =  7;
//uint8_t display_blinkOn      =  8;
//uint8_t display_blinkOff     =  9;
//uint8_t display_backspace    = 10;
//uint8_t display_clearScreen  = 11;
//uint8_t display_setContrast  = 12;
//uint8_t display_setBacklight = 13;
//uint8_t display_loadCustom   = 14;
//uint8_t display_moveLeft     = 15;
//uint8_t display_moveRight    = 16;
//uint8_t display_setAddress   = 17;
//uint8_t display_showFirmware = 18;
//uint8_t display_showAddress  = 19;
extern uint8_t display_commandBits[20];
extern uint8_t display_delays[20];
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
void i2cDisplaySendCommand(I2C_HandleTypeDef * hi2c, uint8_t command, uint8_t param);

void i2cDisplayString(I2C_HandleTypeDef * hi2c, unsigned char * str);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
