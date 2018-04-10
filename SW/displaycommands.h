/*
 * displaycommands.h
 *
 *  Created on: Apr 8, 2018
 *      Author: david
 */

#ifndef DISPLAYCOMMANDS_H_
#define DISPLAYCOMMANDS_H_

uint8_t display_on           =  0;
uint8_t display_off          =  1;
uint8_t display_setCursor    =  2;
uint8_t display_cursorHome   =  3;
uint8_t display_underlineOn  =  4;
uint8_t display_underlineOff =  5;
uint8_t display_cursorLeft   =  6;
uint8_t display_cursorRight  =  7;
uint8_t display_blinkOn      =  8;
uint8_t display_blinkOff     =  9;
uint8_t display_backspace    = 10;
uint8_t display_clearScreen  = 11;
uint8_t display_setContrast  = 12;
uint8_t display_setBacklight = 13;
uint8_t display_loadCustom   = 14;
uint8_t display_moveLeft     = 15;
uint8_t display_moveRight    = 16;
uint8_t display_setAddress   = 17;
uint8_t display_showFirmware = 18;
uint8_t display_showAddress  = 19;
uint8_t display_commandBits[20] = {0x41, 0x42, 0x45, 0x46, 0x47, 0x48, 0x49,
                                   0x4A, 0x4B, 0x4C, 0x4E, 0x51, 0x52, 0x53,
                                   0x54, 0x55, 0x56, 0x62, 0x70, 0x72};
uint8_t display_delays[20] = {0, 0, 0, 2, 2, 2, 0, 0, 0, 0, 0, 2, 1, 0, 1, 0,
                              0, 3, 4, 4};

void transmitDisplayCommand(I2C_HandleTypeDef * hi2c, uint8_t command, uint8_t param);

void transmitDisplayCommand(I2C_HandleTypeDef * hi2c, uint8_t command, uint8_t param) {
    if (command == 2 || command == 12 || command == 13 || command == 17) {
        uint8_t pData[3] = {0xFE, display_commandBits[command], param};
        HAL_I2C_Master_Transmit_IT(hi2c, 80, &pData, 3);
        HAL_Delay(display_delays[command]);
    }
    else if (command != display_loadCustom) {
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

#endif /* DISPLAYCOMMANDS_H_ */
