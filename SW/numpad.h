/*
 * numpad.h
 *
 *  Created on: Apr 17, 2018
 *      Author: david
 */

#ifndef NUMPAD_H_
#define NUMPAD_H_

#define ROW1_PORT	GPIOB
#define ROW2_PORT	GPIOB
#define ROW3_PORT	GPIOB
#define ROW4_PORT	GPIOB
#define COL1_PORT	GPIOB
#define COL2_PORT	GPIOB
#define COL3_PORT	GPIOC
#define ROW1_PIN	GPIO_PIN_10
#define ROW2_PIN	GPIO_PIN_11
#define ROW3_PIN	GPIO_PIN_12
#define ROW4_PIN	GPIO_PIN_13
#define COL1_PIN	GPIO_PIN_14
#define COL2_PIN	GPIO_PIN_15
#define COL3_PIN	GPIO_PIN_6

void matrixDebounce(uint32_t column);
uint32_t matrixButtonDebounce(uint32_t index);

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

uint32_t matrixButtonDebounce(uint32_t index) {
    btnPress[index] = !btnPrevious[index] && btnCurrent[index];
    btnPrevious[index] = btnCurrent[index];
}

#endif /* NUMPAD_H_ */
