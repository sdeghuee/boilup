/*
 * time.h
 *
 *  Created on: Apr 22, 2018
 *      Author: david
 */

#ifndef TIME_H_
#define TIME_H_

#include "stm32f0xx_hal.h"
#include "main.h"

struct _Time {
    uint32_t hours;
    uint32_t minutes;
    uint8_t pm;
    uint8_t alarmEnabled;
    unsigned char str[9];
} typedef Time;

extern Time currentTime;
extern Time alarm;

uint8_t timeEqual(Time time1, Time time2);
void formatTime(Time * time);

#endif /* TIME_H_ */
