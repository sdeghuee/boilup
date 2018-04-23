/*
 * time.c
 *
 *  Created on: Apr 22, 2018
 *      Author: david
 */

#include "time.h"

uint8_t timeEqual(Time time1, Time time2) {
    if (!(time1.pm ^ time2.pm)) {
        if (time1.hours == time2.hours) {
            if (time1.minutes == time2.minutes) {
                return 1;
            }
        }
    }
    return 0;
}

void formatTime(Time * time) {
    if (time->hours > 12) {
        time->hours -= 12;
        time->pm = 1;
        sprintf(time->str, "%d%d:%d%d PM", time->hours / 10, time->hours % 10, time->minutes / 10, time->minutes % 10);
    }
    else {
        time->pm = 0;
        sprintf(time->str, "%d%d:%d%d AM", time->hours / 10, time->hours % 10, time->minutes / 10, time->minutes % 10);
    }
    if (time->hours / 10 == 0) {
        (time->str)[0] = ' ';
    }
}
