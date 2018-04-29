/*
 * time.c
 *
 *  Created on: Apr 22, 2018
 *      Author: david
 */

#include "time.h"

uint8_t timeEqual(Time time1, Time time2) {
    // function to check if two times are equal, specifically current time and alarm time usually
    if (!(time1.pm ^ time2.pm)) {   // make sure the pm flags match
        if (time1.hours == time2.hours) {   // make sure the hours match
            if (time1.minutes == time2.minutes) {   // make sure the minutes match
                return 1;   // if all three are true, return true
            }
        }
    }
    return 0;   // otherwise, return false
}

void formatTime(Time * time) {
    // function to generate a string representation of a given time
    if (time->hours > 12) {     // handles 24 hour clock conversion
        time->hours -= 12;      // if pm, subtract 12 from hours
        time->pm = 1;           // set pm flag appropriately
        sprintf(time->str, "%d%d:%d%d PM", time->hours / 10, time->hours % 10, time->minutes / 10, time->minutes % 10);     // store time into string
    }
    else {
        time->pm = 0;   // otherwise it's am
        sprintf(time->str, "%d%d:%d%d AM", time->hours / 10, time->hours % 10, time->minutes / 10, time->minutes % 10);     // store time into string
    }
    if (time->hours / 10 == 0) {        // removes leading '0' from hours, if present
        (time->str)[0] = ' ';
    }
}
