/*
 * File: rtc.h
 * Author: Alamin Suliman
 * Project: RFID Security Access System
 * Description: Header file for the DS3231 RTC clock module
 */

#ifndef RTC_H
#define RTC_H

#include <stdint.h>

typedef struct {
    uint8_t seconds;  // 0-59
    uint8_t minutes;  // 0-59
    uint8_t hours;    // 0-23 (24-hour format)
    uint8_t day;      // 1-7 (day of week)
    uint8_t date;     // 1-31
    uint8_t month;    // 1-12
    uint8_t year;     // 0-99 (years since 2000)
} rtc_time_t;

void rtc_init(void);
void rtc_set_time(rtc_time_t *time);
void rtc_get_time(rtc_time_t *time);
float rtc_get_temperature(void);

#endif /* RTC_H */
