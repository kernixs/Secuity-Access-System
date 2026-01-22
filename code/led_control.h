/*
 * File: led_control.h
 * Author: Alamin Suliman
 * Lab: RFID Security Access System
 * Description: Header file for LED control functions
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdint.h>  

void initLEDs(void);
void redLED_ON(void);
void redLED_OFF(void);
void greenLED_ON(void);
void greenLED_OFF(void);

#endif // LED_CONTROL_H