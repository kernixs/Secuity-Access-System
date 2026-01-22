/*
 * File: keypad.h
 * Author: Alamin Suliman
 * Project: RFID Security Access System
 * Description: Header file for 4x4 matrix keypad with PIN entry system
 */

#ifndef KEYPAD_H
#define KEYPAD_H

#include <stdint.h>

void initKeyPad(void);
unsigned char readPins(void);
int determineKey(void);
void initPinSystem(void);
void processKeyPress(unsigned char key);
int validatePIN(void);
void clearPIN(void);
int getPinAccepted(void);
void resetSystem(void);

#endif // KEYPAD_H