/*
 * File: led_control.c
 * Author: Alamin Suliman
 * Lab: RFID Security Access System
 * Description: LED control functions for visual feedback red LED(RB5) indicates waiting for correct pin while green LED(RA4) indicate sucessful RFID card scan
 */

#include "xc.h"
#include "led_control.h"
#define FCY 16000000UL
#include <libpic30.h>                //for tghe __delay_ms() function

#define RED_LED LATBbits.LATB5       //define red led pin (RB5)
#define GREEN_LED LATAbits.LATA4     //define green led pin (RA4)

/*
 * Function: initLEDs
 * Description: initializes led pins as outputs and sets initial states
 * red led starts on (waiting for pin) and green led is off
 */
void initLEDs(void)
{
    TRISBbits.TRISB5 = 0;           // Configure RB5 (red led pin) as output
    TRISAbits.TRISA4 = 0;           // Configure RA4 (greeb led pin) as output
    greenLED_OFF();                  // make sure green ledis off initially
    redLED_ON();                     // turn on red led to indicate waiting for pin
}

/*
 * Function: redLED_ON
 * Description: turns on the red led (RB5)
 * red led indicates system is locked and waiting for correct pin entry
 */
void redLED_ON(void)
{
    RED_LED = 1;                     // Set RB5 high
}

/*
 * Function: redLED_OFF
 * Description: turns off the red led(RB5)
 * red led turns off when correct pin is entered
 */
void redLED_OFF(void)
{
    RED_LED = 0;                     // Set RB5 low
}

/*
 * Function: greenLED_ON
 * Description: turns on the green led (RA4)
 * green led indicates successful RFID card scan
 */
void greenLED_ON(void)
{
    GREEN_LED = 1;                   //set RA4 high
}

/*
 * Function: greenLED_OFF
 * Description:turns off the green led (RA4)
 * green led turns off after indication period
 */
void greenLED_OFF(void)
{
    GREEN_LED = 0;                   //set RA4 low
}
