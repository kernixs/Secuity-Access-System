/*
 * File:   main.c
 * Author: Yusuf Mahamud, Alamin Suliman
 * Project: RFID Security Access System
 * Description: Program entry point, implements the main event loop
 */

#include "xc.h"
#include "lcd.h"
#include "rtc.h"
#include "pic24.h"
#include "rfid.h"
#include "keypad.h"          // Include keypad functions
#include "led_control.h"     // Include LED control functions
#define FCY 16000000UL
#include <libpic30.h>

// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)

// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                   // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))

int main(void) {
    pic24_init();                  // Initialize PIC24 system
    lcd_init();                    // Initialize LCD display
    rtc_init();                    // Initialize RTC (Real-Time Clock)
    rfid_init();                   // Initialize RFID reader
    initPinSystem();               // Initialize keypad and PIN entry system
    initLEDs();                    // Initialize LED indicators (RED on RB5, GREEN on RA4)
    
    rtc_time_t time = {0};         // Structure to hold current time from RTC
    char uid_buf[5] = {0};         // Buffer to store RFID card UID (4 bytes + null)
    unsigned char key;             // Variable to store pressed key from keypad
    int cardScanned = 0;           // Flag to prevent multiple scans
    
    while (1) {
        key = readPins();          // Scan keypad matrix for key press
                                   // Returns '0'-'9', 'A'-'F', or '\0' if no key
        
        if (key != '\0') {         // If a valid key was pressed
            processKeyPress(key);  // Process the key (store digit, validate PIN)
                                   // This function handles PIN validation
            
            // Check if correct PIN was just entered
            if (getPinAccepted()) { // If correct PIN ("1234") was entered
                redLED_OFF();       // Turn off RED LED (PIN accepted)
                cardScanned = 0;    // Reset card scanned flag for this session
            }
            
            // Wait for key release (debouncing)
            while (readPins() != '\0') { // Keep checking until key is released
                __delay_ms(10);     // Short delay while waiting
            }
            __delay_ms(50);         // Additional debounce delay after release
        }
        
        // Display current time on LCD line 0
        lcd_setCursor(0, 0);        // Set cursor to line 0, position 0
        rtc_get_time(&time);        // Get current time from RTC
        
        lcd_printChar((time.hours / 10) + '0');   // Hours tens digit
        lcd_printChar((time.hours % 10) + '0');   // Hours ones digit
        lcd_printChar(':');                       // Colon separator
        lcd_printChar((time.minutes / 10) + '0'); // Minutes tens digit
        lcd_printChar((time.minutes % 10) + '0'); // Minutes ones digit
        lcd_printChar(':');                       // Colon separator
        lcd_printChar((time.seconds / 10) + '0'); // Seconds tens digit
        lcd_printChar((time.seconds % 10) + '0'); // Seconds ones digit
        
        // Only scan for RFID if correct PIN was entered and no card scanned yet
        lcd_setCursor(0, 1);        // Set cursor to line 1, position 0
        
        if (getPinAccepted() && !cardScanned) {
            // PIN is correct, attempt to read RFID card
            if (rfid_get_card_uid(uid_buf) == 0) {  // If card detected and UID read successfully
                cardScanned = 1;    // Mark that card was scanned
                
                // Display card UID in hexadecimal format
                for (size_t k = 0; k < 4; k++) {
                    uint8_t value = uid_buf[k];
                    for (int i = 1; i >= 0; i--) {
                        uint8_t nibble = (value >> (i * 4)) & 0x0F;
                        lcd_printChar(nibble + (nibble < 10 ? '0' : ('A' - 10)));
                    }
                }
                
                __delay_ms(2000);   // Display card ID for 2 seconds
                
                // Display timestamp
                lcd_setCursor(0, 0);
                lcd_printStr("Time:");
                lcd_setCursor(0, 1);
                lcd_printChar((time.hours / 10) + '0');
                lcd_printChar((time.hours % 10) + '0');
                lcd_printChar(':');
                lcd_printChar((time.minutes / 10) + '0');
                lcd_printChar((time.minutes % 10) + '0');
                lcd_printChar(':');
                lcd_printChar((time.seconds / 10) + '0');
                lcd_printChar((time.seconds % 10) + '0');
                
                __delay_ms(2000);   // Display timestamp for 2 seconds
                
                // Signal successful card scan with GREEN LED
                greenLED_ON();      // Turn on GREEN LED
                __delay_ms(3000);   // Keep GREEN LED on for 3 seconds
                greenLED_OFF();     // Turn off GREEN LED
                
                // Reset system for next user
                resetSystem();      // Reset PIN system (pinAccepted = 0)
                redLED_ON();        // Turn RED LED back on
                cardScanned = 0;    // Reset card scanned flag
                
            } else {
                // No card detected yet, display waiting message
                lcd_printStr("????????");
            }
        } else if (!getPinAccepted()) {
            // PIN not entered yet, display waiting message
            lcd_printStr("Enter PIN");
        } else {
            // Card already scanned, waiting for reset
            lcd_printStr("        ");  // Clear line
        }
        
        // Small delay between loop iterations
        __delay_ms(100);            // 100ms delay to prevent excessive looping
        
        lcd_clear();
    }
    
    return 0;
}