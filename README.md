# Secuity-Access-System
RFID Security Access System with PIN Authentication A dual-factor authentication system using 4-digit PIN entry via keypad and RFID card scanning, implemented on PIC24FJ64GA002 microcontroller.

## Overview
This embedded security system requires both PIN authentication and RFID card verification for access control. The system provides visual feedback through LED indicators and displays access information on an LCD screen with real-time timestamps(RTC).

## Features
Dual Authentication: PIN (keypad) + RFID card verification
Visual Feedback:

- RED LED indicates system locked (waiting for PIN)
- GREEN LED indicates successful card scan


LCD Display: Shows real-time clock and card information
Security Features:

- 1-second delay after wrong PIN (prevents brute-force)
- Automatic system reset after each transaction


Modular Libraries: Reusable keypad and LED control libraries

## System Operation
Start Program
- Initialize system: (Configure keypad, RFID, LEDs, LCD and RTC)
- Display Time on LCD
- Scan Keypad for PIN input (if PIN incorrect - > one second delay, return to
keypad scan
- If PIN Correct: (Turn RED LED OFF, Enable RFID scanning)
- Scan RFID Card: (If no card detected -> continue scanning)
- If Card Detected: (Read UID, Display UID with timestamp on LCD, Green LED
turns ON for 3 seconds)
- Reset System: (Clear PIN buﬀer - > Turn RED LED ON)
- Loop Back to Start

## Documentation
Complete documentation available in `/docs`:
- Keypad Library API Documentation
- LED Control Library Documentation
- RFID Library Documentation
- LCD Library Documentation
- RTC Library DOcumentation
- Wiring Diagrams
- Usage Examples

## Contributors
Alamin Suliman - Keypad, RTC, LED Libraries

- Developed keypad scanning and PIN entry system
- Implemented LED control with visual feedback
- Inetegrated the RTC I2C interface for time stamping on LCD
- Main program logic/coding for pin entry and access to RFID
- Hardware wiring and testing


Yusuf Mahamud - RFID & LCD Display Libraries

- RFID reader SPI communication
- LCD display integration 
- Main program architecture
- Hardware wiring and testing




