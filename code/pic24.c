/*
 * File: pic24.c
 * Author: Yusuf Mahamud
 * Project: RFID Security Access System
 * Description: Implementation of the PIC24 support library (SPI, I2C, pin setup)
 */

#include "xc.h"
#include "pic24.h"

// Function: pic24_init
// Description: Configures PIC24 device for operation (GPIO, clock, PPS, I2C, SPI)
// Sets digital pins, maps SPI pins via PPS, and enables I2C/SPI peripherals
// Return: none
void pic24_init(void)
{
    // Setup GPIO pins and the system clock.
    TRISBbits.TRISB6 = 0;            // RB6 as output (e.g., LCD_RST)
    TRISBbits.TRISB3 = 0;            // RB3 as output (RFID_CS)
    TRISBbits.TRISB4 = 0;            // RB4 as output (RFID_RST)
    LATBbits.LATB3 = 1;              // Deassert CS by default (inactive HIGH)
    LATBbits.LATB4 = 1;              // Hold RFID reset HIGH
    AD1PCFG = 0xffff;                // All AN pins as digital I/O
    _RCDIV = 0;                      // System clock divider = 1:1

    // Setup I2C communication.
    I2C1CONbits.I2CEN = 0;           // Disable I2C module to configure
    I2C1BRG = 199;                   // Baud rate generator (speed depends on Fcy)
    _MI2C1IF = 0;                    // Clear I2C1 master interrupt flag
    I2C1CONbits.I2CEN = 1;           // Enable I2C module

    // Set PPS to SPI mode.
    __builtin_write_OSCCONL(OSCCON & 0xbf); // Unlock PPS
    RPOR6bits.RP13R = 8;             // RP13 -> SCK1OUT
    RPOR6bits.RP12R = 7;             // RP12 -> SDO1
    RPINR20bits.SDI1R = 2;           // RP2 -> SDI1
    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS

    // Setup SPI communication.
    SPI1CON1 = 0;                    // Reset SPI config
    SPI1STAT = 0;                    // Reset SPI status
    SPI1CON1bits.MSTEN = 1;          // Master mode
    SPI1CON1bits.PPRE = 2;           // Primary prescale 4:1
    SPI1CON1bits.SPRE = 5;           // Secondary prescale 4:1

    SPI1CON1bits.CKP = 0;            // Idle state low (Mode 0)
    SPI1CON1bits.CKE = 1;            // Transmit on active-to-idle (Mode 0 on PIC)
    SPI1STATbits.SPIEN = 1;          // Enable SPI
}

// Function: pic24_write_i2c_raw
// Description: Transmits a single byte on the I2C bus (blocking)
// Input: byte - data byte to transmit
// Return: none
static void pic24_write_i2c_raw(uint8_t byte)
{
    _MI2C1IF = 0;                    // Clear master I2C interrupt flag
    I2C1TRN = byte;                  // Load transmit register
    while (I2C1STATbits.TRSTAT)      // Wait for transmission to start/complete
        ;
    while (!_MI2C1IF)                // Wait for interrupt flag (done)
        ;
}

// Function: pic24_write_i2c
// Description: Transmits an I2C message: START, address, data..., STOP
// Input: addr - 7-bit I2C slave address
//        buffer - pointer to data buffer to send
//        buflen - number of bytes to send
// Return: none
void pic24_write_i2c(uint8_t addr, uint8_t *buffer, int buflen)
{
    // Start I2C transmission.
    I2C1CONbits.SEN = 1;             // Initiate START condition
    while (I2C1CONbits.SEN)          // Wait for START to complete
        ;

    // Send address followed by data bytes.
    pic24_write_i2c_raw(addr << 1);
    for (int i = 0; i < buflen; i++)
    {
        pic24_write_i2c_raw(buffer[i]);
    }

    // End I2C transmission.
    I2C1CONbits.PEN = 1;             // Initiate STOP condition
    while (I2C1CONbits.PEN)          // Wait for STOP to complete
        ;
}

// Function: pic24_read_i2c_raw
// Description: Receives a single byte from the I2C bus and ACK/NACKs it
// Input: last_byte - 1 to NACK after read (end transfer), 0 to ACK (continue)
// Return: the received byte
static uint8_t pic24_read_i2c_raw(int last_byte)
{
    uint8_t byte = 0;

    // Enable receive mode.
    _MI2C1IF = 0;                    // Clear I2C master interrupt flag
    I2C1CONbits.RCEN = 1;            // Enable receive
    while (!I2C1STATbits.RBF)        // Wait until a byte is received
        ;
    byte = I2C1RCV;                  // Read received byte

    // Send NACK to end transmission (otherwise ACK to continue).
    if (last_byte)
    {
        I2C1CONbits.ACKDT = 1;       // NACK
    }
    else
    {
        I2C1CONbits.ACKDT = 0;       // ACK
    }

    // Perform the acknowledge sequence.
    I2C1CONbits.ACKEN = 1;           // Initiate ACK/NACK
    while (I2C1CONbits.ACKEN)        // Wait for completion
        ;

    return byte;                      // Return received byte
}

// Function: pic24_read_i2c
// Description: Reads an I2C message: START, address|READ, read bytes, STOP
// Input: addr - 7-bit I2C slave address
//        buffer - pointer to buffer to store received bytes
//        buflen - number of bytes to read
// Return: none
void pic24_read_i2c(uint8_t addr, uint8_t *buffer, int buflen)
{
    // Start I2C transmission.
    I2C1CONbits.SEN = 1;             // Initiate START condition
    while (I2C1CONbits.SEN)          // Wait for START to complete
        ;

    // Send address with READ bit (LSB = 1)
    pic24_write_i2c_raw((addr << 1) | 1);

    // Read data bytes
    for (int i = 0; i < buflen; i++)
    {
        int is_last_byte = (i == (buflen - 1));

        if (is_last_byte)
        {
            buffer[i] = pic24_read_i2c_raw(1); // NACK on last byte
        }
        else
        {
            buffer[i] = pic24_read_i2c_raw(0); // ACK to continue
        }
    }

    // End I2C transmission.
    I2C1CONbits.PEN = 1;             // Initiate STOP condition
    while (I2C1CONbits.PEN)          // Wait for STOP to complete
        ;
}

// Function: pic24_exchange_spi
// Description: Full-duplex SPI transfer of a single byte
// Input: data - byte to transmit
// Return: byte received simultaneously from the SPI bus
uint8_t pic24_exchange_spi(uint8_t data)
{
    SPI1BUF = data;                  // Write to SPI buffer to start transfer
    while (!SPI1STATbits.SPIRBF)     // Wait until receive buffer is full
        ;
    return (uint8_t)SPI1BUF;         // Read and return received byte
}