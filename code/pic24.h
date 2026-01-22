/*
 * File: pic24.h
 * Author: Yusuf Mahamud
 * Project: RFID Security Access System
 * Description: Header file for the PIC24 support library (SPI, I2C, pin setup)
 */

#ifndef PIC24_H
#define PIC24_H

#include <stdint.h>

void pic24_init(void);
void pic24_write_i2c(uint8_t addr, uint8_t *buffer, int buflen);
void pic24_read_i2c(uint8_t addr, uint8_t *buffer, int buflen);
uint8_t pic24_exchange_spi(uint8_t data);

#endif /* PIC24_H */