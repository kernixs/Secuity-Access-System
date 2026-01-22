/*
 * File: lcd.h
 * Author: Yusuf Mahamud
 * Project: RFID Security Access System
 * Description: Header file for the LCD display module
 */

#ifndef LCD_H
#define LCD_H

void lcd_init(void);
void lcd_setCursor(char x, char y);
void lcd_printChar(char c);
void lcd_printStr(const char *s);
void lcd_clear(void);

#endif /* LCD_H */
