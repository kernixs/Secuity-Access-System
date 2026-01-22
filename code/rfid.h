/*
 * File: rfid.h
 * Author: Yusuf Mahamud
 * Project: RFID Security Access System
 * Description: Header file for the MFRC522 RFID reader module
 */

#ifndef RFID_H
#define RFID_H

#include <stdint.h>

void rfid_init(void);
int rfid_get_card_uid(uint8_t *buffer);

#endif /* RFID_H */