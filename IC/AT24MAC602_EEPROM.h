/*
 * AT24MAC602_EEPROM.h
 *
 *  Created on: 23-08-2013
 *      Author: Bartek
 */

#ifndef AT24MAC602_EEPROM_H_
#define AT24MAC602_EEPROM_H_

#define EEPROM_address 0x50
#define SN_address	   0x58
#define WORD_ADDRESS   0x80

#include "../stdPeriphLibs/lpc_types.h"

extern volatile uint32_t serialNumber[4];

void readSN( void );

void 	EEPROM_write (uint8_t address, uint8_t data);
uint8_t EEPROM_read	 (uint8_t address);

#endif /* AT24MAC602_EEPROM_H_ */
