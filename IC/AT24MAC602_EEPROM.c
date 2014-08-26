/*
 * AT24MAC602_EEPROM.c
 *
 *  Created on: 23-08-2013
 *      Author: Bartek
 */

#include "AT24MAC602_EEPROM.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"


volatile uint32_t serialNumber[4];

void readSN(  )
{
	I2C_M_SETUP_Type I2C_MasterInitStruct;

	uint8_t i = 0;

	uint8_t I2C_TxBuffer[4];
	uint8_t* txBuffPtr = &I2C_TxBuffer[0];

	uint8_t I2C_RxBuffer[16];
	uint8_t* rxBuffPtr = &I2C_RxBuffer[0];

	I2C_TxBuffer[0] = WORD_ADDRESS;

	I2C_MasterInitStruct.sl_addr7bit 			= SN_address;
	I2C_MasterInitStruct.tx_count 				= 0;
	I2C_MasterInitStruct.tx_data 				= txBuffPtr;
	I2C_MasterInitStruct.tx_length 				= 1;
	I2C_MasterInitStruct.retransmissions_count 	= 0;
	I2C_MasterInitStruct.retransmissions_max 	= 5;
	I2C_MasterInitStruct.rx_data   				= rxBuffPtr;
	I2C_MasterInitStruct.rx_length 				= 16;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);


	for(	; i<4; i++)
	{
		serialNumber[i] = (uint32_t)( ( I2C_RxBuffer[(i*4)+3] << 24 ) | ( I2C_RxBuffer[(i*4)+2] << 16 ) |
					( I2C_RxBuffer[(i*4)+1] << 8 ) | ( I2C_RxBuffer[i*4] ) );
	}

}

void 	EEPROM_write (uint8_t address, uint8_t data)
{
	I2C_M_SETUP_Type I2C_MasterInitStruct;
	uint8_t I2C_TxBuffer[4];
	uint8_t* txBuffPtr = I2C_TxBuffer;


	I2C_TxBuffer[0] = address;
	I2C_TxBuffer[1] = data;

	I2C_MasterInitStruct.sl_addr7bit 			= EEPROM_address;
	I2C_MasterInitStruct.tx_count 				= 0;
	I2C_MasterInitStruct.tx_data 				= txBuffPtr;
	I2C_MasterInitStruct.tx_length 				= 2;
	I2C_MasterInitStruct.retransmissions_count 	= 0;
	I2C_MasterInitStruct.retransmissions_max 	= 5;
	I2C_MasterInitStruct.rx_data   				= NULL;
	I2C_MasterInitStruct.rx_length 				= 0;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
}

uint8_t EEPROM_read	 (uint8_t address)
{
	I2C_M_SETUP_Type I2C_MasterInitStruct;

	uint8_t I2C_TxBuffer[4];
	uint8_t* txBuffPtr = I2C_TxBuffer;

	uint8_t I2C_RxBuffer[4];
	uint8_t* rxBuffPtr = I2C_RxBuffer;

	I2C_TxBuffer[0] = address;

	I2C_MasterInitStruct.sl_addr7bit 			= EEPROM_address;
	I2C_MasterInitStruct.tx_count 				= 0;
	I2C_MasterInitStruct.tx_data 				= txBuffPtr;
	I2C_MasterInitStruct.tx_length 				= 1;
	I2C_MasterInitStruct.retransmissions_count 	= 0;
	I2C_MasterInitStruct.retransmissions_max 	= 5;
	I2C_MasterInitStruct.rx_data   				= rxBuffPtr;
	I2C_MasterInitStruct.rx_length 				= 1;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

	return I2C_RxBuffer[0];
}

