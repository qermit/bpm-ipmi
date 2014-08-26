/*
 * MAX6642.c
 *
 *  Created on: 26-08-2013
 *      Author: Bartek
 */



#include "MAX6642.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"


void readMAXTemp (  )
{
	I2C_M_SETUP_Type I2C_MasterInitStruct;

	uint8_t I2C_TxBuffer[10];
	uint8_t * txBuffPtr = I2C_TxBuffer;
	uint8_t I2C_RxBuffer[10];
	uint8_t * rxBuffPtr = I2C_RxBuffer;
	float readTEMP = 0.0;

	I2C_TxBuffer[0] = MAX6642_REMOTE_TEMP_R;

	I2C_MasterInitStruct.sl_addr7bit 		= MAX6642_ADDRESS;
	I2C_MasterInitStruct.tx_count 			= 0;
	I2C_MasterInitStruct.tx_data 			= txBuffPtr;
	I2C_MasterInitStruct.tx_length 			= 1;
	I2C_MasterInitStruct.retransmissions_count 	= 0;
	I2C_MasterInitStruct.retransmissions_max 	= 5;
	I2C_MasterInitStruct.rx_data   			= rxBuffPtr;
	I2C_MasterInitStruct.rx_length 			= 1;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

	I2C_TxBuffer[0] = MAX6642_REMOTE_EXT_R;
	readTEMP = I2C_RxBuffer[0];

	I2C_MasterInitStruct.sl_addr7bit 		= MAX6642_ADDRESS;
	I2C_MasterInitStruct.tx_count 			= 0;
	I2C_MasterInitStruct.tx_data 			= txBuffPtr;
	I2C_MasterInitStruct.tx_length 			= 1;
	I2C_MasterInitStruct.retransmissions_count 	= 0;
	I2C_MasterInitStruct.retransmissions_max 	= 5;
	I2C_MasterInitStruct.rx_data   			= rxBuffPtr;
	I2C_MasterInitStruct.rx_length 			= 1;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

	readTEMP += ( ( (I2C_RxBuffer[0] & 0xC0) >> 6 ) * 0.25);

}

float readFPGATemp (volatile uint8_t* ptr)
{
  float val = (float)(*ptr);
  uint8_t fract_m = *(ptr+1);
  fract_m = ( (fract_m >> 6) & 0x03 );
  float fract = 0.25*( (float)(fract_m) ) ;

  float readTEMP = (float)( val + 8.0 + fract );

  return (readTEMP);
}






