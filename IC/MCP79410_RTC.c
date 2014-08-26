/*
 * MCP79410_RTC.c
 *
 *  Created on: 21-08-2013
 *      Author: Bartek
 */

#define MCP79410_EEPROM 0x57
#define MCP79410_RTC 	0x6f

#include <stdio.h>

#include "MCP79410_RTC.h"
#include "../stdPeriphLibs/lpc_types.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"

static uint8_t bin2BCD( uint8_t  val );
static uint8_t BCD2bin(	uint8_t  val );

static void initDefStruct( uint8_t* structPtr );


void MCP79410_Configure(  )
{
	I2C_M_SETUP_Type I2C_MasterInitStruct;
	RTC_struct_T rtc_cfg_struct;

	uint8_t* cfgPtr = &rtc_cfg_struct;


	initDefStruct(cfgPtr);

	rtc_cfg_struct.st 		= 1;
	rtc_cfg_struct.extosc 	        = 1;
	rtc_cfg_struct.out		= 0;
	rtc_cfg_struct.day		= 3;
	rtc_cfg_struct.hour		= 2;

	I2C_MasterInitStruct.sl_addr7bit 			= MCP79410_RTC;
	I2C_MasterInitStruct.tx_count 				= 0;
	I2C_MasterInitStruct.tx_data 				= cfgPtr;
	I2C_MasterInitStruct.tx_length 				= STRUCT_FIELD;
	I2C_MasterInitStruct.retransmissions_count 	        = 0;
	I2C_MasterInitStruct.retransmissions_max 	        = 5;
	I2C_MasterInitStruct.rx_data   				= NULL;
	I2C_MasterInitStruct.rx_length 				= 0;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

	uint8_t valbin = 15;
	uint8_t valBCD = 35;
	uint8_t calVAL1 = 0;
	uint8_t calVAL2 = 0;

	calVAL1 = bin2BCD(valbin);
	calVAL2 = BCD2bin(valBCD);

}

void MCP79410_GetData(  )
{
	I2C_M_SETUP_Type I2C_MasterInitStruct;
	RTC_struct_T rtc_cfg_struct;

	uint8_t* cfgPtr = &rtc_cfg_struct+1;

	uint8_t I2C_TxBuffer[10];
	uint8_t * txBuffPtr = I2C_TxBuffer;


	initDefStruct(cfgPtr);

	I2C_TxBuffer[0] = 0x00;

	I2C_MasterInitStruct.sl_addr7bit 			= MCP79410_RTC;
	I2C_MasterInitStruct.tx_count 				= 0;
	I2C_MasterInitStruct.tx_data 				= txBuffPtr;
	I2C_MasterInitStruct.tx_length 				= 1;
	I2C_MasterInitStruct.retransmissions_count 	        = 0;
	I2C_MasterInitStruct.retransmissions_max 	        = 5;
	I2C_MasterInitStruct.rx_data   				= cfgPtr;
	I2C_MasterInitStruct.rx_length 				= STRUCT_FIELD-1;

	I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

}


static void initDefStruct( uint8_t* structPtr )
{
	uint8_t i = STRUCT_FIELD;
		for(;i>0;i--)
		{
			*structPtr = 0x00;
			structPtr++;
		}
}

static uint8_t bin2BCD( uint8_t  val )
{
	uint8_t retVal = 0;
	retVal = ( (val/10) << 4 )  |  ( 0x0f & (val%10) );
	return (retVal);
}

static uint8_t BCD2bin(	uint8_t  val )
{
	uint8_t retVal = 0;
	retVal = ( 10 * ( (val & 0xf0) >>4 ) ) + ( val & 0x0F );
	return (retVal);
}
