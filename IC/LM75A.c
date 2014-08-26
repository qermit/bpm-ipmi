/*
 * LM75A.c
 *
 *  Created on: 10-04-2013
 *      Author: Bartek
 */


//#define ADN_ADDRESS 0x96
//#include "../include/INA220.h"

#include "LM75A.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../stdPeriphLibs/lpc17xx_timer.h"
#include <string.h>


#define BUFF_SIZE 20


float get_LM_temp (volatile uint8_t* ptr)
{
        uint16_t val = ( (*ptr) << 1 ) | (  ( (*(ptr+1)) >> 7 ) && 0x01 );

        return ( (float)( val*0.5 ) );
}










//volatile uint8_t tx_buff[BUFF_SIZE];
//volatile uint8_t rx_buff[BUFF_SIZE];
//static void clrBuff ( void );
//static void calculateTemp( uint8_t sensNR );
//
//static void defStructInit( void );
//
//void initLM75( void )
//{
//	clrBuff();
//	defStructInit();
//}
//
//	/**
//	* LM75 default value
//	* 1. Comparator mode
//	* 2. Tos = 80 C
//	* 3. Thys = 75 C
//	* 4. O.S active low
//	* 5. Pointer = 0x00
//	*/
//static void defStructInit( void )
//{
//	uint8_t i = 0;
//
//	sensorTable[0].address = LM75_1_ADDR;
//	sensorTable[1].address = LM75_2_ADDR;
//	sensorTable[2].address = LM75_3_ADDR;
//	sensorTable[3].address = LM75_4_ADDR;
//
//	for(; i < LM_SENSOR_CNT-1; i++)
//	{
//		sensorTable[i].HYST_Value = 75;
//		sensorTable[i].OS_Value = 80;
//		sensorTable[i].temperature = 40;
//	}
//
//}
//void get_LM75_Temp ( void )
//{
//	I2C_M_SETUP_Type I2C_MASTER_SEND;
//	uint8_t a, b;
//	uint8_t i = 0;
//
//	uint8_t* ptr_t = &tx_buff[0];
//	uint8_t* ptr_r = &rx_buff[0];
//
//	tx_buff[0] = TEMP_REG;
//
//
//	I2C_MASTER_SEND.tx_data = ptr_t;
//	I2C_MASTER_SEND.tx_length = 1;
//	I2C_MASTER_SEND.rx_data = ptr_r;
//	I2C_MASTER_SEND.rx_length = 1;
//
//	for (; i < LM_SENSOR_CNT-1; i++)
//	{
//		I2C_MASTER_SEND.sl_addr7bit = sensorTable[i].address;
//		I2C_MasterTransferData(LPC_I2C1, &I2C_MASTER_SEND, I2C_TRANSFER_POLLING);
//		calculateTemp( i+1 );
//
//	}
//	a = MAX(sensorTable[0].temperature, sensorTable[1].temperature);
//	b = MAX(sensorTable[2].temperature, sensorTable[3].temperature);
//
//	maxTemp = MAX( a, b);
//
//	ambientTemp = (sensorTable[0].temperature + sensorTable[1].temperature + sensorTable[2].temperature + sensorTable[3].temperature)/4;
//
//
//}
//
//static void calculateTemp( uint8_t sensNR )
//{
//	if(sensNR <= LM_SENSOR_CNT)
//	{
//		sensorTable[sensNR-1].temperature = (uint8_t)( (rx_buff[0]<<1)/2 );
//	}
//}
//
//static void clrBuff ( void )
//{
//	uint8_t i;
//	for(i=0; i<BUFF_SIZE; i++)
//	{
//		tx_buff[i] = 0x00;
//	}
//}
//
//uint8_t return_TEMP( temperature_T tempType )
//{
//	if( tempType == AMBIENT )
//	{
//		return (ambientTemp);
//	}
//	else
//	{
//		return (maxTemp);
//	}
//
//}


