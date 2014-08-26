/*
 * INA220.c
 *
 *  Created on: 01-07-2013
 *      Author: Bartek
 */

#include <stdio.h>

#include "INA220.h"
#include "../include/diagnosticStructure.h"
#include "../include/CardDiagnostic.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"


//static void init_INA_struct(void);
static void delay( void );
//0x19F

void initINAsensors( void )
{
	//init_INA_struct( );

	I2C_M_SETUP_Type I2C_MASTER_SEND;

	uint8_t txbuff[5];


	uint8_t i = 0;
	uint8_t* txPtr = &txbuff[0];

	txbuff[0] = INA220_CFG_REG;
	txbuff[1] = 0x01;
	txbuff[2] = 0x9F;

	I2C_MASTER_SEND.tx_data = txPtr;
	I2C_MASTER_SEND.tx_length = 3;
	I2C_MASTER_SEND.rx_data = NULL;
	I2C_MASTER_SEND.rx_length = 0;

	for( ; i < VOLT_SENSORS_CNT+CURR_SENSORS_CNT-1 ; i++)
	{
		I2C_MASTER_SEND.sl_addr7bit = INA220_ADDR_table[i];
		I2C_MasterTransferData(LPC_I2C1, &I2C_MASTER_SEND, I2C_TRANSFER_POLLING);
		delay( );
	}

}




float get_INNA_curr (volatile uint8_t* ptr )
{

  float retVal = 0.0;

  //shunt voltage
  uint16_t tmpVal = ( (*ptr)<<8 ) | ( *(ptr+1) );

  if( tmpVal & 0x8000 )
    {
      tmpVal = ( ( ~tmpVal ) & 0x7fff );
      retVal = ( -1.0 * (float)(tmpVal));
    }
  else
    {
      retVal = (float)(tmpVal & 0x7fff);
    }

  retVal /= 1000.0;

  return (retVal);
}

float get_INNA_volt (volatile uint8_t* ptr )
{

  float retVal = 0.0;

  uint16_t tmpVal = 0.0;

  //bus volt
  tmpVal = ( (*ptr)<<8 ) | ( *(ptr+1) );
  tmpVal = (tmpVal >> 3);

  retVal = (float)(  tmpVal*0.004 );


  return (retVal);
}



//static void init_INA_struct(void)
//{
//	volatile uint8_t i = 0;
//
//	for(    ; i<INA220_ADDRESS_CNT;i++)
//	{
//		INA220_Struct[i].MeasuredBusVoltage = 0.0;
//		INA220_Struct[i].BusVoltageRange = 32.0;
//		INA220_Struct[i].address = INA220_ADDR1 + i;
//	}
//}


static void delay(  ) {

  volatile uint16_t i = 0;

  volatile uint8_t z = 0;

	for(i=0; i<20000; i++)
	{
		for(z=0; z<100; z++)
		{

		}
	}

}


//float get_INNA_data(uint8_t address)
//{
//      I2C_M_SETUP_Type I2C_MASTER_SEND;
//      float result = 0.0;
//      uint16_t data = 0;
//
//    uint8_t rxbuff[2];
//    rxbuff[0]=0;
//    rxbuff[1]=0;
//    uint8_t txbuff[1];
//    uint8_t* ptrt = &txbuff[0];
//
//    txbuff[0]=1;
//
//      uint8_t* ptr = &rxbuff[0];
//
//      if( (address<INA220_ADDR1) | (address > INA220_ADDR6) )
//        return (1);
//
//      I2C_MASTER_SEND.tx_data = ptrt;
//      I2C_MASTER_SEND.tx_length = 1;
//      I2C_MASTER_SEND.rx_data = ptr;
//      I2C_MASTER_SEND.rx_length = 2;
//      I2C_MASTER_SEND.sl_addr7bit = address;
//
//      I2C_MasterTransferData(LPC_I2C1, &I2C_MASTER_SEND, I2C_TRANSFER_POLLING);
//
//      data = rxbuff[0]<<8 | rxbuff[1];
//
//      result = ((float)data/32000.0)*32.0;
//
//      return (result);
//}//float get_INNA_data(uint8_t address)
//{
//      I2C_M_SETUP_Type I2C_MASTER_SEND;
//      float result = 0.0;
//      uint16_t data = 0;
//
//    uint8_t rxbuff[2];
//    rxbuff[0]=0;
//    rxbuff[1]=0;
//    uint8_t txbuff[1];
//    uint8_t* ptrt = &txbuff[0];
//
//    txbuff[0]=1;
//
//      uint8_t* ptr = &rxbuff[0];
//
//      if( (address<INA220_ADDR1) | (address > INA220_ADDR6) )
//        return (1);
//
//      I2C_MASTER_SEND.tx_data = ptrt;
//      I2C_MASTER_SEND.tx_length = 1;
//      I2C_MASTER_SEND.rx_data = ptr;
//      I2C_MASTER_SEND.rx_length = 2;
//      I2C_MASTER_SEND.sl_addr7bit = address;
//
//      I2C_MasterTransferData(LPC_I2C1, &I2C_MASTER_SEND, I2C_TRANSFER_POLLING);
//
//      data = rxbuff[0]<<8 | rxbuff[1];
//
//      result = ((float)data/32000.0)*32.0;
//
//      return (result);
//}
