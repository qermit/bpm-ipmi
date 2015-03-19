/*
 * ADN4604_ClockManagement.c
 *
 *  Created on: 07-06-2013
 *      Author: Bartek
 */

#include "ADN4604_ClockManagement.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"

#include <stdint.h>
#include <stdio.h>

static void delay( void );

//#define CBM_CFG
//#define MAST_CFG
//#define SI_CFG

uint8_t initClockMgnt( void )
{
    PINSEL_CFG_Type PIN_CFG;

    GPIO_SetDir(0, 0x01<<0, INPUT);
    GPIO_SetDir(0, 0x01<<1, INPUT);

    PIN_CFG.Pinnum      = 0;
    PIN_CFG.Portnum     = 0;
    PIN_CFG.Pinmode     = PINSEL_PINMODE_TRISTATE;
    PIN_CFG.OpenDrain   = PINSEL_PINMODE_OPENDRAIN;
    PIN_CFG.Funcnum     = PINSEL_FUNC_3;

    PINSEL_ConfigPin(&PIN_CFG);

    PIN_CFG.Pinnum = 1;
    PINSEL_ConfigPin(&PIN_CFG);

    I2C_Init(LPC_I2C1, 100000);
    I2C_Cmd (LPC_I2C1, ENABLE);

    I2C_M_SETUP_Type I2C_MasterInitStruct;
    uint8_t  I2C_TxBuffer[20];
    uint8_t* txBuffPtr = I2C_TxBuffer;

    GPIO_SetValue(1, 0x01<<26);

    I2C_MasterInitStruct.sl_addr7bit = ADN_ADDRESS;
    I2C_MasterInitStruct.tx_count = 0;
    I2C_MasterInitStruct.tx_data = txBuffPtr;
    I2C_MasterInitStruct.tx_length = 2;
    I2C_MasterInitStruct.retransmissions_count = 0;
    I2C_MasterInitStruct.retransmissions_max = 5;
    I2C_MasterInitStruct.rx_data = NULL;
    I2C_MasterInitStruct.rx_length = 0;



#ifdef CBM_CFG
  ///////////////////////////////////////////////
  ////// CBM_CFG
  //////////////////////////////////////
  I2C_TxBuffer[0] = XPT_MAP0_C2; // connect out5 to in8 and out 4 to in13
  I2C_TxBuffer[1] = 0x8D;
  I2C_TxBuffer[2] = '\n';
  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = XPT_MAP0_C3; // connect out6 to in13 and out7 to in15
  I2C_TxBuffer[1] = 0xFD;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = XPT_MAP0_C4; // connect out8 to in8 and out9 to in12
  I2C_TxBuffer[1] = 0xC8;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = XPT_MAP0_C5; // connect out10/11 to in13
  I2C_TxBuffer[1] = 0xDD;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = XPT_MAP0_C7; // connect out14/15 to in13
  I2C_TxBuffer[1] = 0xDD;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = XPT_MAP_TABLE_SEL; // select map0
  I2C_TxBuffer[1] = 0x00;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_4; // TX4_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_5; // TX5_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_6; // TX6_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_7; // TX7_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_8; // TX8_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_9; // TX9_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_10; // TX10_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_11; // TX11_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_14; // TX14_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  I2C_TxBuffer[0] = TX_BC_OUT_15; // TX15_ENABLE
  I2C_TxBuffer[1] = 0x30;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);


  I2C_TxBuffer[0] = XPT_UPDATE; // update
  I2C_TxBuffer[1] = 0x01;
  I2C_TxBuffer[2] = '\n';

  delay();

  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

  //////////////////////////////////////////////////////////////////

#else
#ifdef MAST_CFG
    I2C_TxBuffer[0] = XPT_MAP0_C2; // connect out5 to in13 and out 4 to in13
    I2C_TxBuffer[1] = 0xDD;
    I2C_TxBuffer[2] = '\n';
    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C3; // connect out6/7 to in13
    I2C_TxBuffer[1] = 0xDD;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C4; // connect out8 to in8 and out9 to in13
    I2C_TxBuffer[1] = 0xD8;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C5; // connect out10/11 to in13
    I2C_TxBuffer[1] = 0xDD;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C7; // connect out14/15 to in13
    I2C_TxBuffer[1] = 0xDD;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP_TABLE_SEL; // select map0
    I2C_TxBuffer[1] = 0x00;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_4; // TX4_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_5; // TX5_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_6; // TX6_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_7; // TX7_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_8; // TX8_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_9; // TX9_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_10; // TX10_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_11; // TX11_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_14; // TX14_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_15; // TX15_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);


    I2C_TxBuffer[0] = XPT_UPDATE; // update
    I2C_TxBuffer[1] = 0x01;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
#else
    //DEFAULT
    I2C_TxBuffer[0] = XPT_MAP0_C2; // connect out5 to in8 and out 4 to in13
    I2C_TxBuffer[1] = 0x8D;
    I2C_TxBuffer[2] = '\n';
    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C3; // connect out6/7 to in8
    I2C_TxBuffer[1] = 0x88;//0xDD;//0x88; --- out 6 Adrian clock ;) out 7 Marek clok
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C4; // connect out8/9 to in8
    I2C_TxBuffer[1] = 0x88;//0xDD;//0x88;      // -- out 9 WZ 156.25 output ;)
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C5; // connect out10/11 to in13
    I2C_TxBuffer[1] = 0xDD;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP0_C7; // connect out14/15 to in13
    I2C_TxBuffer[1] = 0xDD;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = XPT_MAP_TABLE_SEL; // select map0
    I2C_TxBuffer[1] = 0x00;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_4; // TX4_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_5; // TX5_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_6; // TX6_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_7; // TX7_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_8; // TX8_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_9; // TX9_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_10; // TX10_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_11; // TX11_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_14; // TX14_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

    I2C_TxBuffer[0] = TX_BC_OUT_15; // TX15_ENABLE
    I2C_TxBuffer[1] = 0x30;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);


    I2C_TxBuffer[0] = XPT_UPDATE; // update
    I2C_TxBuffer[1] = 0x01;
    I2C_TxBuffer[2] = '\n';

    delay();

    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
#endif;
#endif;


#ifdef SI_CFG
    //// SI cfg
        //i2c2 declaration
        GPIO_SetDir(0, 0x01<<10, INPUT);
        GPIO_SetDir(0, 0x01<<11, INPUT);

        PIN_CFG.Pinnum = 10;
        PIN_CFG.OpenDrain   = PINSEL_PINMODE_OPENDRAIN;
        PIN_CFG.Funcnum     = PINSEL_FUNC_2;
        PINSEL_ConfigPin(&PIN_CFG);

        PIN_CFG.Pinnum = 11;
        PINSEL_ConfigPin(&PIN_CFG);

        I2C_Init(LPC_I2C2, 100000);
        I2C_Cmd (LPC_I2C2, ENABLE);


        I2C_TxBuffer[0] = 0x0A;                     //using 2nd channel of multiplexer
        I2C_TxBuffer[1] = '\n';

        uint8_t rxbuff [10];

        I2C_MasterInitStruct.sl_addr7bit = 0x70;
        I2C_MasterInitStruct.tx_count = 0;
        I2C_MasterInitStruct.tx_data = &I2C_TxBuffer[0];
        I2C_MasterInitStruct.tx_length = 1;
        I2C_MasterInitStruct.retransmissions_count = 0;
        I2C_MasterInitStruct.retransmissions_max = 5;
        I2C_MasterInitStruct.rx_data = &rxbuff[0];
        I2C_MasterInitStruct.rx_length = 0;

        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();


        I2C_TxBuffer[0] = 0x89;             //freeze DCO = 1
        I2C_TxBuffer[1] = 0x10;
        I2C_TxBuffer[2] = '\n';


        I2C_MasterInitStruct.sl_addr7bit = 0x55;    //SiLab address
        I2C_MasterInitStruct.tx_count = 0;
        I2C_MasterInitStruct.tx_data = &I2C_TxBuffer[0];
        I2C_MasterInitStruct.tx_length = 2;
        I2C_MasterInitStruct.retransmissions_count = 0;
        I2C_MasterInitStruct.retransmissions_max = 5;
        I2C_MasterInitStruct.rx_data = &rxbuff[0];
        I2C_MasterInitStruct.rx_length = 0;

        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0x7;                      // Register
        I2C_TxBuffer[1] = 0x01;                     // HS_DIV = 000b;   N1[6:2] = 00001b;
        I2C_TxBuffer[2] = '\n';

        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0x8;                      // Register
        I2C_TxBuffer[1] = 0xC2;                     // N1[1:0] = 11b; RFREQ[37:32] = 000010b;
        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0x9;                      // Register
        I2C_TxBuffer[1] = 0xBC;                     // RFREQ[31:24] = 0xBC;
        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0xA;                       // Register
        I2C_TxBuffer[1] = 0x01;                      // RFREQ[23:16] = 0x1E;
        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0xB;                       // Register
        I2C_TxBuffer[1] = 0x1E;                      // RFREQ[15:8] = 0x1E;
        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0xC;                       // Register
        I2C_TxBuffer[1] = 0xB8;                      // RFREQ[7:0] = 0xB8;
        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
    //
        delay();

        I2C_TxBuffer[0] = 0x89;                     //Register
        I2C_TxBuffer[1] = 0x00;                     //Freeze DCO = 0

        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();

        I2C_TxBuffer[0] = 0x87;                     //Register
        I2C_TxBuffer[1] = 0x40;                     //Assert NewFreq bit = 1

        I2C_MasterTransferData(LPC_I2C2, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

        delay();
#endif

    return (1);
}


static void delay(  ) {
uint16_t i = 0;
uint8_t z = 0;

	for(i=0; i<20000; i++)
	{
		for(z=0; z<100; z++)
		{

		}
	}

}


//#ifndef MAST_CFG
//#ifdef DEFAULT_CFG
//    I2C_TxBuffer[0] = XPT_MAP0_C2; // connect out5 to in8 and out 4 to in13
//    I2C_TxBuffer[1] = 0x8E;
//    I2C_TxBuffer[2] = '\n';
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C3; // connect out6/7 to in8
//    I2C_TxBuffer[1] = 0xDD;//0xDD;//0x88; --- out 6 Adrian clock ;) out 7 Marek clok
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C4; // connect out8/9 to in8
//    I2C_TxBuffer[1] = 0xF8;        // -- out 9 WZ 156.25 output ;)
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C5; // connect out10/11 to in13
//    I2C_TxBuffer[1] = 0xDD;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C7; // connect out14/15 to in13
//    I2C_TxBuffer[1] = 0xDD;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP_TABLE_SEL; // select map0
//    I2C_TxBuffer[1] = 0x00;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_4; // TX4_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_5; // TX5_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_6; // TX6_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_7; // TX7_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_8; // TX8_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_9; // TX9_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_10; // TX10_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_11; // TX11_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_14; // TX14_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_15; // TX15_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//
//    I2C_TxBuffer[0] = XPT_UPDATE; // update
//    I2C_TxBuffer[1] = 0x01;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//#endif
//#elsif MAST_CFG
//    I2C_TxBuffer[0] = XPT_MAP0_C2; // connect out5 to in13 and out 4 to in13
//    I2C_TxBuffer[1] = 0xDD;
//    I2C_TxBuffer[2] = '\n';
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C3; // connect out6/7 to in13
//    I2C_TxBuffer[1] = 0xDD;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C4; // connect out8 to in8 and out9 to in13
//    I2C_TxBuffer[1] = 0xD8;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C5; // connect out10/11 to in13
//    I2C_TxBuffer[1] = 0xDD;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP0_C7; // connect out14/15 to in13
//    I2C_TxBuffer[1] = 0xDD;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = XPT_MAP_TABLE_SEL; // select map0
//    I2C_TxBuffer[1] = 0x00;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_4; // TX4_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_5; // TX5_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_6; // TX6_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_7; // TX7_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_8; // TX8_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_9; // TX9_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_10; // TX10_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_11; // TX11_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_14; // TX14_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//    I2C_TxBuffer[0] = TX_BC_OUT_15; // TX15_ENABLE
//    I2C_TxBuffer[1] = 0x30;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//
//    I2C_TxBuffer[0] = XPT_UPDATE; // update
//    I2C_TxBuffer[1] = 0x01;
//    I2C_TxBuffer[2] = '\n';
//
//    delay();
//
//    I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);
//
//#endif;

