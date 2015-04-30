/*
 * ssp_flash.c
 *
 *  Created on: Apr 29, 2015
 *      Author: Henrique Silva
 */

#include "LPC17xx.h"

#include "../stdPeriphLibs/lpc17xx_ssp.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"

#include "../include/ssp_flash.h"
#include "../include/utils.h"

#include "../include/interruptPriorities.h"

#define SSEL_HIGH()     GPIO_SetValue(0, 1 << 6)
#define SSEL_LOW()      GPIO_ClearValue(0, 1 << 6)

/* Tx buffer */
//static uint8_t Tx_Buf[SSP_BUFFER_SIZE];

/* Rx buffer */
//static uint8_t Rx_Buf[SSP_BUFFER_SIZE];

LPC_SSP_TypeDef * LPC_SSP = (LPC_SSP_TypeDef *) LPC_SSP1;

void ssp_init(void)
{
  //LPC_SSP1
  PINSEL_CFG_Type PinSelCfg;
  SSP_CFG_Type SSP_ConfigStruct;

  /*
  * Initialize SPI pin connect
  * P0.6 - SSEL
  * P0.7 - SCK;
  * P0.8 - MISO
  * P0.9 - MOSI
  */

  // P0_6
  PinSelCfg.Portnum = PINSEL_PORT_0;
  PinSelCfg.Pinnum = PINSEL_PIN_6;
  PinSelCfg.Funcnum = PINSEL_FUNC_2;
  PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinSelCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  PINSEL_ConfigPin(&PinSelCfg);
  // P0_7
  PinSelCfg.Pinnum = PINSEL_PIN_7;
  PINSEL_ConfigPin(&PinSelCfg);
  // P0_8
  PinSelCfg.Pinnum = PINSEL_PIN_8;
  PINSEL_ConfigPin(&PinSelCfg);
  // P0_9
  PinSelCfg.Pinnum = PINSEL_PIN_9;
  PINSEL_ConfigPin(&PinSelCfg);

  SSP_ConfigStructInit(&SSP_ConfigStruct);

  SSP_Init(LPC_SSP, &SSP_ConfigStruct);
  SSP_Cmd(LPC_SSP, ENABLE);
}

N25Q_ReadID_t n25q_readid(void)
{
  uint8_t cmd[5], ret[5];

  cmd[0] = N25Q_READ_ID;

  SSP_DATA_SETUP_Type readid_rq = {0};
  readid_rq.rx_data = ret;
  readid_rq.tx_data = cmd;
  readid_rq.length = sizeof(cmd)/sizeof(cmd[0]);

  //TODO: Check if we really have to force SSEL to 0 here
  SSEL_LOW();
  SSP_ReadWrite(LPC_SSP, &readid_rq, SSP_TRANSFER_POLLING);
  SSEL_HIGH();

  N25Q_ReadID_t n25q_id = {0};
  n25q_id.Man_ID = ret[0];
  n25q_id.Dev_ID.mem_type = ret[1] >> 8;
  n25q_id.Dev_ID.mem_size = ret[1] && 0xFF;

  return n25q_id;
}
