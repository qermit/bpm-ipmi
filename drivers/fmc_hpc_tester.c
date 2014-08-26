/*
 * fmc_hpc_tester.c
 *
 *  Created on: 22-05-2014
 *      Author: Bartek
 */

#include "../include/fmc_hpc_tester.h"

#include "../stdPeriphLibs/lpc17xx_i2c.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../stdPeriphLibs/lpc17xx_ssp.h"
#include "../include/LEDdrivers.h"
#include "../include/CardDiagnostic.h"

static void init_i2c(LPC_I2C_TypeDef* lpc_i2c);
static void readADCdata(LPC_I2C_TypeDef* lpc_i2c);
static void busMux(uint8_t chan, LPC_I2C_TypeDef* lpc_i2c, uint8_t muxADDR);
static void I2CExtender(LPC_I2C_TypeDef* lpc_i2c);
static void I2CExtenderDir(LPC_I2C_TypeDef* lpc_i2c);
static void XPTsw(void);
static void initSPItoFPGA(void);
static void ctrlSPI_CS( FunctionalState newState );
static void sendSPIbuff(uint8_t* ptr);
static uint8_t cmp_ff(uint16_t* tab_ptr, LPC_I2C_TypeDef* lpc_i2c);
static uint8_t cmp_zer(uint16_t* tab_ptr, LPC_I2C_TypeDef* lpc_i2c);
static uint16_t ones_det(uint16_t* tab_ptr);
static void sendSPIdataDiag( uint16_t address, uint32_t data );

#define I2C_MUX_TST_ADDR 0x70
#define I2C_MUX_AFC_ADDR 0x72


#define I2C_0_MUX 0x08
#define I2C_1_MUX 0x09
#define I2C_2_MUX 0x0a
#define I2C_3_MUX 0x0b
#define I2C_4_MUX 0x0c


void test_i2c_chips( )
{
  init_i2c(LPC_I2C1);
  init_i2c(LPC_I2C2);
  initSPItoFPGA();

  busMux(I2C_0_MUX, LPC_I2C1, I2C_MUX_TST_ADDR);//I2C_MUX_AFC_ADDR);
//  busMux(I2C_3_MUX, LPC_I2C1, I2C_MUX_TST_ADDR);

  readADCdata(LPC_I2C1);
  readADCdata(LPC_I2C2);

  I2CExtenderDir(LPC_I2C1);
  I2CExtenderDir(LPC_I2C2);

  I2CExtender(LPC_I2C1);
  I2CExtender(LPC_I2C2);

  //XPTsw();
}

static void init_i2c(LPC_I2C_TypeDef* lpc_i2c)
{
  PINSEL_CFG_Type PIN_CFG;

//  GPIO_SetDir(0, 0x01<<0, INPUT);
//  GPIO_SetDir(0, 0x01<<1, INPUT);
//
//  PIN_CFG.Pinnum = 19;
//  PIN_CFG.Portnum = 0;
//  PIN_CFG.Pinmode = PINSEL_PINMODE_TRISTATE;
//  PIN_CFG.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
//  PIN_CFG.Funcnum = PINSEL_FUNC_3;
//
//  PINSEL_ConfigPin(&PIN_CFG);
//
//  PIN_CFG.Pinnum = 20;
//  //PIN_CFG.Pinnum = 1;
//  PINSEL_ConfigPin(&PIN_CFG);
//
//  PIN_CFG.Pinnum = 0;
//  PIN_CFG.Pinmode = PINSEL_PINMODE_PULLUP;
//  PIN_CFG.OpenDrain = PINSEL_PINMODE_NORMAL;
//  PIN_CFG.Funcnum = PINSEL_FUNC_0;
//  PINSEL_ConfigPin(&PIN_CFG);
//
//  PIN_CFG.Pinnum = 1;
//  PINSEL_ConfigPin(&PIN_CFG);

  PIN_CFG.Pinnum = 1;
  PIN_CFG.Portnum = 0;
  PIN_CFG.Pinmode = PINSEL_PINMODE_TRISTATE;
  PIN_CFG.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
  PIN_CFG.Funcnum = PINSEL_FUNC_3;

  PINSEL_ConfigPin(&PIN_CFG);

  PIN_CFG.Pinnum = 0;
  //PIN_CFG.Pinnum = 1;
  PINSEL_ConfigPin(&PIN_CFG);

  PIN_CFG.Pinnum = 19;
  PIN_CFG.Pinmode = PINSEL_PINMODE_PULLUP;
  PIN_CFG.OpenDrain = PINSEL_PINMODE_NORMAL;
  PIN_CFG.Funcnum = PINSEL_FUNC_0;
  PINSEL_ConfigPin(&PIN_CFG);

  PIN_CFG.Pinnum = 20;
  PINSEL_ConfigPin(&PIN_CFG);

  I2C_Init(lpc_i2c, 100000);
  I2C_Cmd(lpc_i2c, ENABLE);
}

static void busMux(uint8_t chan, LPC_I2C_TypeDef* lpc_i2c, uint8_t muxADDR)
{
  uint8_t i2c_tx[10];
  uint8_t i2c_rx[10];

  I2C_M_SETUP_Type i2c_setup;

  Status i2c_stat;
  //config reg
  i2c_tx[0] = chan;

  i2c_setup.sl_addr7bit = muxADDR;
  i2c_setup.tx_data     = &i2c_tx[0];
  i2c_setup.tx_length   = 1;
  i2c_setup.rx_data     = &i2c_rx[0];
  i2c_setup.rx_length   = 0;

  i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

//  i2c_setup.tx_length = 0;
//  i2c_setup.rx_length = 1;
//
//  i2c_stat = I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
}

static void readADCdata(LPC_I2C_TypeDef* lpc_i2c)
{
  volatile uint8_t i2c_tx[10];
  volatile uint8_t i2c_rx[10];
  volatile uint8_t chan1, chan2, chan3, chan4 = 10;
  volatile float scale = 2.5/1024.0;
  volatile float result1, result2, result3, result4;
  I2C_M_SETUP_Type i2c_setup;

  volatile Status i2c_stat;
  //config reg
  i2c_tx[0] = 2;
  i2c_tx[1] = 0;
  i2c_tx[2] = 0xf8;

  i2c_setup.sl_addr7bit = 0x23;
  i2c_setup.tx_data     = &i2c_tx[0];
  i2c_setup.tx_length   = 3;
  i2c_setup.rx_data     = &i2c_rx[0];
  i2c_setup.rx_length   = 0;

  i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

  i2c_tx[0] = 0x70;

  i2c_setup.sl_addr7bit = 0x23;
  i2c_setup.tx_data     = &i2c_tx[0];
  i2c_setup.tx_length   = 1;
  i2c_setup.rx_data     = &i2c_rx[0];
  i2c_setup.rx_length   = 8;

  i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

  chan1   = (i2c_rx[0] & 0x70) >> 4;
  result1 = scale *  (float)(((i2c_rx[0] & 0x0f)<<6) | (i2c_rx[1]>>2));
  chan2   = (i2c_rx[2] & 0x70) >> 4;
  result2 = scale* (float)( ((i2c_rx[2] & 0x0f)<<6) | (i2c_rx[3]>>2));
  chan3   = (i2c_rx[4] & 0x70) >> 4;
  result3 = scale* (float)( ((i2c_rx[4] & 0x0f)<<6) | (i2c_rx[5]>>2));
  chan4   = (i2c_rx[6] & 0x70) >> 4;
  result4 = scale* (float)( ((i2c_rx[6] & 0x0f)<<6) | (i2c_rx[7]>>2));

  if(lpc_i2c == LPC_I2C1)
    sendSPIdataDiag(0x2,  ( (((uint8_t)(result1*10.0) )<<24) | (((uint8_t)(result2*10.0))<<16) | (((uint8_t)(result3*10.0))<<8) | ((uint8_t)(result4*10.0)) )  );
  else
    sendSPIdataDiag(0x3,  ( (((uint8_t)(result1*10.0) )<<24) | (((uint8_t)(result2*10.0))<<16) | (((uint8_t)(result3*10.0))<<8) | ((uint8_t)(result4*10.0)) )  );


//  i2c_setup.tx_length = 1;
//  i2c_setup.rx_length = 2;
//
//  i2c_stat = I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//
//  //timer interval
//  i2c_tx[0] = 3;
//  i2c_tx[1] = 1;
//  i2c_setup.tx_length = 2;
//  i2c_setup.rx_length = 0;
//
//  i2c_stat = I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//
//  i2c_setup.tx_data     = &i2c_tx[3];
//  i2c_setup.tx_length   = 1;
//  i2c_setup.rx_length   = 2;

//  I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//  chan   = (i2c_rx[0] & 0x70) >> 4;
//  result = scale *  (float)(((i2c_rx[0] & 0x0f)<<6) | (i2c_rx[1]>>2));
//
//  I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//  chan   = (i2c_rx[0] & 0x70) >> 4;
//  result = scale* (float)( ((i2c_rx[0] & 0x0f)<<6) | (i2c_rx[1]>>2));
//
//  I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//  chan   = (i2c_rx[0] & 0x70) >> 4;
//  result = scale* (float)( ((i2c_rx[0] & 0x0f)<<6) | (i2c_rx[1]>>2));
//
//  I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//  chan   = (i2c_rx[0] & 0x70) >> 4;
//  result = scale* (float)( ((i2c_rx[0] & 0x0f)<<6) | (i2c_rx[1]>>2));

}

static void XPTsw()
{
  I2C_M_SETUP_Type I2C_MasterInitStruct;
  uint8_t i2c_tx[10];
  uint8_t i2c_rx[10];
  Status i2c_stat;

  i2c_tx[0] = 0x10; // connect out5 to in8 and out 4 to in13


  I2C_MasterInitStruct.sl_addr7bit = 0x4b;
  I2C_MasterInitStruct.tx_count = 0;
  I2C_MasterInitStruct.tx_data = &i2c_tx[0];
  I2C_MasterInitStruct.tx_length = 1;
  I2C_MasterInitStruct.retransmissions_count = 0;
  I2C_MasterInitStruct.retransmissions_max = 5;
  I2C_MasterInitStruct.rx_data = &i2c_rx[0];
  I2C_MasterInitStruct.rx_length = 1;

  i2c_stat =  I2C_MasterTransferData(LPC_I2C1, &I2C_MasterInitStruct, I2C_TRANSFER_POLLING);

}

static void I2CExtender(LPC_I2C_TypeDef* lpc_i2c)
{
  uint8_t i2c_tx[10];
  uint8_t i2c_rx[10];

  uint16_t io_table[11];

  I2C_M_SETUP_Type i2c_setup;

  Status i2c_stat;
  //config reg

//  i2c_tx[0] = 0x00;
//  i2c_tx[1] = 0x01;
  i2c_tx[0] = 0x12;


//  i2c_setup.sl_addr7bit = 0x20;
//  i2c_setup.tx_data     = &i2c_tx[0];
//  i2c_setup.tx_length   = 1;
//  i2c_setup.rx_data     = &i2c_rx[0];
//  i2c_setup.rx_length   = 1;
//
//  i2c_stat = I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);
//
//  i2c_setup.tx_data     = &i2c_tx[1];
//  i2c_setup.rx_data     = &i2c_rx[1];
//  i2c_stat = I2C_MasterTransferData(LPC_I2C1, &i2c_setup, I2C_TRANSFER_POLLING);

    busMux(I2C_3_MUX, lpc_i2c, I2C_MUX_TST_ADDR);

uint8_t i;
for(i=0; i<4; i++)
{
    if(i==3)
      i2c_setup.sl_addr7bit = 0x24;
    else
      i2c_setup.sl_addr7bit = 0x20+i;

    i2c_setup.tx_data     = &i2c_tx[0];
    i2c_setup.tx_length   = 1;
    i2c_setup.rx_data     = &i2c_rx[0];
    i2c_setup.rx_length   = 2;

    i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

    io_table[i] = (i2c_rx[0] << 8) | i2c_rx[1];
}

busMux(I2C_4_MUX, lpc_i2c, I2C_MUX_TST_ADDR);

for(i=0; i<7; i++)
{
    if(i>2)
      i2c_setup.sl_addr7bit = 0x20+i+1;
    else
      i2c_setup.sl_addr7bit = 0x20+i;

    i2c_setup.tx_data     = &i2c_tx[0];
    i2c_setup.tx_length   = 1;
    i2c_setup.rx_data     = &i2c_rx[0];
    i2c_setup.rx_length   = 2;

    i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

    io_table[i+4] = (i2c_rx[0] << 8) | i2c_rx[1];
}

  uint8_t cmp_err = cmp_ff(&io_table[0], lpc_i2c);
  uint8_t cmp_zero = cmp_zer(&io_table[0], lpc_i2c);

  uint16_t ones_cnt = ones_det(&io_table[0]);

  if(lpc_i2c == LPC_I2C1)
    sendSPIdataDiag(0x0, ( (cmp_zero<<24) | (cmp_err<<16) | (ones_cnt) ) );
  else
    sendSPIdataDiag(0x1, ( (cmp_zero<<24) | (cmp_err<<16) | (ones_cnt) ) );
}

static void sendSPIdataDiag( uint16_t address, uint32_t data )
{
uint8_t  Tx_buff[10];

  if( !checkDONEpin() )
    {
      program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Noncritical_Activity);
      return;
    }
  else
    {
      Tx_buff[0] = WR_COMMAND;
      Tx_buff[1] = 0x00;
      Tx_buff[2] = address;
      Tx_buff[3] = ( ( data >> 24) & 0xff );
      Tx_buff[4] = ( ( data >> 16) & 0xff );
      Tx_buff[5] = ( ( data >> 8)  & 0xff );
      Tx_buff[6] = ( data & 0xff );

      sendSPIbuff(&Tx_buff[0]);
    }
}

static void sendSPIbuff(uint8_t* ptr)
{
      SSP_DATA_SETUP_Type ssp_data;

      ssp_data.tx_data = ptr;
      ssp_data.rx_data = NULL;
      ssp_data.length  = 7;

      ctrlSPI_CS(DISABLE);

      SSP_ReadWrite( LPC_SSP0, &ssp_data, SSP_TRANSFER_POLLING );

      ctrlSPI_CS(ENABLE);
}

static uint16_t ones_det(uint16_t* tab_ptr)
{
  uint8_t i,z;
  uint16_t ones_buff = 0;

   for(i=0; i<11; i++)
     {
       for(z=0; z<16; z++)
         {
           ones_buff+= ((*tab_ptr)>>z) & 0x0001;
         }
       tab_ptr++;
     }
   return ones_buff;
}

static uint8_t cmp_ff(uint16_t* tab_ptr, LPC_I2C_TypeDef* lpc_i2c)
{
  uint16_t or_mask[] = {0x0180, 0x0000, 0x0000, 0x8908, 0x0000, 0x0000, 0x0000, 0x2e82, 0x0000, 0x0000, 0x0081};
  uint8_t i;

  for(i=0; i<11; i++)
    {
      if( ((*tab_ptr) | or_mask[i])  != 0xffff )//io_ff[i])
        return 1;
      else
        tab_ptr++;
    }
return 0;

//  uint16_t io1_ff[]  = {0xfe7f, 0xffff, 0xffff, 0x76f7, 0xffff, 0xffff, 0xffff, 0xd17d, 0xffff, 0xffff, 0xff7e};
//  uint16_t io2_ff[]  = {0xfe7f, 0xffff, 0xffff, 0x76f7, 0xffff, 0xffff, 0xffff, 0xd17d, 0xffff, 0xffff, 0xff7e};
//
//  uint16_t* io_ptr;
//
//  if(lpc_i2c == LPC_I2C1)
//    io_ptr = &io1_ff[0];
//  else
//    io_ptr = &io2_ff[0];
//
//  uint8_t i;
//
//  for(i=0; i<11; i++)
//    {
//      if(*tab_ptr != *(io_ptr) )//io_ff[i])
//        return 1;
//      else
//      {
//          tab_ptr++;
//          io_ptr++;
//      }
//
//    }
//return 0;
}

static uint8_t cmp_zer(uint16_t* tab_ptr, LPC_I2C_TypeDef* lpc_i2c)
{
//  uint16_t io1_ff[]  = {0xfe7f, 0xffff, 0xffff, 0x76f7, 0xffff, 0xffff, 0xffff, 0xd17d, 0xffff, 0xffff, 0xff7e};
//  uint16_t io2_ff[]  = {0xfe7f, 0xffff, 0xffff, 0x76f7, 0xffff, 0xffff, 0xffff, 0xd17d, 0xffff, 0xffff, 0xff7e};
//  uint16_t or_mask[] = {0x0180, 0x0000, 0x0000, 0x8908, 0x0000, 0x0000, 0x0000, 0x2e82, 0x0000, 0x0000, 0x0081};
  uint16_t and_mask[] = {0xfe7f, 0xffff, 0xffff, 0x76f7, 0xffff, 0xffff, 0xfffe, 0xd17d, 0xffff, 0xffff, 0xff76};
  uint8_t i;

  for(i=0; i<11; i++)
    {
      if( ((*tab_ptr) & and_mask[i])  != 0x0000 )//io_ff[i])
        return 1;
      else
        tab_ptr++;
    }
return 0;
}


static void initSPItoFPGA(void)
{

      PINSEL_CFG_Type PinCfg;
      SSP_CFG_Type    SSP_InitStruct;

      PinCfg.Funcnum          = PINSEL_FUNC_3;
      PinCfg.OpenDrain        = PINSEL_PINMODE_NORMAL;
      PinCfg.Pinmode          = PINSEL_PINMODE_PULLUP;
      PinCfg.Portnum          = PINSEL_PORT_1;
      PinCfg.Pinnum           = PINSEL_PIN_20;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Pinnum           = PINSEL_PIN_23;
      PINSEL_ConfigPin(&PinCfg);
      PinCfg.Pinnum           = PINSEL_PIN_24;
      PINSEL_ConfigPin(&PinCfg);

      GPIO_SetDir(1, 0x01<<21, OUTPUT);

      ctrlSPI_CS(ENABLE);

      SSP_ConfigStructInit( &SSP_InitStruct );

      SSP_InitStruct.CPHA = 0;
      SSP_InitStruct.CPOL = 0;
      SSP_InitStruct.Databit = SSP_DATABIT_8;
      SSP_InitStruct.Mode = SSP_MASTER_MODE;
      SSP_InitStruct.FrameFormat = SSP_FRAME_SPI;
      SSP_InitStruct.ClockRate = 10000000;

      SSP_Init(LPC_SSP0, &SSP_InitStruct);
      SSP_Cmd(LPC_SSP0, ENABLE);
}

static void ctrlSPI_CS( FunctionalState newState )
{
      if(newState == ENABLE)
              GPIO_SetValue(1,0x01<<21);
      else
              GPIO_ClearValue(1, 0x01<<21);

}


static void I2CExtenderDir(LPC_I2C_TypeDef* lpc_i2c)
{
  uint8_t i2c_tx[10];
  uint8_t i2c_rx[10];

  uint16_t io_table[11];

  I2C_M_SETUP_Type i2c_setup;

  Status i2c_stat;
  //config reg

  i2c_tx[0] = 0x00;
  busMux(I2C_3_MUX, lpc_i2c, I2C_MUX_TST_ADDR);

  uint8_t i;
  for(i=0; i<4; i++)
  {
      if(i==3)
        i2c_setup.sl_addr7bit = 0x24;
      else
        i2c_setup.sl_addr7bit = 0x20+i;

      i2c_setup.tx_data     = &i2c_tx[0];
      i2c_setup.tx_length   = 1;
      i2c_setup.rx_data     = &i2c_rx[0];
      i2c_setup.rx_length   = 2;

      i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

      io_table[i] = (i2c_rx[0] << 8) | i2c_rx[1];
  }

  busMux(I2C_4_MUX, lpc_i2c, I2C_MUX_TST_ADDR);

  for(i=0; i<7; i++)
  {
      if(i>2)
        i2c_setup.sl_addr7bit = 0x20+i+1;
      else
        i2c_setup.sl_addr7bit = 0x20+i;

      i2c_setup.tx_data     = &i2c_tx[0];
      i2c_setup.tx_length   = 1;
      i2c_setup.rx_data     = &i2c_rx[0];
      i2c_setup.rx_length   = 2;

      i2c_stat = I2C_MasterTransferData(lpc_i2c, &i2c_setup, I2C_TRANSFER_POLLING);

      io_table[i+4] = (i2c_rx[0] << 8) | i2c_rx[1];
  }
}

