/**
  * CardDiagnostic.c
  *
  *  Created on: 27-08-2013
  *      Author: Bartek
  */


#include <string.h>
#include <stdio.h>

#include "../include/cmcpins.h"
#include "../include/gpio.h"

#include "../include/CardDiagnostic.h"
#include "../include/sio_usart.h"

#include "../include/fru.h"

#include "../include/LEDdrivers.h"

#include "../include/utils.h"

#include "../include/diagnosticStructure.h"
#include "../IC/ic_commonHeader.h"

#include "../stdPeriphLibs/lpc17xx_timer.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../stdPeriphLibs/lpc17xx_ssp.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"

#include "../include/sensor_sdr.h"
#include "../include/iap_driver.h"

#include "../include/ipmi_i2c_driver.h"
#include "../include/i2c_flags.h"

#include "../include/interruptPriorities.h"


//#define WR_COMMAND 0x80
//#define RD_COMMAND 0x00
#define DIAGNOSTIC_I2C                LPC_I2C1
#define DIAGNOSTIC_I2C_CLK    400000

/*
 * used for spi comm test
 */
//volatile uint32_t testPattern [8] = { 0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF12, 0x12345678, 0x98765432, 0xFFEEDDCC, 0x10293847};

/*
 * global variables
 */
volatile uint8_t sensorPtr = 0;
volatile uint8_t address = 0x00;
volatile uint8_t cnt = 0x00;
volatile uint32_t rptCNT = 0x00;

volatile SystemDiagnosticStruct_T diagnosticStruct;
volatile Diagnostic_I2C_T     diagnosticI2C;

const uint8_t LM75_ADDR_table[] = { LM75_1_ADDR, LM75_2_ADDR, LM75_3_ADDR, LM75_4_ADDR};
const uint8_t INA220_ADDR_table[6] = { INA220_ADDR1, INA220_ADDR2, INA220_ADDR3,
                                     INA220_ADDR4, INA220_ADDR5, INA220_ADDR6 };

volatile uint8_t i2cstatus [20];
volatile uint8_t i2cptr = 0;

/*
 * static functions declaration
 */
static void initDiagnosticStruct(void);
static void Timer3Init(void);
static void initSPItoFPGA(void);
static void ctrlSPI_CS( FunctionalState newState );


static uint8_t cmpBuffs(uint32_t* bufa, uint32_t* bufb);

static void diagI2C_Init ( void );

static void I2CCONSET( uint32_t flags );
static void I2CCONCLR( uint32_t flags );
static void I2CDAT_WRITE( uint8_t val );
static uint8_t I2CDAT_READ( void );

static void copyDataToI2CDiag( void );

static uint32_t receiveSPIbuff(uint8_t* ptr);

static void sendSPIbuff(uint8_t* ptr);
/**
 * @param none
 * @return none
 *
 * Initialize all services and peripherals required for
 * card diagnostic management:\n
 * 1. LPC_I2C1  - to communicate I2C chips on pcb\n
 * 2. LPC_SSP0  - uC to FPGA bidir communication\n
 * 3. TIMER3    - to generate cyclic interrupt (calls sensors reading actualization)\n
 * 4. Initialize diagnostic structure with default values\n
 */
void initDiagnosticMonitor(  )
{
      diagI2C_Init();

      initSPItoFPGA( );

      Timer3Init( );

      initDiagnosticStruct(  );
}


/**
* Returns sensor readout value for
* it is called by SDR service
 */
uint8_t getSensorData(uint8_t  sensorNum)
{
      uint8_t retVal;

      if  (sensorNum < 0)
              return (0);                     // out of range
      if (sensorNum < 0x0F)
        retVal = (uint8_t)diagnosticStruct.sensor[sensorNum].Val1;
      else
        retVal = 0;
      return (retVal);
}

/**
 * Enables and disables diagnostic callback when power is on/off
 */
void diagnosticMonitorCallback( FunctionalState newState )
{
      TIM_Cmd(LPC_TIM3, newState);
      I2C_IntCmd(DIAGNOSTIC_I2C, newState);

      NVIC_SetPriorityGrouping(I2C1_PriorGrup);
      NVIC_SetPriority(I2C1_IRQn, I2C1_Prior);
}

////////////////////////IRQ_Handlers/////////////////////////////////////
/**
 * Timer3 - scanning sensors and when all sensors data is updated
 * it sends updated data to FPGA over SPI
 */
void TIMER3_IRQHandler (void)
{

      if ( diagnosticStruct.sensorPtr == BOARD_SENSORS) //(sensorPtr == BOARD_SENSORS-1)
      {
              sensor_data_convert_complete();
              sendSPIdata( 0x0000, 0x00000000 );
              diagnosticStruct.sensorPtr = 0;
      }
      else
      {

              I2CCONCLR(I2C_CTRL_FL_AA | I2C_CTRL_FL_SI | I2C_CTRL_FL_STA | I2C_CTRL_FL_STO);

              copyDataToI2CDiag( );

              diagnosticStruct.sensorPtr++;

              for (i2cptr=0; i2cptr < 20 ; i2cptr ++)
              {
                      i2cstatus[i2cptr] = 0x00;
              }

              i2cptr = 0;



              I2CCONSET( I2C_CTRL_FL_STA );

      }

      /*  Clear Interrupt */
      TIM_ClearIntPending(LPC_TIM3, TIM_MR3_INT);
}

void I2C1_IRQHandler( void )
{
      NVIC_ClearPendingIRQ(I2C0_IRQn);

      uint8_t curbyte;

      uint32_t status;


      uint8_t bytesToSend     = diagnosticI2C.bytesToSend;
      uint8_t rawDataPtr      = diagnosticI2C.rawDataPtr;
      uint8_t regAddrPtr      = diagnosticI2C.regAddrPtr;
      uint8_t bytesToRead     = diagnosticI2C.bytesToReadFromReg[regAddrPtr];
      uint8_t sensorSlaveAddr = diagnosticI2C.sensorSlaveAddr;
      uint8_t sensorPtr       = diagnosticI2C.sensorPtr;


      status = DIAGNOSTIC_I2C->I2STAT;
      i2cstatus[i2cptr++] = status;

 switch (status) {

 /* Master transmitter/receiver mode common */
   case I2STAT_START_SENT:
     /* A Start condition has been transmitted as a result of
      * i2c_master_read/write. The slave address + R/W bit will
      * be transmitted, an ACK bit received is expected next.
      *====================================================
      ** load I2DAT with the slave address and the data
      * direction bit (SLA+W or SLA+R). The SI bit in I2CON must
      * then be reset before the serial transfer can continue.
      */

     curbyte = ( sensorSlaveAddr << 1 );                              // get address byte

     I2CDAT_WRITE( curbyte );                         // send destination address (write operation)
     I2CCONCLR( I2C_CTRL_FL_SI | I2C_CTRL_FL_STA );

     break;

     /* Master transmitter mode */
   case I2STAT_SLAW_SENT_ACKED:
     /* Previous state was State I2STAT_START_SENT or
      * State I2STAT_REP_START_SENT. Slave Address +
      * Write has been transmitted, ACK has been received.
      * The first data byte will be transmitted, an ACK
      * bit will be received.
      */
     /* write first byte of data */
     curbyte = diagnosticStruct.sensor[sensorPtr].regAddr[regAddrPtr];
     bytesToSend--;

     I2CDAT_WRITE( curbyte );                                         // write first data byte--this should start TWI transfer

     I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_STO | I2C_CTRL_FL_SI );

     break;

   case I2STAT_MASTER_DATA_SENT_ACKED:
     /* Data has been transmitted, ACK has been received.
      * If the transmitted data was the last data byte
      * then transmit a Stop condition, otherwise transmit
      * the next data byte. */

     if( bytesToRead > 0 )
       {
         I2CCONCLR( I2C_CTRL_FL_SI );
         I2CCONSET( I2C_CTRL_FL_STA );
       }
     else {
         // nothing more to send, arm the transmit complete interrupt
         I2CCONSET( I2C_CTRL_FL_STO );
         I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_SI );

     }

     break;

   case I2STAT_REP_START_SENT:
     curbyte = ( ( sensorSlaveAddr << 1 ) | 1  );

     I2CDAT_WRITE( curbyte );                         // send destination address (write operation)
     I2CCONCLR( I2C_CTRL_FL_SI | I2C_CTRL_FL_STA );

     break;

   case I2STAT_SLAR_SENT_ACKED:
     if(  bytesToRead > 1 )
       I2CCONSET( I2C_CTRL_FL_AA );
     else
       I2CCONCLR( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_STO | I2C_CTRL_FL_SI );

     break;

   case I2STAT_MASTER_DATA_RCVD_ACKED:

     diagnosticStruct.sensor[sensorPtr].rawData[rawDataPtr++] = I2CDAT_READ();

     bytesToRead--;

     if(bytesToRead > 1)
       {
         I2CCONSET( I2C_CTRL_FL_AA );
         I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_STO | I2C_CTRL_FL_SI );
       }
     else
       {
         I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_SI | I2C_CTRL_FL_AA | I2C_CTRL_FL_STO );
       }

     break;

   case I2STAT_MASTER_DATA_RCVD_NOT_ACKED:

     diagnosticStruct.sensor[sensorPtr].rawData[rawDataPtr++] = I2CDAT_READ();

     bytesToRead--;

     if( bytesToSend > 0 )
       {
         //there are more registers to read
         //repeated start
         I2CCONCLR( I2C_CTRL_FL_SI );
         I2CCONSET( I2C_CTRL_FL_STO | I2C_CTRL_FL_STA );
         regAddrPtr++;
       }
     else
       { //all data has been read
         //stop condition

         I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_SI );
         I2CCONSET( I2C_CTRL_FL_STO );

         diagnosticStruct.sensor[sensorPtr].Val1 =
             (*(diagnosticStruct.sensor[sensorPtr].ReadOutFunc))(&diagnosticStruct.sensor[sensorPtr].rawData[0]);

       }

     break;



   case I2STAT_STOP_START_RCVD:

     /* Set AA flag & clear SI */
     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

   case I2STAT_SLAW_SENT_NOT_ACKED:
     I2CCONCLR( I2C_CTRL_FL_SI );
     break;


   case I2STAT_ARBITRATION_LOST:
     /* Arbitration has been lost during either Slave
      * Address + Write or data. The bus has been released.
      *
      * Due to the nature of the i2c design, we may not know
      * that we may lose the arbitration until the first
      * different bit of either the SLA+RW or data.
      */

     // master loss of arbitration interrupt
     I2CCONSET( I2C_CTRL_FL_STO );
     I2CCONCLR( I2C_CTRL_FL_SI );
     break;


   case I2STAT_MASTER_DATA_SENT_NOT_ACKED:
     /* Data has been transmitted, NOT ACK received.
      * Send a STOP condition
      */

     I2CCONSET( I2C_CTRL_FL_STO );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

   default:
     I2CCONCLR( I2C_CTRL_FL_SI );
     break;
 }

 diagnosticI2C.bytesToSend                                      = bytesToSend;
 diagnosticI2C.bytesToReadFromReg[diagnosticI2C.regAddrPtr]       = bytesToRead;
 diagnosticI2C.rawDataPtr                                       = rawDataPtr;
 diagnosticI2C.regAddrPtr                                       = regAddrPtr;


}

////////////////////////IRQ_Handlers_end/////////////////////////////////////



uint8_t calculateMedTemp(uint8_t param)
{
      uint8_t MedTemp = 0;
      uint16_t sum = 0;
      uint8_t i = 0;

      for (; i < TEMP_SENSORS_CNT; i++)
      {
              sum += diagnosticStruct.sensor[i].Val1;
      }

      MedTemp = sum/TEMP_SENSORS_CNT;

      return (MedTemp);
}



void sendSPIdata( uint16_t address, uint32_t data )
{

      uint8_t addressFPGAram = 0x00;

      uint8_t i = 0;
      uint8_t y = 1;

//      uint32_t tx_trace[30];
//      uint32_t rx_trace[30];

      uint32_t tx_trace[FPGA_MEM_ADDR_MAX+1];
      uint32_t rx_trace[FPGA_MEM_ADDR_MAX+1];

      uint8_t  Tx_buff[10];


      if( !checkDONEpin() )
      {
          program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Noncritical_Activity);
          return;
      }
      else
      {

              ////////////////////////////////////////////////////
              //////////////////CARD_ID//////////////////////////
              for( i = 0; i < 4; i++)
              {
                      data = diagnosticStruct.cardID[i];

                      tx_trace[addressFPGAram] = data;

                      Tx_buff[0] = WR_COMMAND;
                      Tx_buff[1] = 0x00;
                      Tx_buff[2] = addressFPGAram++;
                      Tx_buff[3] = ( ( data >> 24) & 0xff );
                      Tx_buff[4] = ( ( data >> 16) & 0xff );
                      Tx_buff[5] = ( ( data >> 8)  & 0xff );
                      Tx_buff[6] = ( data & 0xff );

                      sendSPIbuff(&Tx_buff[0]);
              }

              ///////////////////////////////////////////////////
              ///////////////////AMC SLOT///////////////////////
              data = diagnosticStruct.AMC_NR;

              tx_trace[addressFPGAram] = data;

              Tx_buff[0] = WR_COMMAND;
              Tx_buff[1] = 0x00;
              Tx_buff[2] = addressFPGAram++;
              Tx_buff[3] = ( ( data >> 24) & 0xff );
              Tx_buff[4] = ( ( data >> 16) & 0xff );
              Tx_buff[5] = ( ( data >> 8)  & 0xff );
              Tx_buff[6] = ( data & 0xff );

              sendSPIbuff(&Tx_buff[0]);

              ///////////////////////////////////////////////////
              /////////////////DATA VALID///////////////////////

              data = 0x55555555;

              tx_trace[addressFPGAram] = data;

              Tx_buff[0] = WR_COMMAND;
              Tx_buff[1] = 0x00;
              Tx_buff[2] = addressFPGAram++;
              Tx_buff[3] = ( ( data >> 24) & 0xff );
              Tx_buff[4] = ( ( data >> 16) & 0xff );
              Tx_buff[5] = ( ( data >> 8)  & 0xff );
              Tx_buff[6] = ( data & 0xff );

              sendSPIbuff(&Tx_buff[0]);

              ////////////////////////////////////////////////////
              ////////////////////Sensors Data////////////////////


              for( i = 0; i<BOARD_SENSORS; i++)
              {

                  data  =  ( 0x00ffffff & ( (uint32_t)(diagnosticStruct.sensor[i].Val1*1000) ));
                  data  =  ( ( (i+1) << 24 ) | data );



                  tx_trace[addressFPGAram] = data;

                  y++;

                  Tx_buff[0] = WR_COMMAND;
                  Tx_buff[1] = 0x00;
                  Tx_buff[2] = addressFPGAram++;
                  Tx_buff[3] = ( ( data >> 24) & 0xff );
                  Tx_buff[4] = ( ( data >> 16) & 0xff );
                  Tx_buff[5] = ( ( data >> 8)  & 0xff );
                  Tx_buff[6] = ( data & 0xff );

                  sendSPIbuff(&Tx_buff[0]);

              }

              for(addressFPGAram = 0;  addressFPGAram < FPGA_MEM_ADDR_MAX; addressFPGAram++)
                {
                  Tx_buff[0] = RD_COMMAND;
                  Tx_buff[1] = 0x00;
                  Tx_buff[2] = addressFPGAram;
                  rx_trace[addressFPGAram] = receiveSPIbuff(&Tx_buff[0]);
                }
              if( !cmpBuffs( &tx_trace[0], &rx_trace[0] ) )
              {
                  data = 0xAAAAAAAA;
                  program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Off_Activity);
              }
              else
                {
                  program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_50ms_Flash_Activity);
                  data = 0x55555555;
                }

                Tx_buff[0] = WR_COMMAND;
                Tx_buff[1] = 0x00;
                Tx_buff[2] = 0x05;
                Tx_buff[3] = ( ( data >> 24) & 0xff );
                Tx_buff[4] = ( ( data >> 16) & 0xff );
                Tx_buff[5] = ( ( data >> 8)  & 0xff );
                Tx_buff[6] = ( data & 0xff );

                sendSPIbuff(&Tx_buff[0]);


                uint32_t tmp = 0;
                tmp  = gpio_get_pin_value(14, 1);
                tmp |= (gpio_get_pin_value(15, 1)<<1);
                tmp |= (gpio_get_pin_value(16, 1)<<2);
                tmp |= (gpio_get_pin_value(17, 1)<<3);
                tmp |= (gpio_get_pin_value(18, 1)<<4);
                tmp |= (gpio_get_pin_value(19, 1)<<5);

                Tx_buff[0] = WR_COMMAND;
                Tx_buff[1] = 0x00;
                Tx_buff[2] = 0xff;
                Tx_buff[3] = ( ( tmp >> 24) & 0xff );
                Tx_buff[4] = ( ( tmp >> 16) & 0xff );
                Tx_buff[5] = ( ( tmp >> 8)  & 0xff );
                Tx_buff[6] = ( tmp & 0xff );

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

static uint32_t receiveSPIbuff(uint8_t* ptr)
{
      SSP_DATA_SETUP_Type ssp_data;

      uint8_t Rx_buff[10];

      ssp_data.tx_data = ptr;
      ssp_data.rx_data = &Rx_buff[0];
      ssp_data.length  = 7;

      ctrlSPI_CS(DISABLE);

      SSP_ReadWrite( LPC_SSP0, &ssp_data, SSP_TRANSFER_POLLING );
      ctrlSPI_CS(ENABLE);

      return  ( (uint32_t) ( (Rx_buff[3] << 24) | (Rx_buff[4] << 16) | (Rx_buff[5] << 8) | Rx_buff[6] ) );

}


/**
 * return maximum measured temperature
 */
uint8_t calculateMaxTemp(uint8_t param)
{
      uint8_t maxTemp = 0x00;
      uint8_t i = 0;

      for (; i < TEMP_SENSORS_CNT; i++)
      {
              maxTemp = MAX( maxTemp, diagnosticStruct.sensor[i].Val1);
      }

      return (maxTemp);
}


static uint8_t cmpBuffs(uint32_t* bufa, uint32_t* bufb)
{
      uint16_t i = 0;

              for( ; i<FPGA_MEM_ADDR_MAX; i++)
              {
                      if(*bufa != *bufb)
                      {
                              return (0xFF);
                      }
                      bufa++;
                      bufb++;
              }

      return (0);
}

static void initDiagnosticStruct(void)
{
      uint8_t i = 0;
      uint8_t z = 0;
      iap_get_id();

      diagnosticStruct.cardID[0] = uC_id[0];
      diagnosticStruct.cardID[1] = uC_id[1];
      diagnosticStruct.cardID[2] = uC_id[2];
      diagnosticStruct.cardID[3] = uC_id[3];

      diagnosticStruct.AMC_NR = (uint32_t) ( ( 0xff0000 & ( ipmi_i2c_state.ipmbl_addr  << 16) ) | ( 0x00FF & ipmi_i2c_state.slotid ) );

      diagnosticStruct.sensorPtr = 0;

      for(; i < BOARD_SENSORS; i++ )
      {

#ifdef AMC_FMC_Carierr
              if ( i == 0 )
              {
                      diagnosticStruct.sensor[i].Val1         = 0;
                      diagnosticStruct.sensor[i].ReadOutFunc  = readFPGATemp;

                      diagnosticStruct.sensor[i].i2c_addr                     = MAX6642_ADDRESS;
                      diagnosticStruct.sensor[i].bytesToSend                  = 2;
                      diagnosticStruct.sensor[i].bytesToReadFromReg[0]        = 1;
                      diagnosticStruct.sensor[i].bytesToReadFromReg[1]        = 1;

                      for(z = 2 ; z < MAX_FIELD_TO_SEND; z++)
                        diagnosticStruct.sensor[i].bytesToReadFromReg[z] = 0;


                      diagnosticStruct.sensor[i].regAddr[0] = MAX6642_REMOTE_TEMP_R;
                      diagnosticStruct.sensor[i].regAddr[1] = MAX6642_REMOTE_EXT_R;

              }

              else if ( (i > 0) & (i < TEMP_SENSORS_CNT) )
              {
                      diagnosticStruct.sensor[i].Val1         = 0;
                      diagnosticStruct.sensor[i].ReadOutFunc  = get_LM_temp;

                      diagnosticStruct.sensor[i].i2c_addr        = LM75_ADDR_table[i-1];
                      diagnosticStruct.sensor[i].bytesToSend     = 1;
                      diagnosticStruct.sensor[i].bytesToReadFromReg[0] = 2;
                      diagnosticStruct.sensor[i].regAddr[0]            = TEMP_REG;

                      for(z = 1 ; z < MAX_FIELD_TO_SEND; z++)
                        diagnosticStruct.sensor[i].bytesToReadFromReg[z] = 0;




              }
              else if ( ( i >= TEMP_SENSORS_CNT) & (i < (CURR_SENSORS_CNT+TEMP_SENSORS_CNT) ) )
              {
                      diagnosticStruct.sensor[i].Val1                 = 0;
                      diagnosticStruct.sensor[i].ReadOutFunc          = get_INNA_curr;

                      diagnosticStruct.sensor[i].i2c_addr             = INA220_ADDR_table[i - TEMP_SENSORS_CNT];
                      diagnosticStruct.sensor[i].bytesToSend           = 1;
                      diagnosticStruct.sensor[i].bytesToReadFromReg[0] = 2;

                      diagnosticStruct.sensor[i].regAddr[0]            = INA220_SHT_REG;

                      for(z = 1 ; z < MAX_FIELD_TO_SEND; z++)
                        diagnosticStruct.sensor[i].bytesToReadFromReg[z] = 0;
              }
              else
              {
                      diagnosticStruct.sensor[i].Val1                 = 0;
                      diagnosticStruct.sensor[i].ReadOutFunc          = get_INNA_volt;

                      diagnosticStruct.sensor[i].i2c_addr              = INA220_ADDR_table[i - (CURR_SENSORS_CNT+TEMP_SENSORS_CNT)];
                      diagnosticStruct.sensor[i].bytesToSend           = 1;
                      diagnosticStruct.sensor[i].bytesToReadFromReg[0] = 2;

                      diagnosticStruct.sensor[i].regAddr[0]            = INA220_BUS_REG;

                      for(z = 1 ; z < MAX_FIELD_TO_SEND; z++)
                        diagnosticStruct.sensor[i].bytesToReadFromReg[z] = 0;
              }
#endif

//#ifdef AMC_CPU_COM_Express
//              diagnosticStruct.sensor[i].Val1         = 0;
//              diagnosticStruct.sensor[i].Val2         = 0;
//              diagnosticStruct.sensor[i].sType        = 1;
//              diagnosticStruct.sensor[i].ReadOutFunc = get_LM_temp;
//              diagnosticStruct.sensor[i].FuncArg         = i;
//#endif
//
//#ifdef AMC_CPU_COM_Express
//              if ( i == 0 )
//              {
//                      diagnosticStruct.sensor[i].Val1         = 0;
//                      diagnosticStruct.sensor[i].Val2         = 0;
//                      diagnosticStruct.sensor[i].sType        = 1;
//                      diagnosticStruct.sensor[i].ReadOutFunc = readFPGATemp;
//                      diagnosticStruct.sensor[i].FuncArg         = 0;
//              }
//
//              else if ( (i > 0) & (i < TEMP_SENSORS_CNT) )
//              {
//                      diagnosticStruct.sensor[i].Val1         = 0;
//                      diagnosticStruct.sensor[i].Val2         = 0;
//                      diagnosticStruct.sensor[i].sType        = 1;
//                      diagnosticStruct.sensor[i].ReadOutFunc = get_LM_temp;
//                      diagnosticStruct.sensor[i].FuncArg         = i;
//              }
//#endif

      }
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


/**
 * Timer3 configuration
 */
static void Timer3Init(void)
{
      TIM_TIMERCFG_Type TMR3_Cfg;
      TIM_MATCHCFG_Type TMR3_Match;

      /* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
      /* Initialize timer 0, prescale count time of 100uS */
      TMR3_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
      TMR3_Cfg.PrescaleValue = 100;//1000;
      /* Use channel 0, MR0 */
      TMR3_Match.MatchChannel = 0;
      /* Enable interrupt when MR0 matches the value in TC register */
      TMR3_Match.IntOnMatch = ENABLE;
      /* Enable reset on MR0: TIMER will reset if MR0 matches it */
      TMR3_Match.ResetOnMatch = TRUE;
      /* Don't stop on MR0 if MR0 matches it*/
      TMR3_Match.StopOnMatch = FALSE;
      /* Do nothing for external output pin if match (see cmsis help, there are another options) */
      TMR3_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
      /* Set Match value, count value of 2 (2 * 100uS = 200us ) */
      TMR3_Match.MatchValue = 5000;//2000;
      /* Set configuration for Tim_config and Tim_MatchConfig */
      TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &TMR3_Cfg);
      TIM_ConfigMatch(LPC_TIM3, &TMR3_Match);

      NVIC_SetPriorityGrouping(TIMER3_PriorGrup);
      /* Preemption = 1, sub-priority = 1 */
      NVIC_SetPriority(TIMER3_IRQn, TIMER3_Prior);
      /* Enable interrupt for timer 0 */
      NVIC_EnableIRQ(TIMER3_IRQn);
      /* Start timer 0 */

}

static void diagI2C_Init ( void )
{
      PINSEL_CFG_Type PIN_CFG;
      I2C_OWNSLAVEADDR_CFG_Type slave_cfg;

      PIN_CFG.Pinnum = 0;
      PIN_CFG.Portnum = 0;
      PIN_CFG.Pinmode = PINSEL_PINMODE_TRISTATE;
      PIN_CFG.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
      PIN_CFG.Funcnum = PINSEL_FUNC_3;

      PINSEL_ConfigPin(&PIN_CFG);

      PIN_CFG.Pinnum = 1;

      PINSEL_ConfigPin(&PIN_CFG);

      I2C_Init(DIAGNOSTIC_I2C, DIAGNOSTIC_I2C_CLK);
      I2C_Cmd(DIAGNOSTIC_I2C, ENABLE);

      slave_cfg.GeneralCallState = DISABLE;
      slave_cfg.SlaveAddrMaskValue = 0x00;
      slave_cfg.SlaveAddrChannel = 0x00;
      slave_cfg.SlaveAddrChannel = 1;

      I2C_SetOwnSlaveAddr(DIAGNOSTIC_I2C, &slave_cfg);

      I2CCONCLR(I2C_CTRL_FL_AA | I2C_CTRL_FL_SI | I2C_CTRL_FL_STA | I2C_CTRL_FL_STO);


}


////////////////I2C MACROS
/* Conrol bit manipulation macros */
static void I2CCONSET(uint32_t flags)
{
              DIAGNOSTIC_I2C->I2CONSET = flags;
}

static void I2CCONCLR( uint32_t flags)
{
              DIAGNOSTIC_I2C->I2CONCLR = flags;
}


static void I2CDAT_WRITE( uint8_t val )
{
      DIAGNOSTIC_I2C->I2DAT = val;
}

static uint8_t I2CDAT_READ( )
{
      uint8_t val = 0x00;
      val = (uint8_t)( 0x0FF & (DIAGNOSTIC_I2C->I2DAT) );
      return (val);
}


static void copyDataToI2CDiag( void )
{
      uint8_t i = 0;
      for( ; i<MAX_FIELD_TO_SEND; i++)
      {
              diagnosticI2C.bytesToReadFromReg[i] = diagnosticStruct.sensor[diagnosticStruct.sensorPtr].bytesToReadFromReg[i];
      }

      diagnosticI2C.bytesToSend = diagnosticStruct.sensor[diagnosticStruct.sensorPtr].bytesToSend;
      diagnosticI2C.rawDataPtr = 0;
      diagnosticI2C.regAddrPtr = 0;
      diagnosticI2C.sensorPtr = diagnosticStruct.sensorPtr;
      diagnosticI2C.sensorSlaveAddr = diagnosticStruct.sensor[diagnosticStruct.sensorPtr].i2c_addr;
}













