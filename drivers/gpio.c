
#include "../include/cmcpins.h"
#include "../include/gpio.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"
#include "../stdPeriphLibs/lpc17xx_spi.h"


static void delay( void );


void gpio_init() {

  SPI_CFG_Type spiInitialization;

  PINSEL_CFG_Type PinSelCfg;

/*-----------------------------------------------------------------
 *--------------------------PORT 0---------------------------------
 *-----------------------------------------------------------------
 */
  GPIO_SetDir(0, 0xFFFFFFFF, OUTPUT);
  PinSelCfg.Portnum = PINSEL_PORT_0;

//  PinSelCfg.Pinnum = PINSEL_PIN_0;
//  PinSelCfg.Funcnum = PINSEL_FUNC_3;
//  PinSelCfg.OpenDrain = 0;
//  PinSelCfg.Pinmode = 0;
//  PINSEL_ConfigPin(&PinSelCfg);
//
//  PinSelCfg.Pinnum = PINSEL_PIN_1;
//  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Pinnum = PINSEL_PIN_2;
  PinSelCfg.Funcnum = PINSEL_FUNC_1;
  PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinSelCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Pinnum = PINSEL_PIN_3;
  PINSEL_ConfigPin(&PinSelCfg);


  GPIO_SetDir(0, 0x01<<22, INPUT);


  PinSelCfg.Pinnum = PINSEL_PIN_10;
  PinSelCfg.Funcnum = PINSEL_FUNC_2;
  PinSelCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
  PinSelCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Pinnum = PINSEL_PIN_11;
  PINSEL_ConfigPin(&PinSelCfg);


  //SET FMC_I2C1 in Hi-Z
  GPIO_SetDir(0, 0x01<<19, INPUT);
  GPIO_SetDir(0, 0x01<<20, INPUT);
//  PinSelCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
//  PinSelCfg.Funcnum = PINSEL_FUNC_3;
//  PinSelCfg.Pinnum = PINSEL_PIN_19;
//  PINSEL_ConfigPin(&PinSelCfg);
//
//  PinSelCfg.Pinnum = PINSEL_PIN_20;
//  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Funcnum = PINSEL_FUNC_1;
  PinSelCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
  PinSelCfg.Pinnum = PINSEL_PIN_27;
  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Pinnum = PINSEL_PIN_28;
  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Pinnum = PINSEL_PIN_29;
  PinSelCfg.Funcnum = PINSEL_FUNC_1;
  PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinSelCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PINSEL_ConfigPin(&PinSelCfg);

  PinSelCfg.Pinnum = PINSEL_PIN_30;
  PINSEL_ConfigPin(&PinSelCfg);

/*-----------------------------------------------------------------
 *---------------------END OF PORT 0-------------------------------
 *-----------------------------------------------------------------
 *-----------------------------------------------------------------
 * ------------------------PORT 1----------------------------------
 * ----------------------------------------------------------------
 */
  GPIO_SetDir(1, 0xFFFFFFFF, OUTPUT);

  GPIO_SetDir(1, ( 0x01<<IPMI_GA0 | 0x01<<IPMI_GA1 | 0x01<<IPMI_GA2 ), INPUT);
  GPIO_SetValue(LED_PORT, (0x01<<_IPMI_BLLED|0x01<<_IPMI_LED1|0x01<<_IPMI_LED2) );

//  PinSelCfg.Portnum = PINSEL_PORT_1;
//  PinSelCfg.Pinnum = PINSEL_PIN_30;
//  PinSelCfg.Funcnum = PINSEL_FUNC_2;
//  PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
//  PinSelCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
//  PINSEL_ConfigPin(&PinSelCfg);


  GPIO_SetDir(ENA_PORT, 0x001<<ENA_PIN, INPUT);

  GPIO_SetDir(2, 0x0400, INPUT);

  GPIO_SetDir(IPMI_EJTHDL_PORT, 0x001<<IPMI_EJTHDL, INPUT);

  GPIO_SetDir(1, 0x01<<22, INPUT);

  GPIO_SetDir(PGOOD_PORT, 0x01<<PGOOD_PIN, INPUT);



  gpio_clr_gpio_pin(_IPMI_BLLED, LED_PORT);						// turn on blue LED ASAP
  gpio_clr_gpio_pin(UC_PWRENOUT, UC_PWRENOUT_PORT);               // turn off power enable



  PinSelCfg.Pinnum  = PINSEL_PIN_15;
  PinSelCfg.Portnum = PINSEL_PORT_0;
  PinSelCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  PinSelCfg.Funcnum = PINSEL_FUNC_3;
  PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;

  PINSEL_ConfigPin(&PinSelCfg);


  PinSelCfg.Pinnum  = PINSEL_PIN_18;

  PINSEL_ConfigPin(&PinSelCfg);

  GPIO_SetDir(1, 0x01<<26, OUTPUT);

  GPIO_SetValue(1, 0x01<<26);

  GPIO_SetDir(0, 0x01<<21, OUTPUT);
  GPIO_SetDir(0, 0x01<<16, OUTPUT);
  GPIO_SetValue(0, 0x01<<16);
  GPIO_SetValue(0, 0x01<<21);


  GPIO_SetDir(PORT_EN_0, 0x01<<EN_P1V2, OUTPUT);
  GPIO_SetDir(PORT_EN_0, 0x01<<EN_P1V8, OUTPUT);
  GPIO_SetDir(PORT_EN_0, 0x01<<EN_FMC2_P3V3, OUTPUT);
  GPIO_SetDir(PORT_EN_0, 0x01<<EN_FMC1_P3V3, OUTPUT);
  GPIO_SetDir(PORT_EN_0, 0x01<<EM_FMC1_P12V, OUTPUT);
  GPIO_SetDir(PORT_EN_0, 0x01<<EN_FMC2_P12V, OUTPUT);


  GPIO_SetDir(PORT_EN_1, 0x01<<EN_FMC1_PVADJ, OUTPUT);
  GPIO_SetDir(PORT_EN_1, 0x01<<EN_FMC2_PVADJ, OUTPUT);
  GPIO_SetDir(PORT_EN_1, 0x01<<EN_P3V3, OUTPUT);
  GPIO_SetDir(PORT_EN_1, 0x01<<EN_1V5_VTT, OUTPUT);

  GPIO_SetDir(PORT_EN_1, 0x01<<EN_RTM_CONN, OUTPUT);
  GPIO_SetDir(PORT_EN_0, 0x01<<RTM_PRESENCE, INPUT);

  GPIO_SetDir(PORT_EN_3, 0x01<<EN_P1V0, OUTPUT);

  setDC_DC_ConvertersON();

  spiInitialization.CPOL = SPI_CPOL_LO;
  spiInitialization.CPHA = SPI_CPHA_FIRST;
  spiInitialization.Databit = SPI_DATABIT_10;
  spiInitialization.DataOrder = SPI_DATA_MSB_FIRST;
  spiInitialization.ClockRate = 10000000;

  SPI_Init(LPC_SPI, &spiInitialization);

  LPC_SPI->SPCR = 0xA24;



  GPIO_ClearValue(0, 0x01<<16);
  SPI_SendData(LPC_SPI, 0x0125);
  while(! ( SPI_CheckStatus( SPI_GetStatus(LPC_SPI), SPI_STAT_SPIF) ));
  GPIO_SetValue(0, 0x01<<16);

  GPIO_ClearValue(0, 0x01<<16);
  SPI_SendData(LPC_SPI, 0x0025);
  while(! ( SPI_CheckStatus( SPI_GetStatus(LPC_SPI), SPI_STAT_SPIF) ));
  GPIO_SetValue(0, 0x01<<16);


  //SCANSTA JTAG
  GPIO_SetDir(2, 0x0FF, OUTPUT);

  GPIO_SetValue(2, 0x080);

  GPIO_ClearValue(2, 0x07F);

  SPI_ConfigStructInit(&spiInitialization);


  //FPGA_SPI
  GPIO_SetDir(1, 0x01<<20, INPUT);
  GPIO_SetDir(1, 0x01<<21, INPUT);
  GPIO_SetDir(1, 0x01<<23, INPUT);
  GPIO_SetDir(1, 0x01<<24, INPUT);
  // SET PROGRAM_B AS OUTPUT
  // AND SET THIS PIN HIGH
  GPIO_SetDir(0, 0x01<<17, INPUT);
  // SET DONE PIN AS INPUT
  // turn on pull-up
  GPIO_SetDir(0, 0x01<<22, INPUT);
  ////////////////////////////////////////
  // P0_6 - FCS_B - use as FPGA - RST
  ////////////////////////////////////////
  GPIO_SetDir(0, 0x01<<6, INPUT);
  //GPIO_ClearValue(0, 0x01<<6);
  ///////////////////////////////////////
  GPIO_SetDir(0, 0x01<<7, INPUT);
  GPIO_SetDir(0, 0x01<<8, INPUT);
#ifdef AMC_CPU_COM_Express
  GPIO_SetDir(0, 0x01<<9, OUTPUT);
#else
  GPIO_SetDir(0, 0x01<<9, INPUT);
#endif

}


void gpio_set_gpio_pin(unsigned int pin, uint8_t port)
{
	GPIO_SetValue(port, 0x01<<pin);
}


void gpio_clr_gpio_pin(unsigned int pin, uint8_t port)
{
	GPIO_ClearValue(port, 0x01<<pin);
}


void gpio_tgl_gpio_pin(unsigned int pin, uint8_t port)
{
	uint8_t bitVal = ( (GPIO_ReadValue(port)>>pin) & 0x01 );
	if(bitVal)
		GPIO_ClearValue(port, 0x01<<pin);
	else
		GPIO_SetValue(port, 0x01<<pin);
}


uint8_t gpio_get_pin_value(unsigned int pin, uint8_t port)
{
	uint32_t portVal;

	portVal = (uint8_t)( ( GPIO_ReadValue(port)>>pin ) & 0x01 );

	return (portVal);
}


void gpio_disable_gpio_pin_output(unsigned int pin, uint8_t port ) {
  // turns off the output driver for a pin.  useful for pseudo-open-drain applications
}

void reset_FPGA(void)
{
	GPIO_SetDir( 0, 0x01<<6, OUTPUT );
	GPIO_ClearValue( 0, 0x01<<6 );
	delay(  );
	GPIO_SetValue( 0, 0x01<<6 );
	GPIO_SetDir( 0, 0x01<<6, INPUT );
}


uint8_t checkDONEpin( void )
{
	return ( gpio_get_pin_value(FPGA_DONE_PIN, FPGA_DONE_PORT) );
}


uint8_t checkPGOODpin( void )
{
	return ( gpio_get_pin_value(PGOOD_PIN, PGOOD_PORT) );
}


uint8_t checkMasterRESETpin( void )
{
	return ( gpio_get_pin_value( MASTER_RST_PIN, MASTER_RST_PORT ) );
}

void setDC_DC_ConvertersON( void )
{
  GPIO_SetValue(PORT_EN_0, 0x01<<EN_P1V2);
  GPIO_SetValue(PORT_EN_0, 0x01<<EN_P1V8);
  GPIO_SetValue(PORT_EN_0, 0x01<<EN_FMC2_P3V3);
  GPIO_SetValue(PORT_EN_0, 0x01<<EN_FMC1_P3V3);
  GPIO_SetValue(PORT_EN_0, 0x01<<EM_FMC1_P12V);
  GPIO_SetValue(PORT_EN_0, 0x01<<EN_FMC2_P12V);
  GPIO_SetValue(PORT_EN_1, 0x01<<EN_FMC1_PVADJ);
  GPIO_SetValue(PORT_EN_1, 0x01<<EN_FMC2_PVADJ);
  GPIO_SetValue(PORT_EN_1, 0x01<<EN_P3V3);
  GPIO_SetValue(PORT_EN_1, 0x01<<EN_1V5_VTT);
  GPIO_SetValue(PORT_EN_3, 0x01<<EN_P1V0);
}

void setDC_DC_ConvertersOFF( void )
{
  GPIO_ClearValue(PORT_EN_0, 0x01<<EN_P1V2);
  GPIO_ClearValue(PORT_EN_0, 0x01<<EN_P1V8);
  GPIO_ClearValue(PORT_EN_0, 0x01<<EN_FMC2_P3V3);
  GPIO_ClearValue(PORT_EN_0, 0x01<<EN_FMC1_P3V3);
  GPIO_ClearValue(PORT_EN_0, 0x01<<EM_FMC1_P12V);
  GPIO_ClearValue(PORT_EN_0, 0x01<<EN_FMC2_P12V);
  GPIO_ClearValue(PORT_EN_1, 0x01<<EN_FMC1_PVADJ);
  GPIO_ClearValue(PORT_EN_1, 0x01<<EN_FMC2_PVADJ);
  GPIO_ClearValue(PORT_EN_1, 0x01<<EN_P3V3);
  GPIO_ClearValue(PORT_EN_1, 0x01<<EN_1V5_VTT);
  GPIO_ClearValue(PORT_EN_3, 0x01<<EN_P1V0);
}

uint8_t getRTM_Presence( void )
{
  return (gpio_get_pin_value(RTM_PRESENCE, PORT_EN_0));
}

void setRTM_ConnectorON( void )
{
 // if(getRTM_Presence())
    GPIO_SetValue(PORT_EN_1, 0x01<<EN_RTM_CONN);
}

void setRTM_ConnectorOFF( void )
{
  GPIO_ClearValue(PORT_EN_1, 0x01<<EN_RTM_CONN);
}
//////////////////////////////////////////////////////////////////
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
