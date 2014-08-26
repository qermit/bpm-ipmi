
#include "../include/cmcpins.h"

#include "../include/utils.h"
#include "../include/gpio.h"
#include "../include/ejecthandle.h"
#include "../include/swevent.h"
#include "../include/interruptPriorities.h"
#include "../include/timer_callback.h"

#include "../stdPeriphLibs/lpc17xx_exti.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"


#define SWITCHPOLLLIMIT		(5)					// number of consecutive polls to accept new state

volatile uint8_t ejcetHandleFlag = 0x00;

volatile Eject_Hdl_Pos 		HandleStableState;

volatile ptrDriverISRCallback	hdlchgcallbackptr;

#define gethandlepinstate( )					( (gpio_get_pin_value(IPMI_EJTHDL, IPMI_EJTHDL_PORT) == 0 ) ? in_closed : out_open)



void eject_handle_init(void)
{

    PINSEL_CFG_Type PinCfg;
    EXTI_InitTypeDef EXTICfg;

    /* Initialize EXT pin and register */
    /* P2.13 as /EINT3*/
    PinCfg.Funcnum   = PINSEL_FUNC_1;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
    PinCfg.Pinnum    = PINSEL_PIN_13;
    PinCfg.Portnum   = PINSEL_PORT_2;
    PINSEL_ConfigPin(&PinCfg);
    EXTI_Init();

    HandleStableState = gethandlepinstate();

    EXTICfg.EXTI_Line = EXTI_EINT3;
    EXTICfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
    EXTICfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
    EXTI_ClearEXTIFlag(EXTI_EINT3);
    EXTI_Config(&EXTICfg);

    GPIO_IntCmd(IPMI_EJTHDL_PORT, 0x01 << IPMI_EJTHDL, 0);
    GPIO_IntCmd(IPMI_EJTHDL_PORT, 0x01 << IPMI_EJTHDL, 1);

    NVIC_SetPriorityGrouping(IPMI_EJCET_PriorGrup);
    NVIC_SetPriority(EINT3_IRQn, IPMI_EJCET_Prior);
    NVIC_EnableIRQ(EINT3_IRQn);

}


void EINT3_IRQHandler(void)
{
  /* clear the EINT0 flag */
  EXTI_ClearEXTIFlag(EXTI_EINT3);
  GPIO_ClearInt( IPMI_EJTHDL_PORT, 0x01 << IPMI_EJTHDL );

  ejcetHandleFlag = 0xFF;

}


void register_eject_handle_change_callback(ptrDriverISRCallback funcaddr)
{
  // this function is called when the eject handle changes state
  hdlchgcallbackptr = funcaddr;
}


Eject_Hdl_Pos get_eject_handle_stable_state(void)
{
  return (HandleStableState);
}

void ejcetHandleChange( void )
{
  Disable_global_interrupt();

  volatile Eject_Hdl_Pos OverridePosition = gethandlepinstate();

  if(HandleStableState != OverridePosition)
    {
      HandleStableState = OverridePosition;
      if (hdlchgcallbackptr != 0)
        (*hdlchgcallbackptr)();
    }

  ejcetHandleFlag = 0x00;

  Enable_global_interrupt();
}
