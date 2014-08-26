
#include "LPC17xx.h"

#include <stdio.h>

#include "../include/utils.h"
#include "../stdPeriphLibs/lpc17xx_wdt.h"


void Enable_Watchdog(void) {

	WDT_Init( WDT_CLKSRC_PCLK, WDT_MODE_RESET );
	WDT_Start( 1000000 ); //1 second timeout

}


void Disable_Watchdog(void) {

	LPC_WDT->WDMOD = 0x00;

}

void Disable_global_interrupt(  )
{
    __disable_irq();

//	CurrentInteruptMask = NVIC->ISER[0];
//	NVIC->ICER[0] = 0xFFFFFFFF;
}
void Enable_global_interrupt(  )
{
    __enable_irq();
    //NVIC->ISER[0] = CurrentInteruptMask;
}


Bool Is_global_interrupt_enabled(void)
{
	if( NVIC->ISER[0] )
		return (TRUE);
	else
		return (FALSE);
}

void Int_Disable(Bool giflag)
{
   if (giflag)
    Disable_global_interrupt();
}

void Int_Restore(Bool giflag)
{
  if (giflag)
    Enable_global_interrupt();
}

