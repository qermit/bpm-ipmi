

#include <stdio.h>

#include "../include/utils.h"
#include "../include/timer_callback.h"
#include "../include/ejecthandle.h"

#include "../include/gpio.h"
#include "../stdPeriphLibs/lpc17xx_gpio.h"

#include "../include/rtc.h"

#include "../stdPeriphLibs/lpc17xx_timer.h"

#include "../include/interruptPriorities.h"

#define TIMERCALLBACKTBLSIZE    5

volatile uint8_t  event_100us = 0;
volatile uint8_t  event_10ms = 0;
volatile uint8_t  event_100ms = 0;


volatile CallbackTblEntry TimerCallbackTbl[TIMERCALLBACKTBLSIZE];

static void make_callbacks(event eventID);

void TIMER0_IRQHandler (void)
{

  if(TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
      /*  Clear Interrupt */
      TIM_ClearIntPending(LPC_TIM0,TIM_MR0_INT);

      make_callbacks(TEVENT_100USEC);

      event_100us++;

      //gpio_tgl_gpio_pin(22,0);

/**
 * 10 msec call backs
 * ejcet handle debounce
 * systemTime i 10 msec
 */
      if(event_100us == 100)
        {
          make_callbacks(TEVENT_10MSEC);

          if(ejcetHandleFlag)
            ejcetHandleChange();

          systemUpTime++;
          event_100us = 0;
          event_10ms++;

          /**
           * 100 msec call backs
           */
          if(event_10ms == 10)
            {
              make_callbacks(TEVENT_100MSEC);
              event_100ms++;
              event_10ms = 0;

              /**
               * 1 seconds call backs
               */
              if(event_100ms == 10)
                {
                  make_callbacks(TEVENT_1SEC);
                  event_100ms = 0;
                }
            }
        }
     }
}


void timer_callback_init(void) {

	TIM_TIMERCFG_Type TMR0_Cfg;
	TIM_MATCHCFG_Type TMR0_Match;

	/* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
	/* Initialize timer 0, prescale count time of 1000uS */
	TMR0_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
	TMR0_Cfg.PrescaleValue = 10;
	/* Enable interrupt when MR0 matches the value in TC register */
	TMR0_Match.IntOnMatch = ENABLE;
	/* Enable reset on MR0: TIMER will reset if MR0 matches it */
	TMR0_Match.ResetOnMatch = TRUE;
	/* Don't stop on MR0 if MR0 matches it*/
	TMR0_Match.StopOnMatch = FALSE;
	/* Do nothing for external output pin if match (see cmsis help, there are another options) */
	TMR0_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	/* Set Match value, count value of 10 (10 * 1ms = 10ms ) */
	TMR0_Match.MatchValue = 10;
        /* Use channel 0, MR0 */
        TMR0_Match.MatchChannel = 0;
	/* Set configuration for Tim_config and Tim_MatchConfig */
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TMR0_Cfg);
	TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &TMR0_Cfg);



	TIM_ConfigMatch(LPC_TIM0, &TMR0_Match);

        NVIC_SetPriorityGrouping(TIMER0_PriorGrup);
	/* Preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, TIMER0_Prior );
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ( TIMER0_IRQn);

//	/* Start timer 0 */
//	TIM_Cmd(LPC_TIM0, ENABLE);
//	GPIO_SetDir(0, 0x01<<22, OUTPUT);

}



uint8_t register_timer_callback( ptrCallback funcaddr, event eventID )
{
  // installs callback in standard table.  Returns 0 if successful, 1 otherwise
  uint8_t i1;

  Disable_global_interrupt();

  for (i1=0; i1<TIMERCALLBACKTBLSIZE; i1++)
  {
	  if (!TimerCallbackTbl[i1].active)
	  {
		  TimerCallbackTbl[i1].active 	= 1;
		  TimerCallbackTbl[i1].eventID 	= eventID;
		  TimerCallbackTbl[i1].ptr 	= funcaddr;

		  Enable_global_interrupt();

		  return (0);
	  }
  }

  Enable_global_interrupt();

  return (1);   // table full
}


uint8_t unregister_timer_callback( ptrCallback funcaddr )
{
  // remove callback from standard table.  Returns 0 if successful, 1 otherwise
  uint8_t i1;

  Disable_global_interrupt();

  for (i1=0; i1<TIMERCALLBACKTBLSIZE; i1++)
  {
          if (TimerCallbackTbl[i1].ptr == funcaddr)
          {
                  TimerCallbackTbl[i1].active = 0;
                  Enable_global_interrupt();
                  return (0);
          }
  }

  Enable_global_interrupt();

  return (1);   // table full
}




static void make_callbacks(event eventID) {
  event cbe = eventID;
  uint8_t cbi;

  for (cbi=0; cbi<TIMERCALLBACKTBLSIZE; cbi++)
    if (TimerCallbackTbl[cbi].active)
      if (TimerCallbackTbl[cbi].eventID == cbe) {
        TimerCallbackTbl[cbi].active = (*TimerCallbackTbl[cbi].ptr)(cbe, NULL);
      }
}


void start_timers(void) {
	/* Start timer 0 */
	TIM_Cmd(LPC_TIM0, ENABLE);

}

