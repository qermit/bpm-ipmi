/*
 * comExpress.c
 *
 *  Created on: 29-08-2013
 *      Author: Bartek
 */

#include <stdint.h>
#include "../stdPeriphLibs/lpc17xx_gpio.h"
#include "../include/comExpress.h"
#include "../stdPeriphLibs/lpc17xx_timer.h"

static void delay(  void );
static void TIM2_CFG(void);

void initComExpress( void )
{

        GPIO_ClearValue(0, 0x01<<9);
        TIM2_CFG();
       // delay();
	GPIO_SetValue(3, 0x01<<25);


	GPIO_ClearValue(1, 0x01<<18);
	delay();
	GPIO_SetValue(1, 0x01<<18);

	GPIO_SetValue(0, 0x01<<4);

	//GPIO_SetValue(0, 0x01<<9);

	GPIO_SetValue(2, 0x01<<5);
	GPIO_SetValue(2, 0x01<<6);
}

static void TIM2_CFG(  )
{
  TIM_TIMERCFG_Type TMR2_Cfg;
  TIM_MATCHCFG_Type TMR2_Match;

  /* On reset, Timer0/1 are enabled (PCTIM0/1 = 1), and Timer2/3 are disabled (PCTIM2/3 = 0).*/
  /* Initialize timer 0, prescale count time of 100uS */
  TMR2_Cfg.PrescaleOption = TIM_PRESCALE_USVAL;
  TMR2_Cfg.PrescaleValue = 10000;
  /* Use channel 0, MR0 */
  TMR2_Match.MatchChannel = 0;
  /* Enable interrupt when MR0 matches the value in TC register */
  TMR2_Match.IntOnMatch = ENABLE;
  /* Enable reset on MR0: TIMER will reset if MR0 matches it */
  TMR2_Match.ResetOnMatch = FALSE;
  /* Don't stop on MR0 if MR0 matches it*/
  TMR2_Match.StopOnMatch = TRUE;
  /* Do nothing for external output pin if match (see cmsis help, there are another options) */
  TMR2_Match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
  /* Set Match value, count value of 2 (2 * 100uS = 200us ) */
  TMR2_Match.MatchValue = 250;
  /* Set configuration for Tim_config and Tim_MatchConfig */
  TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TMR2_Cfg);
  TIM_ConfigMatch(LPC_TIM2, &TMR2_Match);

  NVIC_SetPriorityGrouping(3);
  /* Preemption = 1, sub-priority = 1 */
  NVIC_SetPriority(TIMER2_IRQn, 2);
  /* Enable interrupt for timer 0 */
  NVIC_EnableIRQ(TIMER2_IRQn);
  /* Start timer 0 */
  TIM_Cmd(LPC_TIM2, ENABLE);
}

void TIMER2_IRQHandler(  )
{
  GPIO_SetValue(0, 0x01<<9);
  TIM_ClearIntPending(LPC_TIM2, TIM_MR0_INT);
//  TIM_Cmd(LPC_TIM2, DISABLE);
}

static void delay(  ) {
uint16_t i = 0;
uint8_t z = 0;

	for(i=0; i<20000; i++)
	{
		for(z=0; z<1000; z++)
		{

		}
	}

}
