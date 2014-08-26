/**
 *=====================================\n
 * Name        : main.c\n
 * Author      :\n
 * Version     :\n
 * Copyright   : Copyright (C) \n
 * Description : main definition \n
 *=====================================\n
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

/**
 * Variable to store CRP value in. Will be placed automatically
 * by the linker when "Enable Code Read Protect" selected.
 * See crp.h header for more information
 */
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#include <stdio.h>
#include "include/cardType.h"

#include "include/ipmiCommonHeader.h"

#include "IC/ic_commonHeader.h"

#include "include/CardDiagnostic.h"

#include "include/cardStatus.h"

#include "include/iap_driver.h"

#include "stdPeriphLibs/lpc_types.h"

int main(void)
{

  ///System clock, peripherals power and flash memory initialization.\n
  SystemInit();
  SystemCoreClockUpdate();

  /// Disable watchdog in case of unexpected reset.
  /// Watchdog could be enabled in previous cycle
  /// by MCH to perform AMC card reset.\n
  Disable_Watchdog();

  Disable_global_interrupt();

  ///uC pins assignments according to their function.\n
  gpio_init();
  ///Prepare main timer
  ///NOTICE! Timer is not enable after this function.\n
  timer_callback_init();
  ///Serial port initialization. It will be used as IPMI console.\n
  sio_init();
  ///Initialize I2C bus (IPMB) to communicate with MCH.\n
  ipmi_i2c_init( );

//TODO  rtc_init();
  ///Initialize hot swap switch.\n
  eject_handle_init();
  ///Initialize IPMI LED's.\n
  LED_init();

  Enable_global_interrupt();

  sio_putstr("\nIPMI MODULE MANAGEMENT\n");
  sprintf(spbuf, "uTCA Slot=%i  IPMB-L ADDR=%02Xh\n", ipmi_i2c_state.slotid, ipmi_i2c_state.ipmbl_addr);
  sio_putstr(spbuf);

  ///Initialize diagnostic monitor.\n
  //initDiagnosticMonitor();


  ///IPMI services initialization.\n
  fru_init();                           /// 1.FRU initialization\n

  swevent_init();			/// 2.Software event service\n

  sensor_svc_init();                    /// 3.Sensor SDR service initialization\n

  ipmb_init();				/// 4.Ipmb message service\n

  ipmi_cmd_parser_init();		/// 5.Register command parser service events\n

  pyldmgr_init();                	/// 6.Payload manager initialization\n

  start_timers();			/// 7.Start main timer\n

  ipmi_i2c_start_slave_listen();	/// 8.Put i2c-interface in listening mode\n

//  initDiagnosticMonitor();
//  diagnosticMonitorCallback (ENABLE);

  //program_LED(LED0_TBL_IDX, Local_Control, &LED_1Hz_Blink_Activity);
  program_LED(IPMI_BLUELED_TBL_IDX, Local_Control, &LED_On_Activity);
  // turn on frontpanel LEDs for test display until payload power is activated
  program_LED(IPMI_LED1_TBL_IDX, Local_Control, &LED_2Hz_Blink_Activity);
  program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Off_Activity);

#ifdef AMC_FMC_uBackplane
  setDC_DC_ConvertersON();
#endif

  /// Main infinite loop\n
  while (1)
    {
      ipmb_service();
      sensor_service();
      pyldmgr_service();

      //console_chk_cmd();

      //commented for test purposes
      checkCardStatus( );


      /// <b>TEST SYSCLK</b>\n
      /// Un-comment to check system clock frequency.
      /// Signal should appear at P1.27\n
//       LPC_SC->CLKOUTCFG = 0x100;
//       LPC_PINCON->PINSEL3 |= 0x01<<22;
    }

 return (0);
}
