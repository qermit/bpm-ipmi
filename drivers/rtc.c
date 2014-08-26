
#include <stdio.h>
#include "../stdPeriphLibs/lpc17xx_timer.h"
#include "../include/rtc.h"

/**TODO - systemUpTime count 10ms ticks of TIMER0,
 * but it should be changed to meet IPMI documentation assumptions
 */
volatile uint32_t systemUpTime = 0;


uint32_t get_rtc_value(void) {
  return (systemUpTime);
}


void set_rtc_value(uint32_t systime) {
	systemUpTime = systime;
}

