
#ifndef TIMER_CALLBACK_H_
#define TIMER_CALLBACK_H_

#include "events.h"

void timer_callback_init(void);

uint8_t register_timer_callback(ptrCallback funcaddr, event eventID);

uint8_t unregister_timer_callback( ptrCallback funcaddr );

void start_timers(void);

void TIMER0_IRQHandler (void);

#endif /* TIMER_CALLBACK_H_ */
