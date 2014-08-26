/*
 * comExpress.h
 *
 *  Created on: 29-08-2013
 *      Author: Bartek
 */

#ifndef COMEXPRESS_H_
#define COMEXPRESS_H_

extern volatile uint8_t commExpressLoggedIn;
extern volatile uint8_t COM_Express_initialized;

void TIMER2_IRQHandler( void );
void initComExpress( void );

#endif /* COMEXPRESS_H_ */
