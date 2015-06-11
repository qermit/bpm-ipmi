/*
 * debug_pins.h
 *
 *  Created on: 11 cze 2015
 *      Author: pmiedzik
 */

#ifndef DEBUG_PINS_H_
#define DEBUG_PINS_H_

#define DBG_PIN_0_PORT 2
#define DBG_PIN_1_PORT 0
#define DBG_PIN_0_PIN 10
#define DBG_PIN_1_PIN 3

void debug_pins_set(unsigned char pin, unsigned char value);
void debug_pins_init(void);
void debug_pins_toggle(unsigned char pin);


#endif /* DEBUG_PINS_H_ */
