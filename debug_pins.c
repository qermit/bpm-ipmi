/*
 * debug_pins.c
 *
 *  Created on: 11 cze 2015
 *      Author: pmiedzik
 */

#include "include/cmcpins.h"
#include "include/debug_pins.h"
#include "stdPeriphLibs/lpc17xx_gpio.h"
#include "stdPeriphLibs/lpc17xx_pinsel.h"
#include "stdPeriphLibs/lpc17xx_spi.h"


#define DBG_PIN_COUNT 2
volatile uint8_t dbg_pin_state[2];


void debug_pins_set(unsigned char pin, unsigned char value)
{
	uint8_t tmp_pin  = DBG_PIN_0_PIN;
	uint8_t tmp_port = DBG_PIN_0_PORT;

	if (pin == 0) {
		tmp_pin = DBG_PIN_0_PIN;
		tmp_port = DBG_PIN_0_PORT;
	} else if (pin == 1) {
		tmp_pin = DBG_PIN_1_PIN;
		tmp_port = DBG_PIN_1_PORT;
	} else {
		return;
	}

	if (value == 0) {
		GPIO_ClearValue(tmp_port, 1<<tmp_pin);
		dbg_pin_state[pin] = 0;
	} else {
		GPIO_SetValue(tmp_port, 1<<tmp_pin);
		dbg_pin_state[pin] = 1;
	}
}



void debug_pins_toggle(unsigned char pin)
{
	if (pin >= DBG_PIN_COUNT) return;
	debug_pins_set(pin, 1 - dbg_pin_state[pin]);
}

void debug_pins_init(void)
{
	PINSEL_CFG_Type PinSelCfg;

	GPIO_SetDir(DBG_PIN_0_PORT, 1<<DBG_PIN_0_PIN, OUTPUT);
	GPIO_SetDir(DBG_PIN_1_PORT, 1<<DBG_PIN_1_PIN, OUTPUT);

	PinSelCfg.Portnum = DBG_PIN_0_PORT;
	PinSelCfg.Pinnum = DBG_PIN_0_PIN;
	PinSelCfg.Funcnum = PINSEL_FUNC_0;
	PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinSelCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PINSEL_ConfigPin(&PinSelCfg);

	PinSelCfg.Portnum = DBG_PIN_1_PORT;
	PinSelCfg.Pinnum = DBG_PIN_1_PIN;
	PINSEL_ConfigPin(&PinSelCfg);


	debug_pins_set(0, 0);
	debug_pins_set(1, 0);
	debug_pins_set(0, 1);
	debug_pins_set(1, 1);
	debug_pins_set(0, 0);
	debug_pins_set(1, 0);
}

