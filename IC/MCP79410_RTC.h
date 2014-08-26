/*
 * MCP79410_RTC.h
 *
 *  Created on: 21-08-2013
 *      Author: Bartek
 */

#ifndef MCP79410_RTC_H_
#define MCP79410_RTC_H_

// RTC_MEMORY MAP

/* MCP79410_SEC
 * bit 7 - start/stop oscillator (1 - on / 0 - off)
 * bit 6-4 - 10 seconds
 * bit 3-0 - seconds */
#define MCP79410_SEC 	0x00
/* MCP79410_MIN
 * bit 6-4 - 10 minutes
 * bit 3-0 - minutes */
#define MCP79410_MIN 	0x01
/* MCP79410_HOUR
 * bit 6 - 24/12 hour format (0 indicates 24-h)
 * bit 5 - AM/PM or 10 hour (when 24-h format)
 * bit 4 - 10 hour
 * bit 3-0 - hour */
#define MCP79410_HOUR 	0x02
/* MCP79410_DAY
 * bit 5 - OSCON - is set by hardware when oscillator is working
 * bit 4 - Vbatt bit - is set when VCC falls down (clear by software)
 * bit 3 - VBATTEN (set to use external battery when VCC goes down)
 * bit 2-0 - day */
#define MCP79410_DAY 	0x03
/* MCP79410_DOM
 * bit 5-4 - 10 day of month
 * bit 3-0 - day of month */
#define MCP79410_DOM 	0x04
/* MCP79410_MONTH
 * bit 5 - leap year
 * bit 4 - 10 month
 * bit 3-0 - month */
#define MCP79410_MONTH 	0x05
/* MCP79410_YEAR
 * bit 7-4 - 10 year
 * bit 3-0 - year */
#define MCP79410_YEAR 	0x06
/* MCP79410_CTRL
 * bit 7 - OUT bit set logic level on the MFP
 * bit 6 - SQWE bit enables the divided output from oscillator when set
 * bit 5-4 - determines which alarms are active
 * 	00 - none
 * 	01 - alarm 0
 * 	10 - alarm 1
 * 	11 - both
 * bit 3 - EXTOSC enable bit, allow external signal to drive RTCC registers
 * bit 2-0 - internal divider for MFP
 * 	000 - 1Hz
 * 	001 - 4.096kHz
 * 	010 - 8.192kHz
 * 	011 - 32.768kHz */
#define MCP79410_CTRL 	0x07
#define MCP79410_CAL 	0x08
#define MCP79410_UID 	0x09

#define STRUCT_FIELD 9

#include "../stdPeriphLibs/lpc_types.h"

typedef struct{
	uint8_t address;
	uint8_t sec :4,
			sec10 :3,
			st:1;
	uint8_t min:4,
			min10:3;
	uint8_t hour:4,
			hour10:2,
			format:1;
	uint8_t day:3,
			vbaten:1,
			vbat:1,
			oscon:1;
	uint8_t date:4,
			date10:2;
	uint8_t month:4,
			month10:1,
			lp:1;
	uint8_t year:4,
			year10:4;
	uint8_t rs0:1,
			rs1:1,
			rs2:1,
			extosc:1,
			alm0:1,
			alm1:1,
			sqwe:1,
			out:1;
}RTC_struct_T;


void MCP79410_Configure( void );
void MCP79410_GetData( void );


#endif /* MCP79410_RTC_H_ */
