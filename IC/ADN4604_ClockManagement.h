/*
 * ADN4604_ClockManagement.h
 *
 *  Created on: 07-06-2013
 *      Author: Bartek
 */

#ifndef ADN4604_CLOCKMANAGEMENT_H_
#define ADN4604_CLOCKMANAGEMENT_H_

#include <stdint.h>

//-----------------------------
//clk mgnt reg
#define ADN_ADDRESS 0x4B //0x96
#define RESET_ADN 	0x00
#define RX_EQ_C0	0x10
#define RX_EQ_C1	0x11
#define RX_C0		0x12
#define RX_C1		0x13


#define TX_BC_BROAD 0x18
#define TX_BC_OUT_0 0x20
#define TX_BC_OUT_1 0x21
#define TX_BC_OUT_2 0x22
#define TX_BC_OUT_3 0x23
#define TX_BC_OUT_4 0x24
#define TX_BC_OUT_5 0x25
#define TX_BC_OUT_6 0x26
#define TX_BC_OUT_7 0x27
#define TX_BC_OUT_8 0x28
#define TX_BC_OUT_9 0x29
#define TX_BC_OUT_10 0x2A
#define TX_BC_OUT_11 0x2B
#define TX_BC_OUT_12 0x2C
#define TX_BC_OUT_13 0x2D
#define TX_BC_OUT_14 0x2E
#define TX_BC_OUT_15 0x2F

#define TX_TABLE_D0_0 0x60
#define TX_TABLE_D0_1 0x62
#define TX_TABLE_D0_2 0x64
#define TX_TABLE_D0_3 0x66
#define TX_TABLE_D0_4 0x68
#define TX_TABLE_D0_5 0x6A
#define TX_TABLE_D0_6 0x6C
#define TX_TABLE_D0_7 0x6E

#define TX_TABLE_D1_0 0x61
#define TX_TABLE_D1_1 0x63
#define TX_TABLE_D1_2 0x65
#define TX_TABLE_D1_3 0x67
#define TX_TABLE_D1_4 0x69
#define TX_TABLE_D1_5 0x6B
#define TX_TABLE_D1_6 0x6D
#define TX_TABLE_D1_7 0x6F

#define TX_D0C_OUT0	0x30
#define TX_D0C_OUT1 0x32
#define TX_D0C_OUT2 0x34
#define TX_D0C_OUT3 0x36
#define TX_D0C_OUT4 0x38
#define TX_D0C_OUT5 0x3A
#define TX_D0C_OUT6 0x3C
#define TX_D0C_OUT7 0x3E
#define TX_D0C_OUT8 0x40
#define TX_D0C_OUT9 0x42
#define TX_D0C_OUT10 0x44
#define TX_D0C_OUT11 0x46
#define TX_D0C_OUT12 0x48
#define TX_D0C_OUT13 0x4A
#define TX_D0C_OUT14 0x4C
#define TX_D0C_OUT15 0x4E


#define TX_D1C_OUT0 0x31
#define TX_D1C_OUT1 0x33
#define TX_D1C_OUT2 0x35
#define TX_D1C_OUT3 0x37
#define TX_D1C_OUT4 0x39
#define TX_D1C_OUT5 0x3B
#define TX_D1C_OUT6 0x3D
#define TX_D1C_OUT7 0x3F
#define TX_D1C_OUT8 0x41
#define TX_D1C_OUT9 0x43
#define TX_D1C_OUT10 0x45
#define TX_D1C_OUT11 0x47
#define TX_D1C_OUT12 0x49
#define TX_D1C_OUT13 0x4B
#define TX_D1C_OUT14 0x4D
#define TX_D1C_OUT15 0x4F

#define TERMINATIO_CONTROL 0xF0

#define XPT_UPDATE 			0x80
#define XPT_MAP_TABLE_SEL 	0x81
#define XPT_BROADCAST 		0x82

#define XPT_MAP0_C0 0x90
#define XPT_MAP0_C1 0x91
#define XPT_MAP0_C2 0x92
#define XPT_MAP0_C3 0x93
#define XPT_MAP0_C4 0x94
#define XPT_MAP0_C5 0x95
#define XPT_MAP0_C6 0x96
#define XPT_MAP0_C7 0x97


#define XPT_MAP1_C0 0x98
#define XPT_MAP1_C1 0x99
#define XPT_MAP1_C2 0x9A
#define XPT_MAP1_C3 0x9B
#define XPT_MAP1_C4 0x9C
#define XPT_MAP1_C5 0x9D
#define XPT_MAP1_C6 0x9E
#define XPT_MAP1_C7 0x9F


#define XPT_STATUS_0 0xB0
#define XPT_STATUS_1 0xB1
#define XPT_STATUS_2 0xB2
#define XPT_STATUS_3 0xB3
#define XPT_STATUS_4 0xB4
#define XPT_STATUS_5 0xB5
#define XPT_STATUS_6 0xB6
#define XPT_STATUS_7 0xB7


uint8_t initClockMgnt( void );

#endif /* ADN4604_CLOCKMANAGEMENT_H_ */
