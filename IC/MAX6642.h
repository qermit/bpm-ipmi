/*
 * MAX6642.h
 *
 *  Created on: 26-08-2013
 *      Author: Bartek
 */

#ifndef MAX6642_H_
#define MAX6642_H_

#include "../stdPeriphLibs/lpc_types.h"

#define MAX6642_ADDRESS 0x48

#define MAX6642_LOCAL_TEMP_R 	0x00
#define MAX6642_REMOTE_TEMP_R 	0x01
#define MAX6642_STATUS_R	0x02
#define MAX6642_CFG_R	 	0x03
#define MAX6642_LOCAL_LIMIT_R	0x05
#define MAX6642_REMOTE_LIMIT_R	0x07
#define MAX6642_CFG_W 		0x09
#define MAX6642_LOCAL_LIMIT_W	0x0B
#define MAX6642_REMOTE_LIMIT_W	0x0D
#define MAX6642_SINGLE_SHOT	0x0F
#define MAX6642_REMOTE_EXT_R 	0x10
#define MAX6642_LOCAL_EXT_R	0x11
#define MAX6642_ID		0xFE


float readFPGATemp ( volatile uint8_t* ptr );
void readMAXTemp ( void );

#endif /* MAX6642_H_ */
