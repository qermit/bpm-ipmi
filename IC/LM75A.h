/*
 * LM75A.h
 *
 *  Created on: 10-04-2013
 *      Author: Bartek
 */

#ifndef LM75A_H_
#define LM75A_H_

#include <stdint.h>
#include "../stdPeriphLibs/lpc_types.h"

#define TEMP_REG 	0x00
#define CONF_REG 	0x01
#define THYST_REG 	0x02
#define TOS_REG		0x03
#define ID_REG 		0x07

#define LM_SENSOR_CNT 5
/** keep addresses in current order to maintain
 *  proper id for all measurements
 */
#define LM75_1_ADDR	0x4C    //FMC1
#define LM75_2_ADDR     0x4D    //FMC2
#define LM75_3_ADDR	0x4E    //DC/DC Converters
#define LM75_4_ADDR	0x4F    //SRAM


float get_LM_temp ( volatile uint8_t* ptr );


//typedef struct
//{
//	uint8_t address;
//	uint8_t OS_Value;
//	uint8_t HYST_Value;
//	uint8_t temperature;
//
//}sensor_unique_data_T;
//
//typedef enum { AMBIENT = 0, HOT_SPOT = 1 } temperature_T;
//
//sensor_unique_data_T sensorTable[LM_SENSOR_CNT];
//
//void initLM75( void );
//void get_LM75_Temp ( void );
//uint8_t return_TEMP( temperature_T tempType );


#endif /* LM75A_H_ */
