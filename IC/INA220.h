/*
 * INA220.h
 *
 *  Created on: 01-07-2013
 *      Author: Bartek
 */

#ifndef INA220_H_
#define INA220_H_

#include "../stdPeriphLibs/lpc_types.h"

#define INA220_ADDRESS_CNT 6

#define INA220_ADDR1 0x45              //FMC1 P12V
#define INA220_ADDR2 0x44              //FMC1 P3V3
#define INA220_ADDR3 0x42              //FMC1 PADJ

#define INA220_ADDR4 0x40              //FMC2 P12V
#define INA220_ADDR5 0x43              //FMC2 P3V3
#define INA220_ADDR6 0x41              //FMC2 PADJ


#define INA220_CFG_REG 0x00
#define INA220_SHT_REG 0x01
#define INA220_BUS_REG 0x02
#define INA220_CAL_REG 0x05
#define INA220_CUR_REG 0x04
#define INA220_PWR_REG 0x03



typedef struct {
	uint8_t address;
	float BusVoltageRange; // default value
	float MeasuredBusVoltage;

}INA220_T;

void initINAsensors( void );
float get_INNA_curr ( volatile uint8_t* ptr );
float get_INNA_volt ( volatile uint8_t* ptr );

//float get_INA_data(uint8_t address);


#endif /* INA220_H_ */
