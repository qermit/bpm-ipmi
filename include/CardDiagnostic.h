/*
 * CardDiagnostic.h
 *
 *  Created on: 27-08-2013
 *      Author: Bartek
 */

#ifndef CARDDIAGNOSTIC_H_
#define CARDDIAGNOSTIC_H_

#include "../stdPeriphLibs/lpc_types.h"
#include "../include/diagnosticStructure.h"

#define WR_COMMAND 0x80
#define RD_COMMAND 0x00

#define FPGA_MEM_ADDR_MAX 22//20

typedef struct
{
	uint8_t		sensorSlaveAddr;
	uint8_t		rawDataPtr;
	uint8_t 	regAddrPtr;
	uint8_t 	sensorPtr;
	uint8_t		bytesToSend;
	uint8_t 	bytesToReadFromReg[MAX_FIELD_TO_SEND];

}Diagnostic_I2C_T;

extern const uint8_t INA220_ADDR_table[6];

void sendSPIdata( uint16_t address, uint32_t data );

void initDiagnosticMonitor( void );

void diagnosticMonitorCallback( FunctionalState newState );

void I2C1_IRQHandler( void );
void TIMER3_IRQHandler( void );

uint8_t getSensorData(uint8_t  sensorNum);

uint8_t calculateMedTemp(uint8_t param);
uint8_t calculateMaxTemp(uint8_t param);




#endif /* CARDDIAGNOSTIC_H_ */
