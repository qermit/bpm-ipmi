/*
 * diagnosticStructure.h
 *
 *  Created on: 22-08-2013
 *      Author: Bartek
 */

#ifndef DIAGNOSTICSTRUCTURE_H_
#define DIAGNOSTICSTRUCTURE_H_

#include "../stdPeriphLibs/lpc_types.h"
#include "../include/cardType.h"

#ifdef AMC_FMC_Carierr
      #define TEMP_SENSORS_CNT             5
      #define VOLT_SENSORS_CNT             6
      #define CURR_SENSORS_CNT             6
      #define BOARD_SENSORS  (TEMP_SENSORS_CNT + VOLT_SENSORS_CNT + CURR_SENSORS_CNT)
#endif


#ifdef AMC_PCIe_Adapter
      #define TEMP_SENSORS_CNT                5
      #define CUR_PWR_SENSORS_CNT             0
#endif

#ifdef AMC_CPU_COM_Express
      #define TEMP_SENSORS_CNT                3
      #define CURR_SENSORS_CNT                0
      #define VOLT_SENSORS_CNT                0
      #define BOARD_SENSORS  (TEMP_SENSORS_CNT)
#endif



#define MAX_FIELD_TO_READ 4
#define MAX_FIELD_TO_SEND 2

typedef float (*ptrReadOutVal)(volatile uint8_t*);


typedef struct {

      float                   Val1;

      uint8_t                 rawData[MAX_FIELD_TO_READ];
      uint8_t                 regAddr[MAX_FIELD_TO_SEND];

      uint8_t                 bytesToSend;
      uint8_t                 bytesToReadFromReg[MAX_FIELD_TO_SEND];

      ptrReadOutVal           ReadOutFunc;

      uint8_t                 i2c_addr;

} sensorStruct_T;


typedef struct {
      /**
       * id
       * temp sens
       *
       */
      uint32_t cardID [4];
      uint32_t AMC_NR;

      uint8_t sensorPtr;

      sensorStruct_T sensor [BOARD_SENSORS];

} SystemDiagnosticStruct_T;

typedef struct{

      uint32_t uniqueID[4];
      uint16_t IPMIaddress;
      uint16_t AMCposition;
      uint32_t dataValid;
      uint32_t sensorData[BOARD_SENSORS];

}DiagnosticSPI_T;












//#include "../stdPeriphLibs/lpc_types.h"
//#include "../include/cardType.h"
//
//#ifdef AMC_FMC_Carierr
//	#define TEMP_SENSORS_CNT 		5
//	#define CUR_PWR_SENSORS_CNT		6
//#endif
//
//
//#ifdef AMC_PCIe_Adapter
//	#define TEMP_SENSORS_CNT 		5
//	#define CUR_PWR_SENSORS_CNT		0
//#endif
//
//#ifdef AMC_CPU_COM_Express
//	#define TEMP_SENSORS_CNT 		3
//	#define CUR_PWR_SENSORS_CNT		0
//#endif
//
//#define BOARD_SENSORS  (TEMP_SENSORS_CNT + CUR_PWR_SENSORS_CNT)
//
//#define MAX_FIELD_TO_READ 10
//#define MAX_FIELD_TO_SEND 5
//
//typedef float (*ptrReadOutVal)(volatile uint8_t*, volatile uint8_t);
//
//
//typedef struct {
//
//	float	 		Val1;
//	float			Val2;
//
//	uint8_t 		rawData[MAX_FIELD_TO_READ];
//	uint8_t			regAddr[MAX_FIELD_TO_SEND];
//
//	uint8_t			bytesToSend;
//	uint8_t			bytesToReadFromReg[MAX_FIELD_TO_SEND];
//
//	ptrReadOutVal 	        ReadOutFunc;
//
////	uint8_t* 		FuncArg;
//	uint8_t			sType;
//	uint8_t			i2c_addr;
//} sensorStruct_T;
//
//
//typedef struct {
//	/**
//	 * id
//	 * temp sens
//	 *
//	 */
//	uint32_t cardID [4];
//	uint32_t AMC_NR;
//
//	uint8_t sensorPtr;
//
//	sensorStruct_T sensor [BOARD_SENSORS];
//
//} SystemDiagnosticStruct_T;
//
//typedef struct{
//
//	uint32_t uniqueID[4];
//	uint16_t IPMIaddress;
//	uint16_t AMCposition;
//	uint32_t dataValid;
//	uint32_t sensorData[BOARD_SENSORS];
//
//}DiagnosticSPI_T;


#endif /* DIAGNOSTICSTRUCTURE_H_ */
