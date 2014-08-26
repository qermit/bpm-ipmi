#ifndef CMCPINS_H_
#define CMCPINS_H_

#include "../include/cardType.h"

// CMC pin definitions
// Port A
/* AMC GA0, GA1, GA2 lines GPIO IN  */

#define IPMI_GA_PORT 	1
#define IPMI_P1_PORT	1

#define	IPMI_GA2			4
#define IPMI_GA1			1
#define IPMI_GA0			0
#define IPMI_P1				8


//----------LED'S
#define LED_PORT 			1
#ifdef AMC_CPU_COM_Express
	#define	_IPMI_BLLED			10
	#define	_IPMI_LED1			9
	#define	_IPMI_LED2			25
#else
	#define	_IPMI_BLLED			9
	#define	_IPMI_LED1			10
	#define	_IPMI_LED2			25
#endif

//---------  HOT_SWAP
#define IPMI_EJTHDL_PORT 	2
#define IPMI_EJTHDL		13

//PAYLOAD_POWER lines
#define	UC_PWRENOUT_PORT	0
#define	UC_PWRENOUT		6

#define PGOOD_PORT		3
#define PGOOD_PIN		26

#define INTn_PORT		0
#define INTn_PIN		8

//--------- #ENABLE
#define ENA_PORT 2
#define ENA_PIN	 8

//---------ENABLE PINS
#define PORT_EN_0	 0
#define EN_P1V2		 23
#define EN_P1V8		 24
#define EN_FMC2_P3V3 25
#define EN_FMC1_P3V3 26
#define EM_FMC1_P12V 4
#define EN_FMC2_P12V 5
#define RTM_PRESENCE 29

#define PORT_EN_1	1
#define EN_FMC1_PVADJ 31
#define EN_FMC2_PVADJ 28
#define EN_P3V3		  27
#define EN_1V5_VTT	  29
#define EN_RTM_CONN       30

#define PORT_EN_3	3
#define EN_P1V0		25

//-----------DONE PINS
#define FPGA_DONE_PORT 0
#define FPGA_DONE_PIN  22

//Gloabal RESETn
#define MASTER_RST_PORT	1
#define MASTER_RST_PIN	22

#endif /* CMCPINS_H_ */
