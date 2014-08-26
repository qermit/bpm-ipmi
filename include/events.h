#ifndef EVENTS_H_
#define EVENTS_H_

#include "../stdPeriphLibs/lpc_types.h"

// timer event types
#define TEVENT_100USEC                  (0)                         // 200 usec event ID
#define TEVENT_200USEC			(1)			    // 200 usec event ID
//#define TEVENT_2MSEC			(1)			    // 2 msec event ID
#define TEVENT_10MSEC			(2)                         // 10 msec eventID
#define TEVENT_100MSEC			(3)			    // 100 msec event ID
#define TEVENT_1SEC			(4)			    // 1 second event type (timer ISR)

// IPMB service
#define IPMBEV_REQ_RCVD			(7)			    // request received
#define IPMBEV_UNACK_REQ_MSG		(8)			    // request message transmitted max # of times w/o acknowledge
#define IPMBEV_UNMATCHED_RSP_MSG	(9)			    // cannot find match in request tbl for rcv'd response msg

// Sensor service values
#define SENSOR_DIAGNOSTIC_UPDATE          (10)        // new conversion readings available from sensor ADCs
#define SENSOR_EJCT_HDL_CHG               (6)         // handle position change
#define SENSOR_GPIO_READ                  (5)         // read GPIO sensor

// Payload Manager Events
#define PYLDMGREV_PAYLD_PWR_ON_DETECT     (11)        // Payload Power (+12) on detected (above lower non-critical)
#define PYLDMGREV_PAYLD_PWR_OFF_DETECT    (12)        // Payload Power (+12) off detected (below 10% of nominal)
#define PYLDMGREV_FPGA_AUTOCONFIG_SVC     (14)        // Autoconfig service event
// RTC
#define RTC_1SEC_EVENT                    (13)        // Value change detected on RTC



typedef volatile uint16_t event;

typedef uint8_t(*ptrCallback)(event, void*);

typedef void(*ptrDriverISRCallback)(void);

typedef struct {
  uint8_t  		active;
  event   	 	eventID;
  ptrCallback 	ptr;
} CallbackTblEntry;


#endif /* EVENTS_H_ */
