
#ifndef SENSOR_SDR_H_
#define SENSOR_SDR_H_

#include "fru.h"
#include "ipmb_svc.h"

#define LOWBYTE             (0)
#define HIGHBYTE            (1)



#define SENSOR_CNT			6//	17

#define HOTSWAP_SENSOR			0

#define FPGA_TEMP_SENSOR		1
#define FMC1_TEMP_SENSOR		2
#define FMC2_TEMP_SENSOR		3
#define DC_DC_CONV_TEMP_SENSOR	        4
#define SDRAM_TEMP_SENSOR		5

#define FMC1_12V_VOLT_SENSOR	6
#define FMC1_12V_CURR_SENSOR	7
#define FMC1_3V3_VOLT_SENSOR	8
#define FMC1_3V3_CURR_SENSOR    9
#define FMC1_ADJ_VOLT_SENSOR	10
#define FMC1_ADJ_CURR_SENSOR    11

#define FMC2_12V_VOLT_SENSOR	12
#define FMC2_12V_CURR_SENSOR	13
#define FMC2_3V3_VOLT_SENSOR	14
#define FMC2_3V3_CURR_SENSOR    15
#define FMC2_ADJ_VOLT_SENSOR	16
#define FMC2_ADJ_CURR_SENSOR    17

#define FIRST_INT_SENSOR         FPGA_TEMP_SENSOR
#define LAST_INT_SENSOR          SDRAM_TEMP_SENSOR


#define MAX_SENSOR_CNT                          (1+SENSOR_CNT)
#define SDR_RECORD_LENGTH                       (64)
#define SDR_SIZE                                ((MAX_SENSOR_CNT+1)*SDR_RECORD_LENGTH)

#if SDR_AREA_SIZE < SDR_SIZE
#error "SDR area too small in nonvolatile storage"
#endif

#define SDR_CNT							     (1+SENSOR_CNT)  // now dynamic from interrogation
#define SDR_MMC							          (0)
#define SDR_HOTSWAP_SENSOR					    (1+HOTSWAP_SENSOR)

#define SDR_FPGA_TEMP_SENSOR					(1+FPGA_TEMP_SENSOR)
#define SDR_FMC1_TEMP_SENSOR					(1+FMC1_TEMP_SENSOR)
#define SDR_FMC2_TEMP_SENSOR				 	(1+FMC2_TEMP_SENSOR)
#define SDR_DC_DC_CONV_TEMP_SENSOR				(1+DC_DC_CONV_TEMP_SENSOR)
#define SDR_SDRAM_TEMP_SENSOR					(1+SDRAM_TEMP_SENSOR)

#define SDR_FMC1_12V_VOLT_SENSOR				(1+FMC1_12V_VOLT_SENSOR)
#define SDR_FMC1_12V_CURR_SENSOR				(1+FMC1_12V_CURR_SENSOR)
#define SDR_FMC1_3V3_VOLT_SENSOR				(1+FMC1_3V3_VOLT_SENSOR)
#define SDR_FMC1_3V3_CURR_SENSOR   				(1+FMC1_3V3_CURR_SENSOR)
#define SDR_FMC1_ADJ_VOLT_SENSOR				(1+FMC1_ADJ_VOLT_SENSOR)
#define SDR_FMC1_ADJ_CURR_SENSOR    			(1+FMC1_ADJ_CURR_SENSOR)

#define SDR_FMC2_12V_VOLT_SENSOR				(1+FMC2_12V_VOLT_SENSOR)
#define SDR_FMC2_12V_CURR_SENSOR				(1+FMC2_12V_CURR_SENSOR)
#define SDR_FMC2_3V3_VOLT_SENSOR				(1+FMC2_3V3_VOLT_SENSOR)
#define SDR_FMC2_3V3_CURR_SENSOR    			(1+FMC2_3V3_CURR_SENSOR)
#define SDR_FMC2_ADJ_VOLT_SENSOR				(1+FMC2_ADJ_VOLT_SENSOR)
#define SDR_FMC2_ADJ_CURR_SENSOR    			(1+FMC2_ADJ_CURR_SENSOR)

#define END_OF_SDR_LIST						      (0xFFFF)



// sensor definitions
//#define SENSOR_CNT						        (2)    // now dynamic from interrogation
//#define HOTSWAP_SENSOR						    (0)
//#define AMBIENT_TEMP_SENSOR						(1)
//#define HOTSPOT_TEMP_SENSOR		  	 	    	(2)
//#define PAYLOAD_12V_SENSOR					    (3)
//#define BACKEND_3p3V_SENSOR					    (4)
//#define ADC1_SENSOR                     (5)
//#define ADC3_SENSOR                     (6)
//#define ADC4_SENSOR                     (7)
//#define ADC5_SENSOR                     (8)
//#define GPIO_SENSOR                     (9)
//#define FIRST_INT_ANALOG_SENSOR         AMBIENT_TEMP_SENSOR
//#define LAST_INT_ANALOG_SENSOR          3

//#define SDR_CNT								          (1+SENSOR_CNT)  // now dynamic from interrogation
//#define SDR_MMC								          (0)
//#define SDR_HOTSWAP_SENSOR					    (1+HOTSWAP_SENSOR)
//#define SDR_AMBIENT_TEMP_SENSOR					(1+AMBIENT_TEMP_SENSOR)
//#define SDR_HOTSPOT_TEMP_SENSOR					(1+HOTSPOT_TEMP_SENSOR)
//#define SDR_PAYLOAD_12V_SENSOR				  (1+PAYLOAD_12V_SENSOR)
//#define SDR_BACKEND_3p3V_SENSOR				  (1+BACKEND_3p3V_SENSOR)
//#define SDR_ADC1_SENSOR                 		(1+ADC1_SENSOR)
//#define SDR_ADC3_SENSOR                 (1+ADC3_SENSOR)
//#define SDR_ADC4_SENSOR                 (1+ADC4_SENSOR)
//#define SDR_ADC5_SENSOR                 (1+ADC5_SENSOR)
//#define SDR_GPIO_SENSOR                 (1+GPIO_SENSOR)
//#define END_OF_SDR_LIST						      (0xFFFF)

// hot swap sensor state mask bits

#define HOTSWAP_MODULE_HANDLE_CLOSED_MASK   (0x01)
#define HOTSWAP_MODULE_HANDLE_OPEN_MASK     (0x02)
#define HOTSWAP_QUIESCED_MASK               (0x04)
#define HOTSWAP_BACKEND_PWR_FAILURE_MASK    (0x08)
#define HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK   (0x10)

// hot swap sensor event codes
#define HOTSWAP_EVENT_HANDLE_CLOSED         (0)
#define HOTSWAP_EVENT_HANDLE_OPENED         (1)
#define HOTSWAP_EVENT_QUIESCED              (2)
#define HOTSWAP_EVENT_BACKEND_FAILURE       (3)
#define HOTSWAP_EVENT_BACKEND_SHUTDOWN      (4)

// common constants found in SDR records
#define SDR_VERSION							   (0x51)
#define ENTITY_ID							   (0xc1)
#define ENTITY_INSTANCE_BASE				                   (0x60)

// context codes for when sensors are considered active
#define SENSORACTV_ALWAYS                   (0)                 // active whenever MMC is alive
#define SENSORACTV_PAYLOADPWRON             (1)                 // active when payload power is on
#define SENSORACTV_BACKENDPWRON             (2)                 // active when backend power is on

typedef uint8_t(*pGetReadoutVal)(uint8_t);		// type definition for function that gets a sensor readout value

// sensor threshold event definitions
#define SENSOREV_UPPER_NONRECOVER_HIGH_OFFSET     (11)
#define SENSOREV_UPPER_NONRECOVER_LOW_OFFSET      (10)
#define SENSOREV_UPPER_CRITICAL_HIGH_OFFSET       (9)
#define SENSOREV_UPPER_CRITICAL_LOW_OFFSET        (8)
#define SENSOREV_UPPER_NONCRITICAL_HIGH_OFFSET    (7)
#define SENSOREV_UPPER_NONCRITICAL_LOW_OFFSET     (6)
#define SENSOREV_LOWER_NONRECOVER_HIGH_OFFSET     (5)
#define SENSOREV_LOWER_NONRECOVER_LOW_OFFSET      (4)
#define SENSOREV_LOWER_CRITICAL_HIGH_OFFSET       (3)
#define SENSOREV_LOWER_CRITICAL_LOW_OFFSET        (2)
#define SENSOREV_LOWER_NONCRITICAL_HIGH_OFFSET    (1)
#define SENSOREV_LOWER_NONCRITICAL_LOW_OFFSET     (0)

#define SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK     (0x800)
#define SENSOREV_UPPER_NONRECOVER_LOW_ASSERT_MASK      (0x400)
#define SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK       (0x200)
#define SENSOREV_UPPER_CRITICAL_LOW_ASSERT_MASK        (0x100)
#define SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK    (0x80)
#define SENSOREV_UPPER_NONCRITICAL_LOW_ASSERT_MASK     (0x40)
#define SENSOREV_LOWER_NONRECOVER_HIGH_ASSERT_MASK     (0x20)
#define SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK      (0x10)
#define SENSOREV_LOWER_CRITICAL_HIGH_ASSERT_MASK       (0x08)
#define SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK        (0x04)
#define SENSOREV_LOWER_NONCRITICAL_HIGH_ASSERT_MASK    (0x02)
#define SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK     (0x01)

#define SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK     (0x20)
#define SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK       (0x10)
#define SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK    (0x08)
#define SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK     (0x04)
#define SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK       (0x02)
#define SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK    (0x01)

#define SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK        (0x80)
#define SENSOREV_MSG_CTL_SELECTED_EVENT_ACTION_MASK    (0x30)


typedef struct {
  uint8_t recID_LSB;
  uint8_t recID_MSB;
  uint8_t SDRversion;
  uint8_t rectype;
  uint8_t reclength;
} SDR_entry_hdr_t;

typedef struct {
  SDR_entry_hdr_t hdr;
  uint8_t ownerID;                                                              // 6
  uint8_t ownerLUN;                                                             // 7
  uint8_t sensornum;                                                    // 8
  uint8_t entityID;                                                             // 9
  uint8_t entityinstance;                                               // 10
  uint8_t sensorinit;                                                   // 11
  uint8_t sensorcap;                                                    // 12
  uint8_t sensortype;                                                   // 13
  uint8_t event_reading_type;                                   // 14
  uint8_t assertion_event_mask[2];              // 15-16
  uint8_t deassertion_event_mask[2];            // 17-18
  uint8_t readable_threshold_mask;              // 19
  uint8_t settable_threshold_mask;              // 20
  uint8_t sensor_units_1;                                               // 21
  uint8_t sensor_units_2;                                               // 22
  uint8_t sensor_units_3;                                               // 23
  uint8_t linearization;                                                // 24
  uint8_t M;                                                                    // 25
  uint8_t M_tol;                                                                // 26
  uint8_t B;                                                                    // 27
  uint8_t B_accuracy;                                                   // 28
  uint8_t acc_exp_sensor_dir;                                   // 29
  uint8_t Rexp_Bexp;                                                    // 30
  uint8_t analog_flags;                                                 // 31
  uint8_t nominal_reading;                                              // 32
  uint8_t normal_max;                                                   // 33
  uint8_t normal_min;                                                   // 34
  uint8_t sensor_max_reading;                                   // 35
  uint8_t sensor_min_reading;                                   // 36
  uint8_t upper_nonrecover_thr;                                 // 37
  uint8_t upper_critical_thr;                                   // 38
  uint8_t upper_noncritical_thr;                                // 39
  uint8_t lower_nonrecover_thr;                                 // 40
  uint8_t lower_critical_thr;                                   // 41
  uint8_t lower_noncritical_thr;                                // 42
  uint8_t pos_thr_hysteresis;                                   // 43
  uint8_t neg_thr_hysteresis;                                   // 44
  uint8_t reserved1;                                                    // 45
  uint8_t reserved2;                                                    // 46
  uint8_t OEM;                                                                  // 47
  uint8_t IDtypelen;                                                    // 48
  char IDstring[16];                                                    // 49-64 (0x40 length max)
} SDR_type_01h_t;


typedef struct {
  SDR_entry_hdr_t hdr;
  uint8_t slaveaddr;
  uint8_t chnum;
  uint8_t power_notification_global_init;
  uint8_t device_cap;
  uint8_t reserved[3];
  uint8_t entityID;
  uint8_t entityinstance;
  uint8_t OEM;
  uint8_t IDtypelen;
  char IDstring[16];
} SDR_type_12h_t;


typedef struct {
  SDR_type_12h_t mmc;                                                           // type 12 SDR for MMC
  SDR_type_01h_t sensor[MAX_SENSOR_CNT];
} SDR_table_t;


typedef struct {
  SDR_type_01h_t* pSDR;

  uint8_t event_msg_ctl;

  uint16_t cur_masked_comp;
  uint16_t prev_masked_comp;

  uint8_t comparator_status;              // for IPMI comparator readout for get sensor reading command
  uint8_t readout_value;

  pGetReadoutVal readout_function;
  uint8_t readout_func_arg;

  uint8_t active_context_code;            // context code for when sensor is active

} sensor_data_entry_t;

typedef sensor_data_entry_t sensor_data_tbl_t[MAX_SENSOR_CNT];

typedef enum {assert=0, deassert} event_assertion_type_t;

typedef struct {
  uint8_t SDR_cnt;
  uint8_t sensor_cnt;
  uint8_t sensor_to_SDR_map[MAX_SENSOR_CNT];
  unsigned short SDRreservationID;
} SDR_state_record_t;

extern volatile SDR_state_record_t SDRstate;

extern volatile sensor_data_tbl_t SensorData;

extern volatile SDR_table_t SDRtbl;


void sensor_svc_init(void);

uint8_t* get_SDR_entry_addr(int SDRrecordID);

SDR_type_01h_t* get_sensor_SDR_addr(int SensorNum);

unsigned short get_SDR_next_recID(SDR_entry_hdr_t* pCurSDRHdr);

void rearm_sensor_events(void);

void sensor_service(void);

void sensor_build_hotswap_sensor_message(volatile ipmb_msg_desc_t* preq, volatile uint8_t event_data);

void sensor_data_convert_complete(void);

uint16_t get_thr_sensor_state(SDR_type_01h_t* pSDR, uint16_t prev_comp_state, uint8_t raw_value);

void process_sensor_events(event_assertion_type_t evtype, volatile uint8_t sensornum, uint16_t evtmask);

#endif /* SDR_H_ */
