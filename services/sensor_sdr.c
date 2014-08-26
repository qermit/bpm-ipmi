#include <stdio.h>
#include <string.h>
#include "../include/swevent.h"
#include "../include/cmcpins.h"
#include "../include/gpio.h"
#include "../include/utils.h"

#include "../include/fru.h"

#include "../include/ejecthandle.h"
#include "../include/sensor_sdr.h"

#include "../include/sio_usart.h"

#include "../include/ipmi_i2c_driver.h"

#include "../include/ipmi_cmd_parser.h"
#include "../include/payload_mgr.h"
#include "../include/math.h"
#include "../include/CardDiagnostic.h"
#include "../include/iap_driver.h"



// Mask bits for sensor updates from hardware drivers
// (new hardware drivers should define additional bits in this mask)
#define   SENS_UPDATE_MSK_DIAGNOSTIC            (1 << 0)                    // board diagnostic sensors mask
#define   SENS_UPDATE_MSK_HANDLE                (1 << 1)                    // eject handle

volatile uint32_t sensor_update_mask;

volatile SDR_table_t SDRtbl;
volatile sensor_data_tbl_t SensorData;
volatile SDR_state_record_t SDRstate;
volatile int sensor_rearm_flag;


//void sensor_data_convert_complete(void);
static void eject_handle_pos_change(void);
static void build_default_SDR_image(void);
static void initialize_sensor_table(void);


int GPIO_sensor_read_callback(event eventID, void* arg);

static uint8_t eject_handle_callback(event eventID, void* arg);

uint16_t get_thr_sensor_state(SDR_type_01h_t* pSDR, uint16_t prev_comp_state, uint8_t raw_value);




void rearm_sensor_events(void)
{
	// called at initialization and when Set Event Receiver command is issued
  sensor_rearm_flag = 1; 	
}


void sensor_svc_init(void) {

  uint8_t i1;

  uint8_t curipmbladdr = ipmi_i2c_state.ipmbl_addr;
  uint8_t curentityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;


  build_default_SDR_image();


  // now update all SDR entries with the correct slot-specific information
  SDRtbl.mmc.slaveaddr = curipmbladdr;
  SDRtbl.mmc.entityinstance = curentityinstance;

  for (i1=0; i1<MAX_SENSOR_CNT; i1++) {
    if (SDRtbl.sensor[i1].hdr.reclength == 0)
      continue;         // skip this entry, nothing there
    // update each sensor SDR
    SDRtbl.sensor[i1].ownerID = curipmbladdr;
    SDRtbl.sensor[i1].entityinstance = curentityinstance;
  }

  // clear sensor change mask
  sensor_update_mask = 0;

  // initialize the sensor table
  initialize_sensor_table();

 // initialize ejection handle call back
 register_eject_handle_change_callback(eject_handle_pos_change);
 // register software event call backs
 register_swevent_callback(eject_handle_callback, SENSOR_EJCT_HDL_CHG);


  // initialize SDR state record
  SDRstate.SDR_cnt = 1;       // count MMC SDR entry as first SDR
  SDRstate.sensor_cnt = 0;
  for (i1=0; i1<MAX_SENSOR_CNT; i1++)
  {
    if ((SDRtbl.sensor[i1].hdr.recID_LSB != 0xFF) || (SDRtbl.sensor[i1].hdr.recID_MSB != 0xFF))
      SDRstate.SDR_cnt++;
    if (SensorData[i1].pSDR != NULL)
      SDRstate.sensor_cnt++;
  }

  SDRstate.SDRreservationID = 0;

  // initialize the sensor-to-SDR map for all sensors
  SDRstate.sensor_to_SDR_map[HOTSWAP_SENSOR] 		= SDR_HOTSWAP_SENSOR;

  SDRstate.sensor_to_SDR_map[FPGA_TEMP_SENSOR] 		= SDR_FPGA_TEMP_SENSOR;
  SDRstate.sensor_to_SDR_map[FMC1_TEMP_SENSOR] 		= SDR_FMC1_TEMP_SENSOR;
  SDRstate.sensor_to_SDR_map[FMC2_TEMP_SENSOR] 		= SDR_FMC2_TEMP_SENSOR;
  SDRstate.sensor_to_SDR_map[DC_DC_CONV_TEMP_SENSOR] 	= SDR_DC_DC_CONV_TEMP_SENSOR;
  SDRstate.sensor_to_SDR_map[SDRAM_TEMP_SENSOR] 	= SDR_SDRAM_TEMP_SENSOR;
#ifdef AMC_FMC_Carierr
//  SDRstate.sensor_to_SDR_map[FMC1_12V_VOLT_SENSOR] = SDR_FMC1_12V_VOLT_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC1_12V_CURR_SENSOR] = SDR_FMC1_12V_CURR_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC1_3V3_VOLT_SENSOR] = SDR_FMC1_3V3_VOLT_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC1_3V3_CURR_SENSOR] = SDR_FMC1_3V3_CURR_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC1_ADJ_VOLT_SENSOR] = SDR_FMC1_ADJ_VOLT_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC1_ADJ_CURR_SENSOR] = SDR_FMC1_ADJ_CURR_SENSOR;
//
//  SDRstate.sensor_to_SDR_map[FMC2_12V_VOLT_SENSOR] = SDR_FMC2_12V_VOLT_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC2_12V_CURR_SENSOR] = SDR_FMC2_12V_CURR_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC2_3V3_VOLT_SENSOR] = SDR_FMC2_3V3_VOLT_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC2_3V3_CURR_SENSOR] = SDR_FMC2_3V3_CURR_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC2_ADJ_VOLT_SENSOR] = SDR_FMC2_ADJ_VOLT_SENSOR;
//  SDRstate.sensor_to_SDR_map[FMC2_ADJ_CURR_SENSOR] = SDR_FMC2_ADJ_CURR_SENSOR;
#endif
  rearm_sensor_events();

}


void sensor_service(void)
{
  // inspect sensor update mask fields for sensor groups that have a change at the hardware level
  // ejection handle
  uint8_t i1;

  if (sensor_update_mask & SENS_UPDATE_MSK_HANDLE)
    {
      // have handle change
      Disable_global_interrupt();
      sensor_update_mask &= ~SENS_UPDATE_MSK_HANDLE;          // clear mask bit
      Enable_global_interrupt();
      post_swevent(SENSOR_EJCT_HDL_CHG, NULL);      // post software event
    }

  // sensor update
  if (sensor_update_mask & SENS_UPDATE_MSK_DIAGNOSTIC)
    {
      // have handle change
      Disable_global_interrupt();
      sensor_update_mask &= ~SENS_UPDATE_MSK_DIAGNOSTIC;        // clear mask bit
      Enable_global_interrupt();
      post_swevent(SENSOR_DIAGNOSTIC_UPDATE, NULL);             // post software event
    //post_swevent(SENSOR_GPIO_READ, NULL);                     // piggy-back GPIO poll on ADC readout
    }
  if (sensor_rearm_flag)
    {
      // rearm events for transmission
      sensor_rearm_flag = 0;
      post_swevent(SENSOR_EJCT_HDL_CHG, NULL);      // post software event for handle change event

      for (i1=0; i1<MAX_SENSOR_CNT; i1++)
        SensorData[i1].cur_masked_comp = 0;        // zeroing out the maked comparator state will trigger new events for values above threshold
    }
}


void initialize_sensor_table(void) {

  // initialize sensor data table
  memset((void*) &SensorData, 0, sizeof(sensor_data_tbl_t));
  SensorData[HOTSWAP_SENSOR].pSDR = &SDRtbl.sensor[HOTSWAP_SENSOR];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////Temperature sensor///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  SensorData[FPGA_TEMP_SENSOR].readout_function 	= getSensorData;
  SensorData[FPGA_TEMP_SENSOR].readout_func_arg 	= 0;
  SensorData[FPGA_TEMP_SENSOR].pSDR 			= &SDRtbl.sensor[FPGA_TEMP_SENSOR];
  SensorData[FPGA_TEMP_SENSOR].event_msg_ctl 		= SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
  SensorData[FPGA_TEMP_SENSOR].active_context_code 	= SENSORACTV_PAYLOADPWRON;

  SensorData[FMC1_TEMP_SENSOR].readout_function 	= getSensorData;
  SensorData[FMC1_TEMP_SENSOR].readout_func_arg 	= 0x01;
  SensorData[FMC1_TEMP_SENSOR].pSDR 			= &SDRtbl.sensor[FMC1_TEMP_SENSOR];
  SensorData[FMC1_TEMP_SENSOR].event_msg_ctl 		= SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
  SensorData[FMC1_TEMP_SENSOR].active_context_code 	= SENSORACTV_PAYLOADPWRON;

  SensorData[FMC2_TEMP_SENSOR].readout_function 	= getSensorData;
  SensorData[FMC2_TEMP_SENSOR].readout_func_arg 	= 0x04;//0x02;
  SensorData[FMC2_TEMP_SENSOR].pSDR 			= &SDRtbl.sensor[FMC2_TEMP_SENSOR];
  SensorData[FMC2_TEMP_SENSOR].event_msg_ctl 		= SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
  SensorData[FMC2_TEMP_SENSOR].active_context_code 	= SENSORACTV_PAYLOADPWRON;

  SensorData[DC_DC_CONV_TEMP_SENSOR].readout_function 	 = getSensorData;
  SensorData[DC_DC_CONV_TEMP_SENSOR].readout_func_arg 	 = 0x02;//0x03;
  SensorData[DC_DC_CONV_TEMP_SENSOR].pSDR 		 = &SDRtbl.sensor[DC_DC_CONV_TEMP_SENSOR];
  SensorData[DC_DC_CONV_TEMP_SENSOR].event_msg_ctl 	 = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
  SensorData[DC_DC_CONV_TEMP_SENSOR].active_context_code = SENSORACTV_PAYLOADPWRON;

  SensorData[SDRAM_TEMP_SENSOR].readout_function 	= getSensorData;
  SensorData[SDRAM_TEMP_SENSOR].readout_func_arg 	= 0x03;//0x04;
  SensorData[SDRAM_TEMP_SENSOR].pSDR 			= &SDRtbl.sensor[SDRAM_TEMP_SENSOR];
  SensorData[SDRAM_TEMP_SENSOR].event_msg_ctl 		= SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
  SensorData[SDRAM_TEMP_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////Current sensor/////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SensorData[FMC1_12V_CURR_SENSOR].readout_function        = getSensorData;
//  SensorData[FMC1_12V_CURR_SENSOR].readout_func_arg        = 0x05;
//  SensorData[FMC1_12V_CURR_SENSOR].pSDR                    = &SDRtbl.sensor[FMC1_12V_CURR_SENSOR];
//  SensorData[FMC1_12V_CURR_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[FMC1_12V_CURR_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC1_3V3_CURR_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC1_3V3_CURR_SENSOR].readout_func_arg        = 0x06;
//  SensorData[SDR_FMC1_3V3_CURR_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC1_3V3_CURR_SENSOR];
//  SensorData[SDR_FMC1_3V3_CURR_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC1_3V3_CURR_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC1_ADJ_CURR_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC1_ADJ_CURR_SENSOR].readout_func_arg        = 0x07;
//  SensorData[SDR_FMC1_ADJ_CURR_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC1_ADJ_CURR_SENSOR];
//  SensorData[SDR_FMC1_ADJ_CURR_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC1_ADJ_CURR_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[FMC2_12V_CURR_SENSOR].readout_function        = getSensorData;
//  SensorData[FMC2_12V_CURR_SENSOR].readout_func_arg        = 0x08;
//  SensorData[FMC2_12V_CURR_SENSOR].pSDR                    = &SDRtbl.sensor[FMC2_12V_CURR_SENSOR];
//  SensorData[FMC2_12V_CURR_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[FMC2_12V_CURR_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC2_3V3_CURR_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC2_3V3_CURR_SENSOR].readout_func_arg        = 0x09;
//  SensorData[SDR_FMC2_3V3_CURR_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC2_3V3_CURR_SENSOR];
//  SensorData[SDR_FMC2_3V3_CURR_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC2_3V3_CURR_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC2_ADJ_CURR_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC2_ADJ_CURR_SENSOR].readout_func_arg        = 0x0a;
//  SensorData[SDR_FMC2_ADJ_CURR_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC2_ADJ_CURR_SENSOR];
//  SensorData[SDR_FMC2_ADJ_CURR_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC2_ADJ_CURR_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////Voltage sensor/////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SensorData[FMC1_12V_VOLT_SENSOR].readout_function        = getSensorData;
//  SensorData[FMC1_12V_VOLT_SENSOR].readout_func_arg        = 0x0b;
//  SensorData[FMC1_12V_VOLT_SENSOR].pSDR                    = &SDRtbl.sensor[FMC1_12V_VOLT_SENSOR];
//  SensorData[FMC1_12V_VOLT_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[FMC1_12V_VOLT_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC1_3V3_VOLT_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC1_3V3_VOLT_SENSOR].readout_func_arg        = 0x0c;
//  SensorData[SDR_FMC1_3V3_VOLT_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC1_3V3_VOLT_SENSOR];
//  SensorData[SDR_FMC1_3V3_VOLT_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC1_3V3_VOLT_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC1_ADJ_VOLT_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC1_ADJ_VOLT_SENSOR].readout_func_arg        = 0x0d;
//  SensorData[SDR_FMC1_ADJ_VOLT_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC1_ADJ_VOLT_SENSOR];
//  SensorData[SDR_FMC1_ADJ_VOLT_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC1_ADJ_VOLT_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[FMC2_12V_VOLT_SENSOR].readout_function        = getSensorData;
//  SensorData[FMC2_12V_VOLT_SENSOR].readout_func_arg        = 0x0e;
//  SensorData[FMC2_12V_VOLT_SENSOR].pSDR                    = &SDRtbl.sensor[FMC2_12V_VOLT_SENSOR];
//  SensorData[FMC2_12V_VOLT_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[FMC2_12V_VOLT_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC2_3V3_VOLT_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC2_3V3_VOLT_SENSOR].readout_func_arg        = 0x0f;
//  SensorData[SDR_FMC2_3V3_VOLT_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC2_3V3_VOLT_SENSOR];
//  SensorData[SDR_FMC2_3V3_VOLT_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC2_3V3_VOLT_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;
//
//  SensorData[SDR_FMC2_ADJ_VOLT_SENSOR].readout_function        = getSensorData;
//  SensorData[SDR_FMC2_ADJ_VOLT_SENSOR].readout_func_arg        = 0x10;
//  SensorData[SDR_FMC2_ADJ_VOLT_SENSOR].pSDR                    = &SDRtbl.sensor[SDR_FMC2_ADJ_VOLT_SENSOR];
//  SensorData[SDR_FMC2_ADJ_VOLT_SENSOR].event_msg_ctl           = SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
//  SensorData[SDR_FMC2_ADJ_VOLT_SENSOR].active_context_code     = SENSORACTV_PAYLOADPWRON;

//  SensorData[GPIO_SENSOR].readout_function = get_gpio_readout_value_readout_value;
//  SensorData[GPIO_SENSOR].pSDR = &SDRtbl.sensor[GPIO_SENSOR];

}


void build_default_SDR_image(void)
{
  char buff[16];
  volatile uint32_t id;

  int i1;
  volatile SDR_type_01h_t* pSDR;
  unsigned short event_mask;

  iap_get_id();

  id = uC_id[2]^uC_id[1];

  // builds the default SDR image in RAM
  memset((void*) &SDRtbl, 0, sizeof(SDR_table_t));

  // preset the LSB and USB of all recID's to 0xffff to indicate that they are unused
  // Marking them in this way will help the SDR readout commands correctly navigate the SDR
  // table during readout by the Shelf Manager
  for (i1=0; i1<MAX_SENSOR_CNT; i1++)
  {
    SDRtbl.sensor[i1].hdr.recID_LSB = 0xff;
    SDRtbl.sensor[i1].hdr.recID_MSB = 0xff;
  }

   // MMC SDR
  SDRtbl.mmc.hdr.recID_LSB = /*SDR_MMC*/    0;    // MMC SDR is first one
  SDRtbl.mmc.hdr.recID_MSB = 0;
  SDRtbl.mmc.hdr.SDRversion = SDR_VERSION;
  SDRtbl.mmc.hdr.rectype = 0x12;
  SDRtbl.mmc.hdr.reclength = sizeof(SDR_type_12h_t)-sizeof(SDR_entry_hdr_t);
  SDRtbl.mmc.slaveaddr = ipmi_i2c_state.ipmbl_addr;
  SDRtbl.mmc.device_cap = 0x3b;
  SDRtbl.mmc.entityID = ENTITY_ID;
  SDRtbl.mmc.entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  SDRtbl.mmc.IDtypelen = 0xc0|13;


  sprintf(&buff[0], "AFC_AMC_%d", ipmi_i2c_state.slotid );

#ifndef AMC_CPU_COM_Express
 strcpy(&SDRtbl.mmc.IDstring[0], &buff[0] );
#else
 strcpy(&SDRtbl.mmc.IDstring[0], "COM_Express");
#endif

  // Hotswap Sensor
  pSDR = &SDRtbl.sensor[HOTSWAP_SENSOR];
  pSDR->hdr.recID_LSB = HOTSWAP_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = HOTSWAP_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x67;
  pSDR->sensorcap = 0x42;
  pSDR->sensortype = 0xf2;
  pSDR->event_reading_type = 0x6f;
  pSDR->assertion_event_mask[LOWBYTE] = 0x1f;
  pSDR->deassertion_event_mask[LOWBYTE] = 0x1f;
  pSDR->IDtypelen = 0xc0|7;
  strcpy(pSDR->IDstring, "Hotswap");


  pSDR = &SDRtbl.sensor[FPGA_TEMP_SENSOR];
  pSDR->hdr.recID_LSB = FPGA_TEMP_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = FPGA_TEMP_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x45;  	//0x7f;
  pSDR->sensorcap = 0x7F; 	//0x58;
  pSDR->sensortype = 0x01;
  pSDR->event_reading_type = 0x01;
  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->assertion_event_mask[HIGHBYTE] = (unsigned char) (event_mask >> 8);
  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->sensor_units_1 = 0x21;
  pSDR->sensor_units_2 = 0x01; // degrees C
  pSDR->sensor_units_3 = 0x00;
  pSDR->linearization = 0x00;
  pSDR->M = 0x01;//2;
  pSDR->M_tol = 0x00;
  pSDR->B = 0;//10;
  pSDR->B_accuracy = 0x00;
  pSDR->acc_exp_sensor_dir = 0x00;
  pSDR->Rexp_Bexp = 0x00;	//0xf1;
  pSDR->analog_flags = 0x06;
  pSDR->nominal_reading = 0x00;
  pSDR->normal_max = 0xf0;
  pSDR->normal_min = 0x00;
  pSDR->sensor_max_reading = 0xff;
  pSDR->sensor_min_reading = 0x00;
  pSDR->upper_nonrecover_thr = 0x80;
  pSDR->upper_critical_thr = 0x46;
  pSDR->upper_noncritical_thr = 0x32;
  pSDR->lower_critical_thr = 0x02;
  pSDR->lower_noncritical_thr = 0x05;
  pSDR->lower_nonrecover_thr = 0x00;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->reserved1 = 0x00;
  pSDR->reserved2 = 0x00;
  pSDR->IDtypelen = 0xc0|9;
  strcpy(pSDR->IDstring, "FPGA-Temp");


  // On-Board Hot Spot Temperature Sensor
  pSDR = &SDRtbl.sensor[FMC1_TEMP_SENSOR];
  pSDR->hdr.recID_LSB = FMC1_TEMP_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = FMC1_TEMP_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x51; //0x45; // 0x7f;
  pSDR->sensorcap  = 0x57; //0x7F; 	//0x58;
  pSDR->sensortype = 0x01;
  pSDR->event_reading_type = 0x01;
  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->assertion_event_mask[HIGHBYTE] = (unsigned char) (event_mask >> 8);
  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->sensor_units_1 = 0x21;
  pSDR->sensor_units_2 = 0x01;
  pSDR->sensor_units_3 = 0x00;
  pSDR->linearization = 0x00;
  pSDR->M = 0x01;
  pSDR->M_tol = 0x00;
  pSDR->B = 0x00;
  pSDR->B_accuracy = 0x00;
  pSDR->acc_exp_sensor_dir = 0x00;
  pSDR->Rexp_Bexp = 0x00;	//0xe1;;
  pSDR->analog_flags = 0x06;
  pSDR->nominal_reading = 0x00;
  pSDR->normal_max = 0xf0;
  pSDR->normal_min = 0x00;
  pSDR->sensor_max_reading = 0xff;
  pSDR->sensor_min_reading = 0x00;
  pSDR->upper_nonrecover_thr = 0x80;
  pSDR->upper_critical_thr = 0x46;
  pSDR->upper_noncritical_thr = 0x32;
  pSDR->lower_critical_thr = 0x02;
  pSDR->lower_noncritical_thr = 0x05;
  pSDR->lower_nonrecover_thr = 0x00;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->reserved1 = 0x00;
  pSDR->reserved2 = 0x00;
  pSDR->IDtypelen = 0xc0|13;
  strcpy(pSDR->IDstring, "FMC1-Temp");

  // On-Board Hot Spot Temperature Sensor
  pSDR = &SDRtbl.sensor[FMC2_TEMP_SENSOR];
  pSDR->hdr.recID_LSB = FMC2_TEMP_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = FMC2_TEMP_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x51; //0x45; // 0x7f;
  pSDR->sensorcap  = 0x57; //0x7F; //0x58;
  pSDR->sensortype = 0x01;
  pSDR->event_reading_type = 0x01;
  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->assertion_event_mask[HIGHBYTE] = (unsigned char) (event_mask >> 8);
  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->sensor_units_1 = 0x21;
  pSDR->sensor_units_2 = 0x01;
  pSDR->sensor_units_3 = 0x00;
  pSDR->linearization = 0x00;
  pSDR->M = 0x01;
  pSDR->M_tol = 0x00;
  pSDR->B = 0x00;
  pSDR->B_accuracy = 0x00;
  pSDR->acc_exp_sensor_dir = 0x00;
  pSDR->Rexp_Bexp = 0x00;	//0xe1;;
  pSDR->analog_flags = 0x06;
  pSDR->nominal_reading = 0x00;
  pSDR->normal_max = 0xf0;
  pSDR->normal_min = 0x00;
  pSDR->sensor_max_reading = 0xff;
  pSDR->sensor_min_reading = 0x00;
  pSDR->upper_nonrecover_thr = 0x80;
  pSDR->upper_critical_thr = 0x46;
  pSDR->upper_noncritical_thr = 0x32;
  pSDR->lower_critical_thr = 0x02;
  pSDR->lower_noncritical_thr = 0x05;
  pSDR->lower_nonrecover_thr = 0x00;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->reserved1 = 0x00;
  pSDR->reserved2 = 0x00;
  pSDR->IDtypelen = 0xc0|13;
  strcpy(pSDR->IDstring, "FMC2-Temp");

  // On-Board Hot Spot Temperature Sensor
  pSDR = &SDRtbl.sensor[DC_DC_CONV_TEMP_SENSOR];
  pSDR->hdr.recID_LSB = DC_DC_CONV_TEMP_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = DC_DC_CONV_TEMP_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x51; //0x45; // 0x7f;
  pSDR->sensorcap  = 0x57; //0x7F; 	//0x58;
  pSDR->sensortype = 0x01;
  pSDR->event_reading_type = 0x01;
  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->assertion_event_mask[HIGHBYTE] = (unsigned char) (event_mask >> 8);
  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->sensor_units_1 = 0x21;
  pSDR->sensor_units_2 = 0x01;
  pSDR->sensor_units_3 = 0x00;
  pSDR->linearization = 0x00;
  pSDR->M = 0x01;
  pSDR->M_tol = 0x00;
  pSDR->B = 0x00;
  pSDR->B_accuracy = 0x00;
  pSDR->acc_exp_sensor_dir = 0x00;
  pSDR->Rexp_Bexp = 0x00;	//0xe1;;
  pSDR->analog_flags = 0x06;
  pSDR->nominal_reading = 0x00;
  pSDR->normal_max = 0xf0;
  pSDR->normal_min = 0x00;
  pSDR->sensor_max_reading = 0xff;
  pSDR->sensor_min_reading = 0x00;
  pSDR->upper_nonrecover_thr = 0x80;
  pSDR->upper_critical_thr = 0x46;
  pSDR->upper_noncritical_thr = 0x32;
  pSDR->lower_critical_thr = 0x02;
  pSDR->lower_noncritical_thr = 0x05;
  pSDR->lower_nonrecover_thr = 0x00;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->reserved1 = 0x00;
  pSDR->reserved2 = 0x00;
  pSDR->IDtypelen = 0xc0|15;
  strcpy(pSDR->IDstring, "DC/DC_Conv-Temp");

  // On-Board Hot Spot Temperature Sensor
  pSDR = &SDRtbl.sensor[SDRAM_TEMP_SENSOR];
  pSDR->hdr.recID_LSB = SDRAM_TEMP_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = SDRAM_TEMP_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x51; //0x45; // 0x7f;
  pSDR->sensorcap  = 0x57; //0x7F; 	//0x58;
  pSDR->sensortype = 0x01;
  pSDR->event_reading_type = 0x01;
  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->assertion_event_mask[HIGHBYTE] = (unsigned char) (event_mask >> 8);
  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->sensor_units_1 = 0x21;
  pSDR->sensor_units_2 = 0x01;
  pSDR->sensor_units_3 = 0x00;
  pSDR->linearization = 0x00;
  pSDR->M = 0x01;
  pSDR->M_tol = 0x00;
  pSDR->B = 0x00;
  pSDR->B_accuracy = 0x00;
  pSDR->acc_exp_sensor_dir = 0x00;
  pSDR->Rexp_Bexp = 0x00;	//0xe1;;
  pSDR->analog_flags = 0x06;
  pSDR->nominal_reading = 0x00;
  pSDR->normal_max = 0xf0;
  pSDR->normal_min = 0x00;
  pSDR->sensor_max_reading = 0xff;
  pSDR->sensor_min_reading = 0x00;
  pSDR->upper_nonrecover_thr = 0x80;
  pSDR->upper_critical_thr = 0x46;
  pSDR->upper_noncritical_thr = 0x32;
  pSDR->lower_critical_thr = 0x02;
  pSDR->lower_noncritical_thr = 0x05;
  pSDR->lower_nonrecover_thr = 0x00;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->reserved1 = 0x00;
  pSDR->reserved2 = 0x00;
  pSDR->IDtypelen = 0xc0|13;
  strcpy(pSDR->IDstring, "SDRAM-Temp");


//  pSDR = &SDRtbl.sensor[PAYLOAD_12V_SENSOR];
//  pSDR->hdr.recID_LSB = PAYLOAD_12V_SENSOR+1;
//  pSDR->hdr.recID_MSB = 0;
//  pSDR->hdr.SDRversion = SDR_VERSION;
//  pSDR->hdr.rectype = 0x01;
//  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
//  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
//  pSDR->sensornum = PAYLOAD_12V_SENSOR;
//  pSDR->entityID = ENTITY_ID;
//  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
//  pSDR->sensorinit = 0x7f;
//  pSDR->sensorcap = 0x58;
//  pSDR->sensortype = 0x02;                  // type 2 = voltage
//  pSDR->event_reading_type = 0x01;					// type 1 = thresholds
//  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK |
//    SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK | SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK |
//    SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
//  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
//  pSDR->assertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
//  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
//  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
//  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK |
//    SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK | SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK;
//  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK |
//    SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK | SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK;
//  pSDR->sensor_units_2 = 0x4;							// 22 volts
//  //pSDR->linearization;									// 24
//  pSDR->M = 56;											// 25
//  pSDR->M_tol = 0;											// 26
//  //pSDR->B = 0;												// 27
//  //pSDR->B_accuracy = 0;										// 28
//  //pSDR->acc_exp_sensor_dir = 0;								// 29
//  pSDR->Rexp_Bexp = 0xd0;										// 30
//  pSDR->analog_flags = 7;								// 31 nominal reading, min & max specified
//  pSDR->nominal_reading = 216;							// 32 nominal = 12.000V
//  pSDR->normal_max = 255;								// 33 max = 14.28V
//  pSDR->normal_min = 0;									// 34 min = 0V
//  pSDR->sensor_max_reading = 255;								// 35
//  pSDR->sensor_min_reading = 0x00;
//  pSDR->upper_nonrecover_thr = 250;
//  pSDR->upper_critical_thr = 236;								// 38
//  pSDR->upper_noncritical_thr = 225;							// 39
//  pSDR->lower_critical_thr = 193;								// 41
//  pSDR->lower_noncritical_thr = 205;							// 42
//  pSDR->lower_nonrecover_thr = 178;
//  pSDR->pos_thr_hysteresis = 2;
//  pSDR->neg_thr_hysteresis = 2;
//  pSDR->IDtypelen = 0xc0|5;
//  strcpy(pSDR->IDstring, "+12V");
/*
  // Backend 3.3V
  pSDR = &SDRtbl.sensor[BACKEND_3p3V_SENSOR];
  pSDR->hdr.recID_LSB = BACKEND_3p3V_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = BACKEND_3p3V_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x7f;
  pSDR->sensorcap = 0x58;
  pSDR->sensortype = 0x02;							     // type 2 = voltage
  pSDR->event_reading_type = 0x01;					 // type 1 = thresholds
  event_mask = SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK |
    SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK | SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK |
    SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  pSDR->assertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->assertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->deassertion_event_mask[LOWBYTE] = (unsigned char) (event_mask & 0xff);
  pSDR->deassertion_event_mask[HIGHBYTE] = 0x70 | (unsigned char) (event_mask >> 8);
  pSDR->readable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK |
    SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK | SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->settable_threshold_mask = SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK | SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK |
    SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK | SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK | SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK;
  pSDR->sensor_units_2 = 0x4;							// 22 volts
  //pSDR->linearization;									// 24
  pSDR->M = 144;											// 25
  pSDR->M_tol = 0;											// 26
  //pSDR->B = 0;												// 27
  //pSDR->B_accuracy = 0;										// 28
  //pSDR->acc_exp_sensor_dir = 0;								// 29
  pSDR->Rexp_Bexp = 0xc0;								// 30 R = -1
  pSDR->analog_flags = 7;									// 31
  pSDR->nominal_reading = 206;								// 32
  pSDR->normal_max = 255;										// 33
  pSDR->normal_min = 0;										// 34
  pSDR->sensor_max_reading = 255;								// 35
  pSDR->upper_nonrecover_thr = 250;
  pSDR->upper_critical_thr = 245;								// 38
  pSDR->upper_noncritical_thr = 240;							// 39
  pSDR->lower_critical_thr = 194;								// 41
  pSDR->lower_noncritical_thr = 200;							// 42
  pSDR->lower_nonrecover_thr = 185;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->IDtypelen = 0xc0|11;
  strcpy(pSDR->IDstring, "+3.3V BkEnd");

  // ADC Channel 1 (0.75X attenuator)
  pSDR = &SDRtbl.sensor[ADC1_SENSOR];
  pSDR->hdr.recID_LSB = ADC1_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = ADC1_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x7f;
  pSDR->sensorcap = 0x43;                   // no hysteresis, thresholds, or events (default condition, can be updated to 0x7e via IPMI cmds)
  pSDR->sensortype = 0x02;                  // type 2 = voltage
  pSDR->event_reading_type = 0x01;          // type 1 = thresholds
  pSDR->assertion_event_mask[LOWBYTE] = 0;
  pSDR->assertion_event_mask[HIGHBYTE] = 0;
  pSDR->deassertion_event_mask[LOWBYTE] = 0;
  pSDR->deassertion_event_mask[HIGHBYTE] = 0;
  pSDR->readable_threshold_mask = 0;
  pSDR->settable_threshold_mask = 0;
  pSDR->sensor_units_2 = 0x4;             // 22 volts
  //pSDR->linearization;                  // 24
  pSDR->M = 157;                      // 25
  pSDR->M_tol = 0;                      // 26
  //pSDR->B = 0;                        // 27
  //pSDR->B_accuracy = 0;                   // 28
  //pSDR->acc_exp_sensor_dir = 0;               // 29
  pSDR->Rexp_Bexp = 0xc0;               // 30 R = -1
  pSDR->analog_flags = 6;                 // 31
  pSDR->nominal_reading = 0;                // 32
  pSDR->normal_max = 255;                   // 33
  pSDR->normal_min = 0;                   // 34
  pSDR->sensor_max_reading = 255;                // 35
  pSDR->upper_nonrecover_thr = 0;
  pSDR->upper_critical_thr = 0;               // 38
  pSDR->upper_noncritical_thr = 0;              // 39
  pSDR->lower_critical_thr = 0;               // 41
  pSDR->lower_noncritical_thr = 0;              // 42
  pSDR->lower_nonrecover_thr = 0;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  //pSDR->IDtypelen = 0xc0|4;
  //strcpy(pSDR->IDstring, "ADC1");
  // For AMC13 use, this sensor is referred to as "T2 1.2V"
  pSDR->IDtypelen = 0xc0|7;
  strcpy(pSDR->IDstring, "T2 1.2V");

  // ADC Channel 3 (1X attenuator)
  pSDR = &SDRtbl.sensor[ADC3_SENSOR];
  pSDR->hdr.recID_LSB = ADC3_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = ADC3_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x7f;
  pSDR->sensorcap = 0x43;                   // no hysteresis, thresholds, or events (default condition, can be updated to 0x7e via IPMI cmds)
  pSDR->sensortype = 0x02;                  // type 2 = voltage
  pSDR->event_reading_type = 0x01;          // type 1 = thresholds
  pSDR->assertion_event_mask[LOWBYTE] = 0;
  pSDR->assertion_event_mask[HIGHBYTE] = 0;
  pSDR->deassertion_event_mask[LOWBYTE] = 0;
  pSDR->deassertion_event_mask[HIGHBYTE] = 0;
  pSDR->readable_threshold_mask = 0;
  pSDR->settable_threshold_mask = 0;
  pSDR->sensor_units_2 = 0x4;             // 22 volts
  //pSDR->linearization;                  // 24
  pSDR->M = 118;                      // 25
  pSDR->M_tol = 0;                      // 26
  //pSDR->B = 0;                        // 27
  //pSDR->B_accuracy = 0;                   // 28
  //pSDR->acc_exp_sensor_dir = 0;               // 29
  pSDR->Rexp_Bexp = 0xc0;               // 30 R = -1
  pSDR->analog_flags = 6;                 // 31
  pSDR->nominal_reading = 0;                // 32
  pSDR->normal_max = 255;                   // 33
  pSDR->normal_min = 0;                   // 34
  pSDR->sensor_max_reading = 255;                // 35
  pSDR->upper_nonrecover_thr = 0;
  pSDR->upper_critical_thr = 0;               // 38
  pSDR->upper_noncritical_thr = 0;              // 39
  pSDR->lower_critical_thr = 0;               // 41
  pSDR->lower_noncritical_thr = 0;              // 42
  pSDR->lower_nonrecover_thr = 0;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->IDtypelen = 0xc0|4;
  strcpy(pSDR->IDstring, "ADC3");
*/

/*
  // ADC Channel 4 (1X attenuator)
  pSDR = &SDRtbl.sensor[ADC4_SENSOR];
  pSDR->hdr.recID_LSB = ADC4_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = ADC4_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x7f;
  pSDR->sensorcap = 0x43;                   // no hysteresis, thresholds, or events (default condition, can be updated to 0x7e via IPMI cmds)
  pSDR->sensortype = 0x02;                  // type 2 = voltage
  pSDR->event_reading_type = 0x01;          // type 1 = thresholds
  pSDR->assertion_event_mask[LOWBYTE] = 0;
  pSDR->assertion_event_mask[HIGHBYTE] = 0;
  pSDR->deassertion_event_mask[LOWBYTE] = 0;
  pSDR->deassertion_event_mask[HIGHBYTE] = 0;
  pSDR->readable_threshold_mask = 0;
  pSDR->settable_threshold_mask = 0;
  pSDR->sensor_units_2 = 0x4;             // 22 volts
  //pSDR->linearization;                  // 24
  pSDR->M = 118;                      // 25
  pSDR->M_tol = 0;                      // 26
  //pSDR->B = 0;                        // 27
  //pSDR->B_accuracy = 0;                   // 28
  //pSDR->acc_exp_sensor_dir = 0;               // 29
  pSDR->Rexp_Bexp = 0xc0;               // 30 R = -1
  pSDR->analog_flags = 6;                 // 31
  pSDR->nominal_reading = 0;                // 32
  pSDR->normal_max = 255;                   // 33
  pSDR->normal_min = 0;                   // 34
  pSDR->sensor_max_reading = 255;                // 35
  pSDR->upper_nonrecover_thr = 0;
  pSDR->upper_critical_thr = 0;               // 38
  pSDR->upper_noncritical_thr = 0;              // 39
  pSDR->lower_critical_thr = 0;               // 41
  pSDR->lower_noncritical_thr = 0;              // 42
  pSDR->lower_nonrecover_thr = 0;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->IDtypelen = 0xc0|4;
  strcpy(pSDR->IDstring, "ADC4");
*/

/*
  // ADC Channel 5 (1X attenuator)
  pSDR = &SDRtbl.sensor[ADC5_SENSOR];
  pSDR->hdr.recID_LSB = ADC5_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = ADC5_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x7f;
  pSDR->sensorcap = 0x43;                   // no hysteresis, thresholds, or events (default condition, can be updated to 0x7e via IPMI cmds)
  pSDR->sensortype = 0x02;                  // type 2 = voltage
  pSDR->event_reading_type = 0x01;          // type 1 = thresholds
  pSDR->assertion_event_mask[LOWBYTE] = 0;
  pSDR->assertion_event_mask[HIGHBYTE] = 0;
  pSDR->deassertion_event_mask[LOWBYTE] = 0;
  pSDR->deassertion_event_mask[HIGHBYTE] = 0;
  pSDR->readable_threshold_mask = 0;
  pSDR->settable_threshold_mask = 0;
  pSDR->sensor_units_2 = 0x4;             // 22 volts
  //pSDR->linearization;                  // 24
  pSDR->M = 118;                      // 25
  pSDR->M_tol = 0;                      // 26
  //pSDR->B = 0;                        // 27
  //pSDR->B_accuracy = 0;                   // 28
  //pSDR->acc_exp_sensor_dir = 0;               // 29
  pSDR->Rexp_Bexp = 0xc0;               // 30 R = -1
  pSDR->analog_flags = 6;                 // 31
  pSDR->nominal_reading = 0;                // 32
  pSDR->normal_max = 255;                   // 33
  pSDR->normal_min = 0;                   // 34
  pSDR->sensor_max_reading = 255;                // 35
  pSDR->upper_nonrecover_thr = 0;
  pSDR->upper_critical_thr = 0;               // 38
  pSDR->upper_noncritical_thr = 0;              // 39
  pSDR->lower_critical_thr = 0;               // 41
  pSDR->lower_noncritical_thr = 0;              // 42
  pSDR->lower_nonrecover_thr = 0;
  pSDR->pos_thr_hysteresis = 2;
  pSDR->neg_thr_hysteresis = 2;
  pSDR->IDtypelen = 0xc0|4;
  strcpy(pSDR->IDstring, "ADC5");
*/

/*  // GPIO (digital, no events)
  pSDR = &SDRtbl.sensor[GPIO_SENSOR];
  pSDR->hdr.recID_LSB = GPIO_SENSOR+1;
  pSDR->hdr.recID_MSB = 0;
  pSDR->hdr.SDRversion = SDR_VERSION;
  pSDR->hdr.rectype = 0x01;
  pSDR->hdr.reclength = sizeof(SDR_type_01h_t)-sizeof(SDR_entry_hdr_t);
  pSDR->ownerID = ipmi_i2c_state.ipmbl_addr;
  pSDR->sensornum = GPIO_SENSOR;
  pSDR->entityID = ENTITY_ID;
  pSDR->entityinstance = ENTITY_INSTANCE_BASE + ipmi_i2c_state.slotid;
  pSDR->sensorinit = 0x7f;
  pSDR->sensorcap = 0x43;                   // no hysteresis, thresholds, or events (default condition, can be updated to 0x7e via IPMI cmds)
  pSDR->sensortype = 0xc0;                  // OEM-defined (by this code)
  pSDR->event_reading_type = 0x70;          // OEM-defined (by this code)
  pSDR->IDtypelen = 0xc0|8;
  strcpy(pSDR->IDstring, "GPIO 7:0");
*/
}


void sensor_data_convert_complete(void)
{
  //new sensors reading is available
  sensor_update_mask |= SENS_UPDATE_MSK_DIAGNOSTIC;
}


static void eject_handle_pos_change(void)
{
  sensor_update_mask |= SENS_UPDATE_MSK_HANDLE;
}


unsigned char* get_SDR_entry_addr(int SDRrecordID)
{
  // returns the address of the specified SDR record ID as a pointer to unsigned chars.
  // returns NULL if the record is out of range or not defined
  int i1;
  int lclrecID;

  if (SDRrecordID >= 0xffff)
    return NULL;            // out of valid range, no match

  // check mmc record
  lclrecID = (SDRtbl.mmc.hdr.recID_MSB << 8) + SDRtbl.mmc.hdr.recID_MSB;
  if (lclrecID == SDRrecordID)
    return ( (unsigned char*) &SDRtbl.mmc );

  // check sensor SDRs
  for (i1=0; i1 < MAX_SENSOR_CNT; i1++)
    {
      lclrecID = (SDRtbl.sensor[i1].hdr.recID_MSB << 8) + SDRtbl.sensor[i1].hdr.recID_LSB;
      if (lclrecID == SDRrecordID)
        return ( (unsigned char*) &SDRtbl.sensor[i1] );
    }

  return NULL;
}


SDR_type_01h_t* get_sensor_SDR_addr(int SensorNum)
{
  // returns a pointer to the sensor SDR for a given sensor number.
  // returns NULL if no sensor is found
  if ((SensorNum < 0) || (SensorNum >=MAX_SENSOR_CNT))
    return NULL;
  return ( SensorData[SensorNum].pSDR );
}

unsigned short get_SDR_next_recID(SDR_entry_hdr_t* pCurSDRHdr)
{
  SDR_type_01h_t* pT1SDR;
  unsigned short recID;

  // check if this is header to last SDR in table if it is, then there are no more by definition
  if (pCurSDRHdr == &SDRtbl.sensor[MAX_SENSOR_CNT-1].hdr)
    return END_OF_SDR_LIST;

  // find pointer to header after this one, which will be a sensor type 1 SDR
  if (pCurSDRHdr == &SDRtbl.mmc.hdr)
    pT1SDR = &(SDRtbl.sensor[0]);
  else
    {
    pT1SDR = (SDR_type_01h_t*) pCurSDRHdr;
    pT1SDR++;
    }

  // traverse table looking for valid recID.  Stop and return its value when we find it
  while (pT1SDR < &SDRtbl.sensor[MAX_SENSOR_CNT-1])
    {
    recID = (pT1SDR->hdr.recID_MSB << 8) + pT1SDR->hdr.recID_LSB;

    if (recID < END_OF_SDR_LIST)
      return ( recID );

    pT1SDR++;
    }

  return ( END_OF_SDR_LIST );         // none found
}


//float SDR_readout_to_display_val(const SDR_type_01h_t* pSDR, unsigned char readoutval) {
//  float retval, Mf, Rf, Bf;
//  long Mi, Bi, Bexp, Rexp;
//
//  Mi = pSDR->M | (((pSDR->M_tol) & 0xc0) << 2);
//  if (Mi & 0x200)
//    Mi |= 0xfffffc00;     // extend sign bit
//  Bi = pSDR->B | (((pSDR->B_accuracy) & 0xc0) << 2);
//  if (Bi & 0x200)
//    Bi |= 0xfffffc00;     // extend sign bit
//  Mf = (float) Mi;
//  Bf = (float) Bi;
//  Bexp = pSDR->Rexp_Bexp & 0xf;
//  if (Bexp & 0x8)
//    Bexp |= 0xfffffff0;
//  Rexp = pSDR->Rexp_Bexp >> 4;
//  if (Rexp & 0x8)
//    Rexp |= 0xfffffff0;     // extend sign bit
//  Rf = pow(10., ((float) Rexp));
//  Bf *= pow(10., (float) Bexp);
//
//  retval = (Mf* ((float) readoutval) + Bf) * Rf;
//  return retval;
//}
//
//
//float readout_to_SDR_signal(const SDR_type_01h_t* pSDR, unsigned char readoutval) {
//  // Uses params in SDR to convert an 8-bit readout value to a floating point signal value
//  float retval, Mf, Rf, Bf;
//  int Mi, Bi, Bexp, Rexp;
//
//  Mi = pSDR->M | (((pSDR->M_tol) & 0xc0) << 2);
//  if (Mi & 0x200)
//    Mi |= 0xfffffc00;			// extend sign bit
//  Bi = pSDR->B | (((pSDR->B_accuracy) & 0xc0) << 2);
//  if (Bi & 0x200)
//    Bi |= 0xfffffc00;			// extend sign bit
//  Mf = (float) Mi;
//  Bf = (float) Bi;
//  Bexp = pSDR->Rexp_Bexp & 0xf;
//  if (Bexp & 0x8)
//    Bexp |= 0xfffffff0;
//  Rexp = pSDR->Rexp_Bexp >> 4;
//  if (Rexp & 0x8)
//    Rexp |= 0xfffffff0;			// extend sign bit
//  Rf = pow(10., -1* ((float) Rexp));
//  Bf *= pow(10., (float) Bexp);
//  retval = (((float) readoutval)*Rf - Bf)/Mf;
//  return retval;
//}
//
//
//unsigned char get_gpio_readout_value_readout_value(long junk) {
//  // returns current state of pins labeled GPIO7:GPIO0 as byte value, assuming their reference position
//  // as the first 8 bits of port B.  If the pins move or get reordered, this code will have to change!!!
//  unsigned long portBval = 0x000;
////TODO - add if necessary
//  //portBval = GPIO.port[1].pvr;
//  return portBval & 0xff;
//}
//
//
//int Process_Internal_ADC_Sensors(event eventID, void* arg)
//{
//	  int i1;
//	  unsigned short assertion_mask, deassertion_mask, cur_comp_state;
//	  sensor_data_entry_t* pSD;
//
//	  for (i1=FIRST_INT_SENSOR; i1 <= LAST_INT_SENSOR; i1++) {
//	    pSD = &SensorData[i1];
//	    if ((pSD->pSDR == NULL) || (pSD->readout_function == NULL))
//	      continue;     // sensor may not be actively in SDR in this configuration, so skip
//
//	    // first update sensor value
//	    pSD->readout_value = (*(pSD->readout_function))(pSD->readout_func_arg);
//
//	    // now update threshold states
//	    if ((pSD->pSDR->sensorcap & 0x0c) != 0x00) {
//	      // sensor has thresholds, apply comparators
//	      cur_comp_state = get_thr_sensor_state(pSD->pSDR, pSD->cur_masked_comp, pSD->readout_value);
//
//	      // update comparator status which is returned with IPMI Get Sensor Reading command
//	      // Per IPMI spec:
//	      // [7:6] - reserved. Returned as 1b. Ignore on read.
//	      // [5] - 1b = at or above (>=) upper non-recoverable threshold
//	      // [4] - 1b = at or above (>=) upper critical threshold
//	      // [3] - 1b = at or above (>=) upper non-critical threshold
//	      // [2] - 1b = at or below (<=) lower non-recoverable threshold
//	      // [1] - 1b = at or below (<=) lower critical threshold
//	      // [0] - 1b = at or below (<=) lower non-critical threshold
//
//	      pSD->comparator_status = 0xc0;
//	      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_nonrecover_thr) ? SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK : 0;
//	      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_critical_thr) ? SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK : 0;
//	      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_noncritical_thr) ? SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK : 0;
//	      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_nonrecover_thr) ? SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK : 0;
//	      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_critical_thr) ? SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK : 0;
//	      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_noncritical_thr) ? SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK : 0;
//	    }
//	    else {
//	      cur_comp_state = 0;      // no thresholds
//	      pSD->comparator_status = 0xc0;
//	    }
//
//	    // determine system power context.  If sensor is in context then examine threshold status and determine if
//	    // IPMI events should be generated
//	    // pSD->prev_masked_comp = pSD->cur_masked_comp;
//	    switch (pSD->active_context_code) {
//	      case SENSORACTV_ALWAYS:
//	        break;              // proceed to further event processing
//
//	      case SENSORACTV_PAYLOADPWRON:
//	        if ((pyldmgr_get_payload_power_status() == power_off) || (SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK))  {
//	          pSD->cur_masked_comp = 0;         // no thresholds set when payload supply is off or about to be off
//	          continue;
//	        }
//	        break;
//
//	      case SENSORACTV_BACKENDPWRON:
//	        if ((pyldmgr_get_backend_power_status() == power_off) || (SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK)){
//	          pSD->cur_masked_comp = 0;         // no thresholds set when backend power is off or about to be off
//	          continue;
//	        }
//	        break;
//
//	      default:
//	     continue;           // undefined context, skip to next sensor
//	}
//
//	// examine masked comparator states and send any IPMI events assocated wit hthem
//	pSD->cur_masked_comp = cur_comp_state;
//	assertion_mask = pSD->cur_masked_comp & (~(pSD->prev_masked_comp)) & (pSD->pSDR->assertion_event_mask[LOWBYTE] | ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
//	deassertion_mask = (~(pSD->cur_masked_comp)) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] | ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));
//	process_sensor_events(assert, i1, assertion_mask);
//	process_sensor_events(deassert, i1, deassertion_mask);
//
//	pSD->prev_masked_comp = pSD->cur_masked_comp;
//
// }
//
//return 1;     // stay resident
//


//  int i1;
//  unsigned short assertion_mask, deassertion_mask, cur_comp_state;
//  sensor_data_entry_t* pSD;
//
//  for (i1=FIRST_INT_ANALOG_SENSOR; i1 <= LAST_INT_ANALOG_SENSOR; i1++) {
//    pSD = &SensorData[i1];
//    if ((pSD->pSDR == NULL) || (pSD->readout_function == NULL))
//      continue;     // sensor may not be actively in SDR in this configuration, so skip
//
//    // first update sensor value
//    pSD->readout_value = (*(pSD->readout_function))(pSD->readout_func_arg);
//
//    // now update threshold states
//    if ((pSD->pSDR->sensorcap & 0x0c) != 0x00) {
//      // sensor has thresholds, apply comparators
//      cur_comp_state = get_thr_sensor_state(pSD->pSDR, pSD->cur_masked_comp, pSD->readout_value);
//
//      // update comparator status which is returned with IPMI Get Sensor Reading command
//      // Per IPMI spec:
//      // [7:6] - reserved. Returned as 1b. Ignore on read.
//      // [5] - 1b = at or above (>=) upper non-recoverable threshold
//      // [4] - 1b = at or above (>=) upper critical threshold
//      // [3] - 1b = at or above (>=) upper non-critical threshold
//      // [2] - 1b = at or below (<=) lower non-recoverable threshold
//      // [1] - 1b = at or below (<=) lower critical threshold
//      // [0] - 1b = at or below (<=) lower non-critical threshold
//
//      pSD->comparator_status = 0xc0;
//      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_nonrecover_thr) ? SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK : 0;
//      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_critical_thr) ? SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK : 0;
//      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_noncritical_thr) ? SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK : 0;
//      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_nonrecover_thr) ? SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK : 0;
//      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_critical_thr) ? SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK : 0;
//      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_noncritical_thr) ? SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK : 0;
//    }
//    else {
//      cur_comp_state = 0;      // no thresholds
//      pSD->comparator_status = 0xc0;
//    }
//
//    // determine system power context.  If sensor is in context then examine threshold status and determine if
//    // IPMI events should be generated
//    pSD->prev_masked_comp = pSD->cur_masked_comp;
//    switch (pSD->active_context_code) {
//      case SENSORACTV_ALWAYS:
//        break;              // proceed to further event processing
//
//      case SENSORACTV_PAYLOADPWRON:
//        if ((pyldmgr_get_payload_power_status() == power_off) || (SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK))  {
//          pSD->cur_masked_comp = 0;         // no thresholds set when payload supply is off or about to be off
//          continue;
//        }
//        break;
//
//      case SENSORACTV_BACKENDPWRON:
//        if ((pyldmgr_get_backend_power_status() == power_off) || (SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK)){
//          pSD->cur_masked_comp = 0;         // no thresholds set when backend power is off or about to be off
//          continue;
//        }
//        break;
//
//      default:
//        continue;           // undefined context, skip to next sensor
//    }
//
//    // examine masked comparator states and send any IPMI events assocated wit hthem
//    pSD->cur_masked_comp = cur_comp_state;
//    assertion_mask = pSD->cur_masked_comp & (~(pSD->prev_masked_comp)) & (pSD->pSDR->assertion_event_mask[LOWBYTE] | ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
//    deassertion_mask = (~(pSD->cur_masked_comp)) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] | ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));
//    process_sensor_events(assert, i1, assertion_mask);
//    process_sensor_events(deassert, i1, deassertion_mask);
//  }
//  return 1;     // stay resident
//}


static uint8_t eject_handle_callback(event eventID, void* arg) {
  // this function gets called when a change in the stable handle position has been detected
  // result should be that the hotswap sensor state is updated, and a hotswap event message
  // generated, if appropriate
  Eject_Hdl_Pos curpos = get_eject_handle_stable_state();
  ipmb_msg_desc_t reqmsg;

  if (curpos == in_closed) {
    // handle closed
    SensorData[HOTSWAP_SENSOR].readout_value &= ~HOTSWAP_MODULE_HANDLE_OPEN_MASK;
    SensorData[HOTSWAP_SENSOR].readout_value |= HOTSWAP_MODULE_HANDLE_CLOSED_MASK;
    sensor_build_hotswap_sensor_message(&reqmsg, HOTSWAP_EVENT_HANDLE_CLOSED);

    //*** Special Code 19-Apr-2011 for systems with dumb power supplies ***
    //In systems with managed power, the payload power supply will be off when the hotswap
    //handle is closed.  However, on systems with a dumb power supply, this may not be the
    //case, as power is applied as soon as the diode is detected.  This presents a problem
    //as the IPMI spec calls for the Quieced bit in the hotswap sensor to be cleared when
    //payload power turn-on is detected.  Since the state machine uses the Quiesce flag to
    //mask sensor events at system turn-on/turn-off, the possibility exists that if the bit
    //gets set, it will never clear until the module is physically removed.  So this code
    //simulates the normal reapplication of 12V power by sending out a simulated power-on
    //detect event, even though the 12V power never went away (because of the dumb supply).
    if ((pyldmgr_get_payload_power_status() == power_on) && (SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK))
      post_swevent(PYLDMGREV_PAYLD_PWR_ON_DETECT, NULL);
  }
  else {
    // handle opened
    SensorData[HOTSWAP_SENSOR].readout_value &= ~HOTSWAP_MODULE_HANDLE_CLOSED_MASK;
    SensorData[HOTSWAP_SENSOR].readout_value |= HOTSWAP_MODULE_HANDLE_OPEN_MASK;
    sensor_build_hotswap_sensor_message(&reqmsg, HOTSWAP_EVENT_HANDLE_OPENED);
  }
  ipmb_send_request(&reqmsg, NULL);
  return (1);
}


void sensor_build_hotswap_sensor_message(volatile ipmb_msg_desc_t* preq, volatile unsigned char event_data)
{
  volatile uint8_t* prqdata = preq->buf + IPMI_RQ_DATA_OFFSET-1;     // request data pointer, offset for 1-base indexing

  ipmb_init_req_hdr(preq->buf, ipmb_get_event_rcvr_ipmb_addr(), NETFN_SE, ipmb_get_event_rcvr_lun(), 0);

  prqdata[0] = IPMICMD_SE_PLATFORM_EVENT;
  prqdata[1] = 0x04;                                        // Event Message Revision = 0x04 (IPMI spec)
  prqdata[2] = 0xf2;                                        // sensor type = module hot swap
  prqdata[3] = HOTSWAP_SENSOR;                              // sensor number
  prqdata[4] = 0x6f;                                        // generic availability
  prqdata[5] = event_data;                                  // input arg
  prqdata[6] = calc_ipmi_xsum(&preq->buf[3], 8);            // message checksum
  preq->len = 12;
}


uint16_t get_thr_sensor_state(SDR_type_01h_t* pSDR, uint16_t prev_comp_state, uint8_t raw_value)
{

  volatile uint16_t retval = prev_comp_state;
  volatile uint8_t neghyst = pSDR->neg_thr_hysteresis;
  volatile uint8_t poshyst = pSDR->pos_thr_hysteresis;

  if (raw_value >= pSDR->upper_nonrecover_thr)
    retval |= SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;
  else if (raw_value < pSDR->upper_nonrecover_thr-poshyst)
    retval &= ~SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK;

  if (raw_value <= pSDR->upper_nonrecover_thr)
    retval |= SENSOREV_UPPER_NONRECOVER_LOW_ASSERT_MASK;
  else if (raw_value > pSDR->upper_nonrecover_thr+neghyst)
    retval &= ~SENSOREV_UPPER_NONRECOVER_LOW_ASSERT_MASK;

  if (raw_value >= pSDR->upper_critical_thr)
    retval |= SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK;
  else if (raw_value < pSDR->upper_critical_thr-poshyst)
    retval &= ~SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK;

  if (raw_value <= pSDR->upper_critical_thr)
    retval |= SENSOREV_UPPER_CRITICAL_LOW_ASSERT_MASK;
  else if (raw_value > pSDR->upper_critical_thr+neghyst)
    retval &= ~SENSOREV_UPPER_CRITICAL_LOW_ASSERT_MASK;

  if (raw_value >= pSDR->upper_noncritical_thr)
    retval |= SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK;
  else if (raw_value < pSDR->upper_noncritical_thr-poshyst)
    retval &= ~SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK;

  if (raw_value <= pSDR->upper_noncritical_thr)
    retval |= SENSOREV_UPPER_NONCRITICAL_LOW_ASSERT_MASK;
  else if (raw_value > pSDR->upper_noncritical_thr+neghyst)
    retval &= ~SENSOREV_UPPER_NONCRITICAL_LOW_ASSERT_MASK;

  if (raw_value >= pSDR->lower_nonrecover_thr)
    retval |= SENSOREV_LOWER_NONRECOVER_HIGH_ASSERT_MASK;
  else if (raw_value < pSDR->lower_nonrecover_thr-poshyst)
    retval &= ~SENSOREV_LOWER_NONRECOVER_HIGH_ASSERT_MASK;

  if (raw_value <= pSDR->lower_nonrecover_thr)
    retval |= SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK;
  else if (raw_value > pSDR->lower_nonrecover_thr+neghyst)
    retval &= ~SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK;

  if (raw_value >= pSDR->lower_critical_thr)
    retval |= SENSOREV_LOWER_CRITICAL_HIGH_ASSERT_MASK;
  else if (raw_value < pSDR->lower_critical_thr-poshyst)
    retval &= ~SENSOREV_LOWER_CRITICAL_HIGH_ASSERT_MASK;

  if (raw_value <= pSDR->lower_critical_thr)
    retval |= SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK;
  else if (raw_value > pSDR->lower_critical_thr+neghyst)
    retval &= ~SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK;

  if (raw_value >= pSDR->lower_noncritical_thr)
    retval |= SENSOREV_LOWER_NONCRITICAL_HIGH_ASSERT_MASK;
  else if (raw_value < pSDR->lower_noncritical_thr-poshyst)
    retval &= ~SENSOREV_LOWER_NONCRITICAL_HIGH_ASSERT_MASK;

  if (raw_value <= pSDR->lower_noncritical_thr)
    retval |= SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK;
  else if (raw_value > pSDR->lower_noncritical_thr+neghyst)
    retval &= ~SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK;

  return ( retval );
}


void process_sensor_events(event_assertion_type_t evtype, volatile uint8_t sensornum, uint16_t evtmask)
{
  int i1;
  volatile ipmb_msg_desc_t evtmsg;
  volatile sensor_data_entry_t* pSD;

  if ((sensornum < 0) || (sensornum >= MAX_SENSOR_CNT))
    return;                     // invalid sensor number
  pSD = &SensorData[sensornum];

  if (pSD->pSDR == NULL)
    return;                    // no associated SDR

  if (!(pSD->event_msg_ctl & SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK))
    return;     // event messages disabled for this sensor

  for (i1=0; i1<=SENSOREV_UPPER_NONRECOVER_HIGH_OFFSET; i1++)
    {
        if (evtmask & (1<<i1))
          {
            // send sensor event
            ipmb_init_req_hdr(evtmsg.buf, ipmb_get_event_rcvr_ipmb_addr(), NETFN_SE, ipmb_get_event_rcvr_lun(), 0);
            evtmsg.buf[5] = IPMICMD_SE_PLATFORM_EVENT;
            evtmsg.buf[6] = 0x04;           // event message revision
            evtmsg.buf[7] = pSD->pSDR->sensortype;
            evtmsg.buf[8] = pSD->pSDR->sensornum; //TODO verify this (unsigned char) sensornum;

            if (evtype == assert)
              evtmsg.buf[9] = 0x7f & pSD->pSDR->event_reading_type;   // bit 7=0 for assertion, event reading type should be 0x01 (threshold)
            else
              evtmsg.buf[9] = 0x80 | pSD->pSDR->event_reading_type;   // bit 7=1 for deassertion, event reading type should be 0x01 (threshold)

            evtmsg.buf[10] = ((unsigned char) i1 & 0xf) | 0x50;     // trigger reading in byte 2, threshold in byte 3, event offset bits 0-3
            evtmsg.buf[11] = pSD->readout_value;
            evtmsg.buf[12] = *(((unsigned char*) &pSD->pSDR->upper_nonrecover_thr) + (5-(i1>>1)));     // get appropriate threshold for event
            evtmsg.buf[13] = calc_ipmi_xsum(&evtmsg.buf[3], 10);
            evtmsg.len = 14;

            ipmb_send_request(&evtmsg, NULL);
    }
  }
}


int GPIO_sensor_read_callback(event eventID, void* arg) {
  // poll the GPIO port and get an updated sensor value
 // SensorData[GPIO_SENSOR].readout_value = (*(SensorData[GPIO_SENSOR].readout_function))(SensorData[GPIO_SENSOR].readout_func_arg);

  return (1);       // stay resident
}



