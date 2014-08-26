
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "include/ipmiCommonHeader.h"


static uint8_t sensor_update_callback(event eventID, void* arg);
static uint8_t payload_power_off_callback(event eventID, void* arg);
static uint8_t payload_power_on_callback(event eventID, void* arg);
static uint8_t pyldmgr_100ms_timer_callback(event eventID, void* arg);
static uint8_t pyldmgr_1sec_timer_callback(event eventID, void* arg);

static void check_alarm_status(void);

//const LED_activity_desc_t LED_Noncritical_Activity 	= {Blink, LEDOFF,	120, 	30};     // 1.5 sec period, on 20% of time
const LED_activity_desc_t LED_Critical_Activity 	= {Blink, LEDON, 	20, 	20};     // 0.4 sec period, on 50% of time

volatile uint32_t isr_event_mask;                     // used to relay events out of the ISR timespace to be processed by pyldmgr_service()
volatile payload_mgr_state_t pyldmgr_state;



void pyldmgr_ipmicmd_fru_ctrl(const unsigned char ctlcode)
{
  // run this function when an AMC 3.10 FRU control command is received.  For this callback,
  // arg points to the IPMB request message, indicating the type of control.  The calling
  // function formats and returns a response.  This function only needs to parse the request
  // and perform the appropriate control
  switch (ctlcode)
  {
    case FRU_CTLCODE_COLD_RST:   // cold reset
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "Cold Reset FRU Command\n");
      if (pyldmgr_get_backend_power_status() == power_on)
        pyldmgr_state.cold_reset_timer = COLD_RESET_TICKS;            // setting timer will cause power to be disabled while timer counts down
      break;

    case FRU_CTLCODE_WARM_RST:   // warm reset
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "Warm Reset FRU Command\n");
      if (pyldmgr_get_backend_power_status() == power_on)
        pyldmgr_state.warm_reset_timer = WARM_RESET_TICKS;
      break;

    case FRU_CTLCODE_REBOOT:   // graceful reboot
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "Graceful Reboot FRU Command\n");
      if (pyldmgr_get_backend_power_status() == power_on)
        pyldmgr_state.fpga_load_timer = FPGA_LOAD_TICKS;
      break;

    case FRU_CTLCODE_QUIESCE:   // quiesce
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "Quiesce FRU Command\n");
      pyldmgr_state.quiesce_timer = QUIESCE_DELAY_TICKS;
      break;

    case 3:
    default:
      // unknown or unsupported control option
      break;
  }
}

power_status_t pyldmgr_get_backend_power_status(void)
{
  // returns definitive on/off status of backend power, which depends on payload power status, back end power
  // enable pin state and backend power startup timer
  power_status_t retval;

  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  if (pyldmgr_get_payload_power_status() == power_on)
  {
    if ((get_backend_power_pin == BACKEND_PWR_ON) && (pyldmgr_state.backend_startup_timer == 0) &&
      (pyldmgr_state.bkend_pwr_ena == BACKEND_PWR_ON))
      retval = power_on;
    else
      retval = power_off;
  }
  else
    retval = power_off;

  Int_Restore(giflag);

  return (retval);
}


power_status_t pyldmgr_get_payload_power_status(void)
{
  // returns definitive on/off status of payload power, based on comparator state and timer
  power_status_t retval;

  Bool giflag = Is_global_interrupt_enabled();


  retval = ((pyldmgr_state.payload_pwr.cur_on_state) && (pyldmgr_state.payload_startup_timer == 0)) ? power_on : power_off;

  Enable_global_interrupt();

  return (retval);
}



void pyldmgr_init(void)
{
  memset((void *) &pyldmgr_state, 0, sizeof(pyldmgr_state));

  sio_putstr("Uninitialized Payload Manager default area detected.  Writing default image....\n");

  pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena = BACKEND_PWR_ON;         // default to backend power enabled
  pyldmgr_state.nonvol_settings.default_boot_mode = TRIGGER_BOOT_MODE;               // default to trigger mode
  pyldmgr_state.nonvol_settings.fpga_auto_cfg_inhibit = 1;                           // initialize with fpga auto config inhibited
  pyldmgr_state.nonvol_settings.bkend_pwr_shtdn_evt_ena = 0;                         // initialize with backend shutdown event disabled
  pyldmgr_state.nonvol_settings.global_mask_level = (unsigned char) status_OK;

  memset((void*) &pyldmgr_state.nonvol_settings.sensor_mask_level[0], (unsigned char) status_OK, MAX_SENSOR_CNT);

  pyldmgr_state.bkend_pwr_ena = pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena;

  pyldmgr_state.cold_reset_timer = 0;
  pyldmgr_state.quiesce_timer = 0;
  pyldmgr_state.quiesced_flag = 0;
  pyldmgr_state.payload_startup_timer = 0;
  pyldmgr_state.backend_startup_timer = 0;
  pyldmgr_state.autoconfig_poll_timer = 0;
  pyldmgr_state.bkend_monitor.cur_alarm_level = status_OK;
  pyldmgr_state.bkend_monitor.cur_alarm_source = none;
  pyldmgr_state.mmc_uptime = 0;
  pyldmgr_state.bkend_hottime = 0;

  // register timer callbacks
  register_timer_callback(pyldmgr_100ms_timer_callback, TEVENT_100MSEC);
  register_timer_callback(pyldmgr_1sec_timer_callback, TEVENT_1SEC);

  // register software event callbacks
  register_swevent_callback(payload_power_on_callback,  PYLDMGREV_PAYLD_PWR_ON_DETECT);
  register_swevent_callback(payload_power_off_callback, PYLDMGREV_PAYLD_PWR_OFF_DETECT);

  register_swevent_callback(sensor_update_callback,     SENSOR_DIAGNOSTIC_UPDATE);

}



void pyldmgr_service(void)
{
  volatile ipmb_msg_desc_t evtmsg;

  // check for events from ISRs
  if (isr_event_mask & PAYLDMGR_PWRON_ISR_DETECT)
    {
    // payload power on detected in ISR

    Bool giflag = Is_global_interrupt_enabled();
    Int_Disable(giflag);

    isr_event_mask &= ~PAYLDMGR_PWRON_ISR_DETECT;

    Int_Restore(giflag);

    sio_filt_putstr(TXTFILT_INFO, 1, "Payload Power On detected\n");

    post_swevent(PYLDMGREV_PAYLD_PWR_ON_DETECT, NULL);

    }

  if (isr_event_mask & PAYLDMGR_PWROFF_ISR_DETECT)
    {
    // payload power off detected in ISR

    Bool giflag = Is_global_interrupt_enabled();
    Int_Disable(giflag);

    isr_event_mask &= ~PAYLDMGR_PWROFF_ISR_DETECT;

    Int_Restore(giflag);

    sio_filt_putstr(TXTFILT_INFO, 1, "Payload Power Off detected\n");

    post_swevent(PYLDMGREV_PAYLD_PWR_OFF_DETECT, NULL);
    }

    check_alarm_status( );

  // check to see if quiescing timer has expired.  If it has, then the quiesced_flag is set,
  // indicating that a hotswap message should be generated
  if (pyldmgr_state.quiesced_flag)
    {
    pyldmgr_state.quiesced_flag = 0;      // clear flag
    sensor_build_hotswap_sensor_message(&evtmsg, HOTSWAP_EVENT_QUIESCED);
    SensorData[HOTSWAP_SENSOR].readout_value |= HOTSWAP_QUIESCED_MASK;

    ipmb_send_request(&evtmsg, NULL);
    }

}



void pyldmgr_ipmicmd_backend_pwr(const ipmb_msg_desc_t* prqmsg) {
  // called when a backend power command is received
  volatile ipmb_msg_desc_t evtmsg;
  const volatile uint8_t* prqdata = prqmsg->buf + IPMI_RQ_DATA_OFFSET-1;     // request data pointer, offset for 1-base indexing

  switch(*prqdata)
  {
    case 0:   // turn off backend power if not already off
      if (pyldmgr_state.bkend_pwr_ena == BACKEND_PWR_ON)
        {
          pyldmgr_state.bkend_pwr_ena = BACKEND_PWR_OFF;
          // reset timers to zero
          pyldmgr_state.cold_reset_timer = 0;
          pyldmgr_state.payload_startup_timer = 0;
          pyldmgr_state.backend_startup_timer = 0;
          pyldmgr_state.warm_reset_timer = 0;
          pyldmgr_state.fpga_load_timer = 0;
          // if configuration calls for it, send a backend power shutdown hotswap event message

          if (pyldmgr_state.nonvol_settings.bkend_pwr_shtdn_evt_ena)
          {
            sensor_build_hotswap_sensor_message(&evtmsg, HOTSWAP_EVENT_BACKEND_SHUTDOWN);
            ipmb_send_request(&evtmsg, NULL);
          }

        }

      break;

    case 1:   // turn on backend power bit if not already on
      pyldmgr_state.bkend_pwr_ena = BACKEND_PWR_ON;
      break;

    default:
      // unknown command parameter, don't do anything
      break;
  }
}


static uint8_t payload_power_off_callback(event eventID, void* arg)
{
  // reset power enable to auto-on-off default state
  pyldmgr_state.bkend_pwr_ena = pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena;

  // reset all timers to zero
  pyldmgr_state.cold_reset_timer = 0;
  pyldmgr_state.quiesce_timer = 0;
  pyldmgr_state.payload_startup_timer = 0;
  pyldmgr_state.backend_startup_timer = 0;
  pyldmgr_state.warm_reset_timer = 0;
  pyldmgr_state.fpga_load_timer = 0;

  //setDC_DC_ConvertersOFF();

  return  (1);
}


static uint8_t payload_power_on_callback(event eventID, void* arg)
{
  // clear the backend failure, shutdown, and quiesced bits in the hotswap sensor
  SensorData[HOTSWAP_SENSOR].readout_value &= ~(HOTSWAP_QUIESCED_MASK | HOTSWAP_BACKEND_PWR_FAILURE_MASK |
                                                HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK);

  pyldmgr_state.bkend_monitor.cur_alarm_level = status_OK;
  pyldmgr_state.bkend_monitor.cur_alarm_source = none;

  program_LED(IPMI_BLUELED_TBL_IDX,     Local_Control, &LED_Off_Activity);
//  program_LED(IPMI_LED1_TBL_IDX,        Local_Control, &LED_Off_Activity);
  program_LED(IPMI_LED2_TBL_IDX,        Local_Control, &LED_Off_Activity);

//TODO  // initialize the startup timer
  pyldmgr_state.payload_startup_timer = 0;//MONITOR_STARTUP_TICKS;

  setDC_DC_ConvertersON();

  return (1);
}



static uint8_t pyldmgr_100ms_timer_callback(event eventID, void* arg)
{
  // this routine runs on the 100ms event timer, and is used to implement the timed functions of the
  // payload manager, including reset timers, power supply startup timers, and the quiesce timer.  It runs
  // at the same rate as the internal ADC and so it is also used to update the payload state machine.

  //update payload power detect
  pyldmgr_state.payload_pwr.prev_on_state = pyldmgr_state.payload_pwr.cur_on_state;

  if (get_backend_power_pin)
    {
      pyldmgr_state.payload_pwr.cur_on_state = 1;

      if (pyldmgr_state.payload_pwr.prev_on_state == 0)
        isr_event_mask |= PAYLDMGR_PWRON_ISR_DETECT;
    }
  else
    {
      pyldmgr_state.payload_pwr.cur_on_state = 0;

      if (pyldmgr_state.payload_pwr.prev_on_state == 1)
         isr_event_mask |= PAYLDMGR_PWROFF_ISR_DETECT;
    }

  // examine cold reset timer
  if (pyldmgr_state.cold_reset_timer)
    {
      pyldmgr_state.cold_reset_timer--;
    if (!pyldmgr_state.cold_reset_timer)
      {
      // restart the back end holdoff timer to allow voltages time to come up
      if (pyldmgr_state.bkend_pwr_ena)
        pyldmgr_state.backend_startup_timer = MONITOR_STARTUP_TICKS+1;
      }
     }

  // examine warm reset timer
  if (pyldmgr_state.warm_reset_timer) {
    pyldmgr_state.warm_reset_timer--;
    if (!pyldmgr_state.warm_reset_timer) {
    //TODO
    //  set_warm_reset(FPGA_CPU_RESET_DEASSERT);
    }
    else
      {
     // set_warm_reset(FPGA_CPU_RESET_ASSERT);
      }
  }

  // examine flash load timer
  if (pyldmgr_state.fpga_load_timer)
    pyldmgr_state.fpga_load_timer--;

  // examine quiesce timer
  if (pyldmgr_state.quiesce_timer) {
    pyldmgr_state.quiesce_timer--;
    if (!pyldmgr_state.quiesce_timer)
      pyldmgr_state.quiesced_flag = 1;
  }

  // examine payload power startup timer
  if (pyldmgr_state.payload_startup_timer)
    pyldmgr_state.payload_startup_timer--;

  // examine backend startup timer
  if (pyldmgr_state.backend_startup_timer) {
    pyldmgr_state.backend_startup_timer--;
    if (!pyldmgr_state.backend_startup_timer)
      isr_event_mask |= PAYLDMGR_BKEND_TIMER_EXP_ISR_DETECT;
  }

  // examine autoconfig timer
  if (pyldmgr_state.autoconfig_poll_timer) {
    pyldmgr_state.autoconfig_poll_timer--;
    if (!pyldmgr_state.autoconfig_poll_timer)
      isr_event_mask |= PAYLDMGR_AUTOCONFIG_TIMER_FLAG;
  }

  return (1);
}


static uint8_t pyldmgr_1sec_timer_callback(event eventID, void* arg)
{
  // this routine runs on the 1 second event timer, and is used to implement mmc uptime counter
  // and the backend hot time counter
  pyldmgr_state.mmc_uptime++;

  if (get_backend_power_pin)
    pyldmgr_state.bkend_hottime++;

  return (1);
}


static void check_alarm_status(void)
{
  // This function is called during payload manager service.  It checks the various state variables that
  // affect the alarm status of the system, updating the alarm level and taking appropriate actions as
  // necessary
  int i1;

  volatile alarm_level_t working_alarm_level = status_OK;            // working alarm level for current status calculation
  volatile alarm_source_class_t working_alarm_source = none;         // working alarm source for current status calculation
  volatile alarm_level_t sensor_alarm_mask;
  volatile sensor_data_entry_t* pSD;

  volatile uint16_t cur_assert_mask;
  volatile ipmb_msg_desc_t evtmsg;

  if (pyldmgr_state.bkend_monitor.cur_alarm_level == fault)
    // in fault state, stay there!
    return;

  // determine working alarm level for this call, from available state inputs
  for (i1=FIRST_INT_SENSOR; i1<=LAST_INT_SENSOR; i1++)
    {
      if (pyldmgr_state.nonvol_settings.global_mask_level > pyldmgr_state.nonvol_settings.sensor_mask_level[i1])
        sensor_alarm_mask = (alarm_level_t) pyldmgr_state.nonvol_settings.global_mask_level;
      else
        sensor_alarm_mask = (alarm_level_t) pyldmgr_state.nonvol_settings.sensor_mask_level[i1];

      pSD = &SensorData[i1];

      cur_assert_mask = pSD->cur_masked_comp & (pSD->pSDR->assertion_event_mask[LOWBYTE] |
                                               ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));

      // assess severity level, starting with most nonrecoverable
      if (cur_assert_mask & (SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONRECOVER_LOW_ASSERT_MASK |
           SENSOREV_LOWER_NONRECOVER_HIGH_ASSERT_MASK | SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK))
        {
          // threshold at nonrecoverable severity
          if ((working_alarm_level < fault) && (sensor_alarm_mask < fault))
            {
              working_alarm_source = (pSD->pSDR->sensortype == 0x01) ? temperature : voltage;
              working_alarm_level = fault;
            }
          continue;
        }

      if (cur_assert_mask & (SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_CRITICAL_LOW_ASSERT_MASK |
              SENSOREV_LOWER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK))
        {
          // threshold at critical
          if ((working_alarm_level < critical) && (sensor_alarm_mask < critical))
            {
              working_alarm_source = (pSD->pSDR->sensortype == 0x01) ? temperature : voltage;
              working_alarm_level = critical;
            }
          continue;
        }

      if (cur_assert_mask & (SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_UPPER_NONCRITICAL_LOW_ASSERT_MASK |
                    SENSOREV_LOWER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK))
        {
          // threshold at noncritical
          if ((working_alarm_level < noncritical) && (sensor_alarm_mask < noncritical))
            {
              working_alarm_source = (pSD->pSDR->sensortype == 0x01) ? temperature : voltage;
              working_alarm_level = noncritical;
            }
        }
  }

  // compare working alarm level to system state, making adjustments as necessary

  if (working_alarm_level != pyldmgr_state.bkend_monitor.cur_alarm_level)
    {
      // update alarm level
      pyldmgr_state.bkend_monitor.cur_alarm_level = working_alarm_level;
      pyldmgr_state.bkend_monitor.cur_alarm_source = (working_alarm_level == status_OK) ? none : working_alarm_source;

      switch (pyldmgr_state.bkend_monitor.cur_alarm_level)
        {
        case status_OK:
          // things have gone back to normal
          program_LED(IPMI_LED1_TBL_IDX, Local_Control, &LED_Off_Activity);
        break;

        case noncritical:
          program_LED(IPMI_LED1_TBL_IDX, Local_Control, &LED_Noncritical_Activity);
          break;

      case critical:
        program_LED(IPMI_LED1_TBL_IDX, Local_Control, &LED_Critical_Activity);
        break;

      case fault:
        // hit the nonrecoverable fault status--update LED state and send hotswap message bearing the news
        program_LED(IPMI_LED1_TBL_IDX, Local_Control, &LED_On_Activity);

        pyldmgr_state.cold_reset_timer = 0;
        pyldmgr_state.payload_startup_timer = 0;
        pyldmgr_state.backend_startup_timer = 0;
        pyldmgr_state.warm_reset_timer = 0;
        pyldmgr_state.fpga_load_timer = 0;

        if (pyldmgr_state.bkend_monitor.cur_alarm_source == voltage)
          {
            SensorData[HOTSWAP_SENSOR].readout_value |= HOTSWAP_BACKEND_PWR_FAILURE_MASK;
            sensor_build_hotswap_sensor_message(&evtmsg, HOTSWAP_EVENT_BACKEND_FAILURE);
            ipmb_send_request(&evtmsg, NULL);
          }
        else
          {
            SensorData[HOTSWAP_SENSOR].readout_value |= HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK;
            sensor_build_hotswap_sensor_message(&evtmsg, HOTSWAP_EVENT_BACKEND_SHUTDOWN);
            ipmb_send_request(&evtmsg, NULL);
          }
        break;

      default:
        // shouldn't come here, do nothing
        break;
    }
  }
}



static uint8_t sensor_update_callback(event eventID, void* arg)
{
  int i1;
//TODO verify if uint16_t is a proper length for masks
  volatile uint16_t assertion_mask, deassertion_mask, cur_comp_state;
  volatile sensor_data_entry_t* pSD;

  volatile power_status_t payld_pwr_status;

  volatile power_status_t bkend_pwr_status;

  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  // update payload power detect
  pyldmgr_state.payload_pwr.prev_on_state = pyldmgr_state.payload_pwr.cur_on_state;

//  if (SensorData[PAYLOAD_12V_SENSOR].readout_value >= pyldmgr_state.payload_pwr.upper_thr)
//    {
//      pyldmgr_state.payload_pwr.cur_on_state = 1;
//
//      if (pyldmgr_state.payload_pwr.prev_on_state == 0)
//        post_swevent(PYLDMGREV_PAYLD_PWR_ON_DETECT, NULL);
//    }
//
//  else
//    {
//      if (SensorData[PAYLOAD_12V_SENSOR].readout_value <= pyldmgr_state.payload_pwr.lower_thr)
//        {
//          pyldmgr_state.payload_pwr.cur_on_state = 0;
//
//          if (pyldmgr_state.payload_pwr.prev_on_state == 1)
//           post_swevent(PYLDMGREV_PAYLD_PWR_OFF_DETECT, NULL);
//        }
//  }

// update payload and backend power status, based on latest comparator state

  payld_pwr_status = pyldmgr_get_payload_power_status();
  bkend_pwr_status = pyldmgr_get_backend_power_status();

  // update comparators/event status for each sensor
  for (i1=FIRST_INT_SENSOR; i1<=LAST_INT_SENSOR; i1++)
    {
      pSD = &SensorData[i1];

    if (pSD->pSDR == NULL)
      continue;     // something very wrong in that SDR pointer is not initialized

    pSD->readout_value = (*(pSD->readout_function))(pSD->readout_func_arg);

    pSD->prev_masked_comp = pSD->cur_masked_comp;

    if ((pSD->pSDR->sensorcap & 0xc) != 0x00)
      {
      // sensor has thresholds, apply comparators
      cur_comp_state = get_thr_sensor_state(pSD->pSDR, pSD->prev_masked_comp, pSD->readout_value);
      // update comparator status which is returned with IPMI Get Sensor Reading command
      // Per IPMI spec:
      // [7:6] - reserved. Returned as 1b. Ignore on read.
      // [5] - 1b = at or above (>=) upper non-recoverable threshold
      // [4] - 1b = at or above (>=) upper critical threshold
      // [3] - 1b = at or above (>=) upper non-critical threshold
      // [2] - 1b = at or below (<=) lower non-recoverable threshold
      // [1] - 1b = at or below (<=) lower critical threshold
      // [0] - 1b = at or below (<=) lower non-critical threshold
      pSD->comparator_status = 0xc0;
      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_nonrecover_thr) ? SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK : 0;
      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_critical_thr) ? SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK : 0;
      pSD->comparator_status |= (pSD->readout_value >= pSD->pSDR->upper_noncritical_thr) ? SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK : 0;
      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_nonrecover_thr) ? SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK : 0;
      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_critical_thr) ? SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK : 0;
      pSD->comparator_status |= (pSD->readout_value <= pSD->pSDR->lower_noncritical_thr) ? SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK : 0;
    }
    else
      {
      cur_comp_state = 0;      // no thresholds
      pSD->comparator_status = 0xc0;
      }

    // All sensors are always scanned against their thresholds.  Which sensors will be ignored or have their
    // threshold status used depends on the sensor and the power state of the system.  The ambient temperature
    // sensor is always monitored.  The payload power thresholds are considered if payload comparator detects
    // sufficient voltage there for the payload power to be considered "on".  The remaining sensors are only
    // considered if the backend power is on (and the payload power is also on, which is factors into
    // the backend power status).  Also, if the quiesced mask bit is set in the hot swap sensor, it is
    // assumed that the +12V power will be going off at any time, if it is not already off, so no faults will
    // be generated or sensor events generated, except for ambient temperature sensor events (which the
    // shelf manager may choose to receive or ignore).

    switch (i1)
    {
      case FPGA_TEMP_SENSOR:

        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (bkend_pwr_status == power_on))
          {
            // back end power is on and stabilized, so get current event state from threshold comparators
            pSD->cur_masked_comp = cur_comp_state;
            assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] |
                              ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
            deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] |
                                  ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));

            // since not quiesced and backend power running, monitor for sensor event purposes

            process_sensor_events(assert, i1, assertion_mask);
            process_sensor_events(deassert, i1, deassertion_mask);

          }
        else
          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off

        break;

      case FMC1_TEMP_SENSOR:

        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (bkend_pwr_status == power_on))
          {
            // back end power is on and stabilized, so get current event state from threshold comparators
            pSD->cur_masked_comp = cur_comp_state;
            assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] |
                              ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
            deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] |
                                  ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));

            // since not quiesced and backend power running, monitor for sensor event purposes

            process_sensor_events(assert, i1, assertion_mask);
            process_sensor_events(deassert, i1, deassertion_mask);

          }
        else
          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off

        break;

      case FMC2_TEMP_SENSOR:

        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (bkend_pwr_status == power_on))
          {
            // back end power is on and stabilized, so get current event state from threshold comparators
            pSD->cur_masked_comp = cur_comp_state;
            assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] |
                              ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
            deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] |
                                  ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));

            // since not quiesced and backend power running, monitor for sensor event purposes

            process_sensor_events(assert, i1, assertion_mask);
            process_sensor_events(deassert, i1, deassertion_mask);

          }
        else
          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off

        break;

      case DC_DC_CONV_TEMP_SENSOR:

        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (bkend_pwr_status == power_on))
          {
            // back end power is on and stabilized, so get current event state from threshold comparators
            pSD->cur_masked_comp = cur_comp_state;
            assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] |
                              ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
            deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] |
                                  ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));

            // since not quiesced and backend power running, monitor for sensor event purposes

            process_sensor_events(assert, i1, assertion_mask);
            process_sensor_events(deassert, i1, deassertion_mask);

          }
        else
          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off

        break;

      case SDRAM_TEMP_SENSOR:

        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (bkend_pwr_status == power_on))
          {
            // back end power is on and stabilized, so get current event state from threshold comparators
            pSD->cur_masked_comp = cur_comp_state;
            assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] |
                              ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
            deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] |
                                  ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));

            // since not quiesced and backend power running, monitor for sensor event purposes

            process_sensor_events(assert, i1, assertion_mask);
            process_sensor_events(deassert, i1, deassertion_mask);

          }
        else
          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off

        break;

//      case PAYLOAD_12V_SENSOR:
//        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (payld_pwr_status == power_on))
//          {
//          // supply is detected as on and stabilized--get current event state from threshold comparators
//          pSD->cur_masked_comp = cur_comp_state;
//          assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] | ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
//          deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] | ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));
//          // since not quiesced and +12V running, monitor payload power for sensor event purposes
//          process_sensor_events(assert, i1, assertion_mask);
//          process_sensor_events(deassert, i1, deassertion_mask);
//        }
//        else
//          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off
//        break;
//
//

//      case BACKEND_3p3V_SENSOR:
//      case ADC1_SENSOR:
//      case ADC3_SENSOR:
//      case ADC4_SENSOR:
//      case ADC5_SENSOR:
//        if (!(SensorData[HOTSWAP_SENSOR].readout_value & HOTSWAP_QUIESCED_MASK) && (bkend_pwr_status == power_on)) {
//          // back end power is on and stabilized, so get current event state from threshold comparators
//          pSD->cur_masked_comp = cur_comp_state;
//          assertion_mask = pSD->cur_masked_comp & (~pSD->prev_masked_comp) & (pSD->pSDR->assertion_event_mask[LOWBYTE] | ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));
//          deassertion_mask = (~pSD->cur_masked_comp) & pSD->prev_masked_comp & (pSD->pSDR->deassertion_event_mask[LOWBYTE] | ((pSD->pSDR->deassertion_event_mask[HIGHBYTE] & 0xf) << 8));
//          // since not quiesced and backend power running, monitor for sensor event purposes
//          process_sensor_events(assert, i1, assertion_mask);
//          process_sensor_events(deassert, i1, deassertion_mask);
//        }
//        else
//          pSD->cur_masked_comp = 0;         // no thresholds set when supply is off or about to be off
//        break;

      default:
        break;
    }
  }

  Int_Restore(giflag);

  return (1);
}




