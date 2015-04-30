
#include "LPC17xx.h"
#include <stdio.h>
#include <string.h>

#include "include/utils.h"
#include "include/swevent.h"

#include "include/timer_callback.h"
#include "include/ejecthandle.h"

#include "include/comExpress.h"

#include "include/sio_usart.h"

#include "include/ipmi_i2c_driver.h"

#include "include/LEDdrivers.h"

#include "include/fru.h"

#include "include/ipmb_svc.h"
#include "include/ipmi_cmd_parser.h"

#include "include/sensor_sdr.h"
#include "include/payload_mgr.h"

#define IPMI_DBG

//#ifdef AMC_CPU_COM_Express
//volatile uint8_t commExpressLoggedIn = 0x00;
//#endif

// storage for IPMI command override LED activitiy descriptors
LED_activity_desc_t IPMI_LED_overrides[IPMI_LED_CNT];

static uint8_t ipmi_req_parser(event eventID, void* parg);

static void parse_App_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

static void parse_SensEvt_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

static void parse_Storage_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

static void parse_PICMG_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

//static void parse_Custom_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

//static void parse_IPMI_cmds( const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp );



void ipmi_cmd_parser_init( void )
{
  register_swevent_callback(ipmi_req_parser, IPMBEV_REQ_RCVD);		// register for received requests
}


uint8_t ipmi_req_parser(event eventID, void* parg)
{
  // this routine is called through the swevent handler in response to the posting of a
  // IPMBEV_REQ_RCVD event by the IPMB service
  ipmb_msg_desc_t* preq = (ipmb_msg_desc_t*) parg;
  ipmb_msg_desc_t rspmsg;

  uint8_t netFN;

  // initialize response message header
  memset((void *) &rspmsg.buf, 0, sizeof(rspmsg.buf));  // zero out entire response message first

  ipmb_init_rsp_hdr(rspmsg.buf, preq->buf);				// response header generated from request header
  rspmsg.len = IPMBRSPOVHEAD; 							// length of response including hdr, comp code and xsums, but no data

  // get netFN
  netFN = IPMB_RQ_netFN_GET(preq->buf);

  switch(netFN) {
    case NETFN_SE:
      parse_SensEvt_cmds(preq, &rspmsg);
      break;

    case NETFN_STORAGE:
      parse_Storage_cmds(preq, &rspmsg);
      break;

    case NETFN_APP:
      parse_App_cmds(preq, &rspmsg);
      break;

//    case NETFN_CUSTOM:
//      parse_Custom_cmds(preq, &rspmsg);
//      break;

    case NETFN_GRPEXT:
  	  if (preq->len < NETFN_PICMG_MIN_RQ_LENGTH) {
  		  // msg too short
  	  	rspmsg.len = 0;
    		break;
  	  }
      if (preq->buf[IPMI_RQ_DATA_OFFSET] == NETFN_PICMG_IDENTIFIER)
      	parse_PICMG_cmds(preq, &rspmsg);
      else
       	rspmsg.len = 0;
      break;

    // these are unsupported
    case NETFN_FIRMWARE:
    case NETFN_TRANSPORT:
    case NETFN_CHASSIS:
    case NETFN_BRIDGE:
    // completely unknown netFN
    default:
      rspmsg.len = 0;
      break;
  }

  // check response message length--if zero, it has been trapped as a bad or as-yet-unsupported
  // message
  if (rspmsg.len){
      ipmb_send_response(&rspmsg);

      #ifdef IPMI_DBG
        sio_putstr("\nIPMI Request:\n");
        ipmb_msg_dump(preq, TXTFILT_DBG_DETAIL);
        sio_putstr("\nIPMI Response:\n");
        ipmb_msg_dump(&rspmsg, TXTFILT_DBG_DETAIL);
      #endif
  }
  else {
  	sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "?Invalid or Unsupported Request Message:\n");
  	ipmb_msg_dump(preq, TXTFILT_DBG_DETAIL);
        rspmsg.len = IPMBRSPOVHEAD;			// put length back
        IPMI_RS_CCODE(rspmsg.buf) = IPMI_RS_INVALID_CMD;
        ipmb_send_response(&rspmsg);
  }

  return (1);					// callback stays resident
}


void parse_App_cmds(const ipmb_msg_desc_t* preq, ipmb_msg_desc_t* prsp) {

	const volatile uint8_t* prqdata  = &preq->buf[IPMI_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
 	volatile uint8_t* prsdata 		 = &prsp->buf[IPMI_RS_DATA_OFFSET-1];		// offset pointer for 1-base refs to rsp data

	const volatile LED_state_rec_t* pLEDstate;

  switch (IPMI_RQ_CMD(preq->buf)) {
    case IPMICMD_APP_GET_DEVICE_ID:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_APP_GET_DEVICE_ID\n");

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

      uint8_t len = sizeof(App_Device_ID_record_t);
      uint8_t i = 0;

      for(; len>0; len--)
      {
    	  prsdata[ 2+i ] = 	fru_buf[APP_DEV_ID_BYTE_OFFSET+i];
          i++;
      }

      prsp->len += 11;
      break;

    case IPMICMD_APP_COLD_RESET:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_APP_COLD_RESET\n");
      // use unserviced watchdog timer to trigger cold reset
      Enable_Watchdog();
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    case IPMICMD_PICMG_GET_FRU_LED_STATE:
          sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_FRU_LED_STATE\n");
          prsdata[2] = NETFN_PICMG_IDENTIFIER;
          if (prqdata[3] >= IPMI_LED_CNT) {
            IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
            prsp->len += 1;
            break;
          }
          // need to poke around in the LED state, so turn off interrupts for a little
          // bit while we see what's going on with the LED
          Disable_global_interrupt();
          pLEDstate = &LEDstate[prqdata[3]];      // pointer to LED state
          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
          // response byte 3, LED states
          prsdata[3] = 0x00;
          if (pLEDstate->LEDstate == Local_Control)
            prsdata[3] |= 0x01;
          else {
            if (pLEDstate->pOvrideDesc->delay2 == 0)
              prsdata[3] |= 0x04;         // lamp test
            else
              prsdata[3] |= 0x02;         // override
          }
          // response byte 4/5/6, local control
          switch (pLEDstate->pLocalDesc->action) {
            case On:
              prsdata[4] = 0xff;
              prsdata[5] = 0;
              break;
            case Blink:
              if (pLEDstate->pLocalDesc->initstate == LEDON) {
                prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
                prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
              }
              else {
                prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
                prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
              }
              break;
            case Off:
            case Bypass:

            default:
              prsdata[4] = 0;
              prsdata[5] = 0;
             break;
          }
          prsdata[6] = pLEDstate->Color;
          prsp->len += 5;             // update length for bytes encoded so far
          // response bytes 7-9, override state
          if (prsdata[3] > 1) {
            // have override bytes to report
            prsdata[9] = pLEDstate->Color;
            switch (pLEDstate->pOvrideDesc->action) {
              case On:
                prsdata[7] = 0xff;
                prsdata[8] = 0;
                prsp->len += 3;
                break;
              case Blink:
                if (pLEDstate->pOvrideDesc->delay2 == 0) {
                  // lamp test
                  prsdata[7] = 0xfb;
                  prsdata[8] = 0;
                  prsdata[10] = (unsigned char) ((pLEDstate->pOvrideDesc->delay1/10) & 0xff);
                  prsp->len += 4;
                }
                else {
                  if (pLEDstate->pOvrideDesc->initstate == LEDON) {
                    prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
                    prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
                  }
                  else {
                    prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
                    prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
                  }
                  prsp->len += 3;
                }
              break;
              case Off:
              case Bypass:
              default:
                prsdata[7] = 0;
                prsdata[8] = 0;
                prsp->len += 3;
            }
          }
          Enable_global_interrupt();
          break;


    default:
      prsp->len = 0;			// nothing supported here yet
     break;
  }
}


void parse_SensEvt_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp)
{
  const volatile uint8_t* prqdata 	= &preq->buf[IPMI_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  volatile uint8_t* prsdata 		= &prsp->buf[IPMI_RS_DATA_OFFSET-1];		// offset pointer for 1-base refs to rsp data

  volatile unsigned short rsvID, recordID;

  SDR_entry_hdr_t* pSDRhdr;
  SDR_type_01h_t*  pSensorSDR;

  volatile uint8_t* pSDRdata;
  volatile uint8_t* prsbyte;
  volatile uint8_t  rdlength;

  switch (IPMI_RQ_CMD(preq->buf)) {
    case IPMICMD_SE_SET_EVENT_RECEIVER:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_SET_EVENT_RECEIVER\n");
      ipmb_set_event_rcvr_ipmb_addr(prqdata[1]);
      ipmb_set_event_rcvr_lun(prqdata[2] & 0x3);
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

      rearm_sensor_events();                // signals that sensor events should be reissued

      break;

    case IPMICMD_SE_GET_EVENT_RECEIVER:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_EVENT_RECEIVER\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = ipmb_get_event_rcvr_ipmb_addr( );
      prsdata[3] = ipmb_get_event_rcvr_lun( ) & 0x3;
      prsp->len += 2;
      break;

    case IPMICMD_SE_GET_DEVICE_SDR_INFO:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_DEVICE_SDR_INFO\n");
     IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (IPMB_RQ_rsLUN_GET(preq->buf) != 0)
        prsdata[2] = 0;								// no sensors or devices for LUN != 0
      else {
        if (preq->len <= IPMBREQOVHEAD) {
      	  // operation data byte missing, so by definition return the number of sensors
          prsdata[2] = SDRstate.sensor_cnt;
        }
        else {
      	  if ( prqdata[1] & 0x1 )
      		prsdata[2] = SDRstate.SDR_cnt;
          else
            prsdata[2] = SDRstate.sensor_cnt;
        }
      }
      prsdata[3] = 0x01;				// per AMC.0 3.11.1, static sensors, LUN 0 only
      prsp->len += 2;
      break;

    case IPMICMD_SE_GET_DEVICE_SDR:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_DEVICE_SDR\n");

      rsvID = prqdata[2] << 8 | prqdata[1];			// reservation ID for this command

      if ((prqdata[5] != 0) && (rsvID != SDRstate.SDRreservationID))
      {
        // reservation ID doesn't match currently active one, so send an error completion code
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RSV_ID;
        break;
      }

      recordID = prqdata[4] << 8 | prqdata[3];
      pSDRdata = get_SDR_entry_addr(recordID);

      if (!pSDRdata)
      {
        // record ID out of range
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }

      pSDRhdr = (SDR_entry_hdr_t*) pSDRdata;

      if (prqdata[6] == 0xff)
    	rdlength = pSDRhdr->reclength - prqdata[5];
      else
        rdlength = prqdata[6];

      if (rdlength > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-3))
      {
    	  IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
    	  break;
      }
  	  else
  	  {
  	    // return read data
  	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

  	    recordID 	= get_SDR_next_recID(pSDRhdr);
  	    prsdata[2] 	= recordID & 0xff;
  	    prsdata[3] 	= recordID >> 8;
            prsp->len  += 2+rdlength;
 	    prsbyte 	= &prsdata[4];
 	    pSDRdata   += prqdata[5];         // advance data pointer to starting offset

 	    while (rdlength)
 	    {
 	    	*prsbyte++ = *pSDRdata++;
 	        rdlength--;
 	    }
      }

      break;

    case IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY\n");
      // bump SDR repository reservation counter and return the value

      SDRstate.SDRreservationID++;

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

	  prsdata[2] = SDRstate.SDRreservationID & 0xff;
      prsdata[3] = SDRstate.SDRreservationID >> 8;

      prsp->len += 2;

      break;

    case IPMICMD_SE_GET_SENSOR_HYSTERESIS:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_HYSTERESIS\n");
      // find SDR number from sensor number

      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);

      if (pSensorSDR == NULL)
      {
        // bad sensor ID
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

      prsp->len += 2;

      if (((pSensorSDR->sensorcap & 0x30) < 0x10) || ((pSensorSDR->sensorcap & 0x30) > 0x20))
      {
        // hysteresis not readable on this sensor
        prsdata[2] = 0x00;
        prsdata[3] = 0x00;
      }
      else
      {
        prsdata[2] = pSensorSDR->pos_thr_hysteresis;
        prsdata[3] = pSensorSDR->neg_thr_hysteresis;
      }

      break;

//    case IPMICMD_SE_SET_SENSOR_THRESHOLD:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_SET_SENSOR_THRESHOLD\n");
//
//      // This command will update the sensor thresholds in both the RAM and EEPROM-based SDR records for
//      // the specified sensor.
//      // Here's the trick:  we're taking advantage of the fact that the thresholds all fall within the
//      // same 32-byte block in the EEPROM, so we can build a buffer image of the current thresholds,
//      // update those that are changed by the command, and write the settings back to EEPROM with a single
//      // write operation, such that the EEPROM updates itself while the response is returned.  We avoid the
//      // situation where we have to wait for one or more block writes to complete before finishing the
//      // others, which would put the EEPROM internal update time for the earlier writes in the command
//      // processing path, which we want to avoid--lest the controller decide to reissue the commands
//
//      // find SDR number from sensor number
//      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
//
//      if (pSensorSDR == NULL)
//      {
//        // bad sensor ID
//      	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//      // initialize buffer with current or new threshold values
//      pSDRdata = (unsigned char*) &pSensorSDR->upper_nonrecover_thr;
//
//      for (i1=0; i1<6; i1++)
//        if ((prqdata[2] & (1 << i1)) & pSensorSDR->settable_threshold_mask)
//          // new value for threshold
//          pSDRdata[5-i1] = prqdata[3+i1];
// 	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//
//      eepspi_write(pSDRdata, SDR_AREA_BYTE_OFFSET+(unsigned short) (pSDRdata - (unsigned char*) &SDRtbl), 6);
//      break;

    case IPMICMD_SE_GET_SENSOR_THRESHOLD:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_THRESHOLD\n");

      // find SDR number from sensor number
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);

      if (pSensorSDR == NULL)
      {
        // bad sensor ID
      	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

	  prsdata[2] = pSensorSDR->readable_threshold_mask;
	  prsdata[8] = pSensorSDR->upper_nonrecover_thr;
	  prsdata[7] = pSensorSDR->upper_critical_thr;
	  prsdata[6] = pSensorSDR->upper_noncritical_thr;
	  prsdata[5] = pSensorSDR->lower_nonrecover_thr;
	  prsdata[4] = pSensorSDR->lower_critical_thr;
	  prsdata[3] = pSensorSDR->lower_noncritical_thr;

	  prsp->len += 7;

	 break;

//    case IPMICMD_SE_GET_SENSOR_EVENT_STATUS:
//    	sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_EVENT_STATUS\n");
//    	pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
//    	if (pSensorSDR == NULL) {
//    		// bad sensor ID
//    		IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//    	}
//    	// access sensor data table to get sensor value
//	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//	    if (prqdata[1] == HOTSWAP_SENSOR) {
//	      // return AMC-specified response for hotswap sensor
//	      prsdata[2] = 0x00;
//	      prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;   // scanning always enabled, plus event status
//	      prsdata[4] = SensorData[HOTSWAP_SENSOR].readout_value;
//	    }
//	    else {
//	      // return IPMI-specified response for temperature or voltage sensors
//	    //prsdata[2] = SensorData[prqdata[1]].readout_value;
//	      prsdata[2] = SensorData[prqdata[1]].event_msg_ctl | 0x40;		// scanning always enabled, plus event status
//	      prsdata[3] = 0x80; //SensorData[prqdata[1]].comparator_status;
//      }
//      prsp->len += 2;
//
//      break;

    case IPMICMD_SE_GET_SENSOR_READING:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_READING\n");

      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);

      if (pSensorSDR == NULL)
      {
        // bad sensor ID
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }

      // access sensor data table to get sensor value
	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

	    if (prqdata[1] == HOTSWAP_SENSOR)
	    {
	      // return AMC-specified response for hotswap sensor
	      prsdata[2] = 0x00;
	      prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;   // scanning always enabled, plus event status
	      prsdata[4] = SensorData[HOTSWAP_SENSOR].readout_value;
	    }
	    else
	    {
	      // return IPMI-specified response for temperature or voltage sensors
	      prsdata[2] = SensorData[prqdata[1]].readout_value;
	      prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;		// scanning always enabled, plus event status
	      prsdata[4] = SensorData[prqdata[1]].comparator_status;
	    }

	  prsp->len += 3;

	break;

    case IPMICMD_SE_GET_SENSOR_EVENT_ENABLE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_EVENT_ENABLE\n");
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      if ((pSensorSDR->event_reading_type > 0x0c) || (pSensorSDR->event_reading_type < 0x01) || ((pSensorSDR->sensorcap & 0x3) == 0x3)) {
        // sensor event/reading type not supported or per event enable/disable not allowed for this sensor
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      prsdata[2] = 0x40 | (SensorData[prqdata[1]].event_msg_ctl & SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK);
      prsdata[3] = pSensorSDR->assertion_event_mask[LOWBYTE];
      prsdata[5] = pSensorSDR->deassertion_event_mask[LOWBYTE];
      if (pSensorSDR->event_reading_type == 0x01) {
        // threshold type
        prsdata[4] = pSensorSDR->assertion_event_mask[HIGHBYTE] & 0xf;
        prsdata[6] = pSensorSDR->deassertion_event_mask[HIGHBYTE] & 0xf;
      }
      else {
        // discrete type
        prsdata[4] = pSensorSDR->assertion_event_mask[HIGHBYTE];
        prsdata[6] = pSensorSDR->deassertion_event_mask[HIGHBYTE];
      }
      prsp->len += 5;
      break;

    case IPMICMD_SE_SET_SENSOR_EVENT_ENABLE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_SET_SENSOR_EVENT_ENABLE\n");

      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);

      if (pSensorSDR == NULL)
      {
        // bad sensor ID
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }

      if ((pSensorSDR->event_reading_type > 0x0c) || (pSensorSDR->event_reading_type < 0x01) || ((pSensorSDR->sensorcap & 0x3) != 0x0))
      {
        // sensor event/reading type not supported or per event enable/disable not allowed for this sensor
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        break;
      }

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

      if (!(prqdata[2] & SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK))
        // disable event messages from this sensor
        SensorData[prqdata[1]].event_msg_ctl &= ~SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
      else
        // enable event messages from this sensor
        SensorData[prqdata[1]].event_msg_ctl |= SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;

      if (preq->len <= 9)
        break;      // no extra bytes

      switch (prqdata[2] & SENSOREV_MSG_CTL_SELECTED_EVENT_ACTION_MASK)
      {
        case 0x10:
          // enable selected events
          pSensorSDR->assertion_event_mask[LOWBYTE] |= prqdata[3];
          if (preq->len >= 11)
            pSensorSDR->assertion_event_mask[HIGHBYTE] |= prqdata[4];
          if (preq->len >= 12)
            pSensorSDR->deassertion_event_mask[LOWBYTE] |= prqdata[5];
            if (preq->len >= 13)
          pSensorSDR->deassertion_event_mask[HIGHBYTE] |= prqdata[6];
          pSDRdata = (unsigned char *) &(pSensorSDR->assertion_event_mask[LOWBYTE]);

          break;

        case 0x20:
          // disable selected events
          pSensorSDR->assertion_event_mask[LOWBYTE] &= ~prqdata[3];
          if (preq->len >= 11)
            pSensorSDR->assertion_event_mask[HIGHBYTE] &= ~prqdata[4];
          if (preq->len >= 12)
            pSensorSDR->deassertion_event_mask[LOWBYTE] &= ~prqdata[5];
          if (preq->len >= 13)
            pSensorSDR->deassertion_event_mask[HIGHBYTE] &= ~prqdata[6];
          pSDRdata = (unsigned char *) &(pSensorSDR->assertion_event_mask[LOWBYTE]);

          break;

        case 0x00:
        case 0x30:

        default:
          // nothing to do
          break;
      }
      break;

    case IPMICMD_SE_PLATFORM_EVENT:
      // should never get one of these incoming!!
    default:
      prsp->len = 0;			// nothing supported here yet
     break;
  }
}


void parse_Storage_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const volatile uint8_t* prqdata 	= &preq->buf[IPMI_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  volatile uint8_t* prsdata 		= &prsp->buf[IPMI_RS_DATA_OFFSET-1];		// offset pointer for 1-base refs to rsp data

  volatile uint16_t fruoffset;
  volatile uint8_t datalen;


  switch (IPMI_RQ_CMD(preq->buf))
  {
    case IPMICMD_STOR_GET_FRU_INVEN_AREA_INFO:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_GET_FRU_INVEN_AREA_INFO\n");

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

      prsdata[2] = FRU_AREA_SIZE_L;
      prsdata[3] = FRU_AREA_SIZE_H;
      prsdata[4] = 0;               // byte addressing

      prsp->len += 3;

    break;

    case IPMICMD_STOR_READ_FRU_DATA:
    	sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_READ_FRU_DATA\n");

    	fruoffset = prqdata[2] + (prqdata[3] << 8);
    	datalen = prqdata[4];

    	if (datalen > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-1))
    	{
    		IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
    		break;
    	}

    	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
    	prsdata[2] = datalen;

    	volatile uint8_t len = datalen;
    	volatile uint8_t i = 0;

    	for(	; len>0; len--)
    	{
    		prsdata[ 3+i ] = 	fru_buf[COMMON_HEADER_BYTE_OFFSET+fruoffset+i];
    		i++;
    	}

    	prsp->len += datalen+1;

    break;

    case IPMICMD_STOR_WRITE_FRU_DATA:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_WRITE_FRU_DATA\n");

      fruoffset = prqdata[2] + (prqdata[3] << 8);
      datalen = prqdata[4];

      if (datalen > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-1))
      {
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
        break;
      }

      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;

      prsdata[2] = datalen;

      len = datalen;
      i = 0;

      for(	; len>0; len--)
      {
    	  fru_buf[COMMON_HEADER_BYTE_OFFSET+fruoffset+i] = prsdata[ 4+i ];
      	  i++;
      }

      prsp->len += 1;

    break;

    default:
      prsp->len = 0;			// nothing supported here yet
      break;
  }
}


void parse_PICMG_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const volatile uint8_t* prqdata 	= &preq->buf[IPMI_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  volatile uint8_t* prsdata 		= &prsp->buf[IPMI_RS_DATA_OFFSET-1];		// offset pointer for 1-base refs to rsp data

  uint8_t i1;

  LED_activity_desc_t *pLEDact;
  const volatile LED_state_rec_t* pLEDstate;
  SDR_type_01h_t* pSensorSDR;

  switch (IPMI_RQ_CMD(preq->buf)) {

    case IPMICMD_PICMG_SET_AMC_PORT_STATE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_SET_AMC_PORT_STATE\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsp->len += 1;
      #ifdef AMC_CPU_COM_Express
        commExpressLoggedIn = 0xFF;
      #endif
     break;

    case IPMICMD_PICMG_GET_AMC_PORT_STATE:

    break;

    case IPMICMD_PICMG_SET_AMC_CLOCK_STATE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_SET_AMC_CLK_PORT_STATE\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsp->len += 1;
    break;

    case IPMICMD_PICMG_GET_AMC_CLOCK_STATE:

    break;

    case IPMICMD_PICMG_GET_PICMG_PROP:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_PICMG_PROP\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsdata[3] = 0x14;                      // per AMC Spec 3.1.5
      prsdata[4] = 2;//0;                         // max FRU device ID
      prsdata[5] = 2;//0;                         // FRU device ID for device containing this MMC
      prsp->len += 4;
      break;

    case IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      // capable of cold reset, warm reset, and graceful reboot
      prsdata[3] = 0x06;
      prsp->len += 2;
      break;

    case IPMICMD_PICMG_FRU_CONTROL:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_FRU_CONTROL\n");

      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsp->len += 1;
      if ((prqdata[3] == 3) || (prqdata[3] > 4))
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
      else
      {
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
        pyldmgr_ipmicmd_fru_ctrl(prqdata[3]);
      }
      break;

    case IPMICMD_PICMG_GET_FRU_LED_PROP:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_FRU_LED_PROP\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsdata[3] = 0x07;        // blue LED, LED1, LED2
      prsdata[4] = 0;           // don't report any application specific LEDs
      prsp->len += 3;
      break;

    case IPMICMD_PICMG_GET_LED_COLOR_CAP:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_LED_COLOR_CAP\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      switch (prqdata[3]) {
        case IPMI_BLUELED_TBL_IDX:
          prsdata[3] = 1 << LEDCOLOR_BLUE;      // blue only capability
          prsdata[4] = LEDCOLOR_BLUE;           // blue in local ctl state
          prsdata[5] = LEDCOLOR_BLUE;           // blue in override state
          prsdata[6] = 0;                       // no flags
          prsp->len += 5;
          break;

        case IPMI_LED1_TBL_IDX:
          prsdata[3] = 1 << LEDCOLOR_RED;      // red only capability
          prsdata[4] = LEDCOLOR_RED;           // red in local ctl state
          prsdata[5] = LEDCOLOR_RED;           // red in override state
          prsdata[6] = 0;                       // no flags
          prsp->len += 5;
          break;

        case IPMI_LED2_TBL_IDX:
          prsdata[3] = 1 << LEDCOLOR_GREEN;      // green only capability
          prsdata[4] = LEDCOLOR_GREEN;           // green in local ctl state
          prsdata[5] = LEDCOLOR_GREEN;           // green in override state
          prsdata[6] = 0;                       // no flags
          prsp->len += 5;
          break;

        default:
          // unknown LED
          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
          prsp->len += 1;
          break;
      }
      break;

    case IPMICMD_PICMG_SET_FRU_LED_STATE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_SET_FRU_LED_STATE\n");
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsp->len += 2;
      for (i1=0; i1<IPMI_LED_CNT; i1++)
        if ((prqdata[3] == i1) || (prqdata[3] == 0xff)) {
          // program this LED
          pLEDact = &IPMI_LED_overrides[i1];
          switch (prqdata[4]) {
            case 0x00:
              // off override
              program_LED(i1, Override, &LED_Off_Activity);
              break;
            case 0xfb:
              // lamp test
              pLEDact->action = Blink;
              pLEDact->initstate = LEDON;
              pLEDact->delay1 = prqdata[5]*10;            // duration given in 100ms units
              pLEDact->delay2 = 0;
              program_LED(i1, Override, pLEDact);
              break;

            case 0xfc:
              // restore to local control
              program_LED(i1, Local_Control, NULL);
              break;
            case 0xfd:
            case 0xfe:
              // don't do anything
              break;
            case 0xff:
              // on override
              program_LED(i1, Override, &LED_On_Activity);
              break;
            default:
              // blink override
              pLEDact->action = Blink;
              pLEDact->initstate = LEDOFF;
              pLEDact->delay1 = prqdata[4];            // off time
              pLEDact->delay2 = prqdata[5];            // on time
              program_LED(i1, Override, pLEDact);
          }
        }
      break;

    case IPMICMD_PICMG_GET_FRU_LED_STATE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_FRU_LED_STATE\n");
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      if (prqdata[3] >= IPMI_LED_CNT) {
        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        prsp->len += 1;
        break;
      }
      // need to poke around in the LED state, so turn off interrupts for a little
      // bit while we see what's going on with the LED
      Disable_global_interrupt();
      pLEDstate = &LEDstate[prqdata[3]];      // pointer to LED state
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      // response byte 3, LED states
      prsdata[3] = 0x00;
      if (pLEDstate->LEDstate == Local_Control)
        prsdata[3] |= 0x01;
      else {
        if (pLEDstate->pOvrideDesc->delay2 == 0)
          prsdata[3] |= 0x04;         // lamp test
        else
          prsdata[3] |= 0x02;         // override
      }
      // response byte 4/5/6, local control
      switch (pLEDstate->pLocalDesc->action) {
        case On:
          prsdata[4] = 0xff;
          prsdata[5] = 0;
          break;
        case Blink:
          if (pLEDstate->pLocalDesc->initstate == LEDON) {
            prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
            prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
          }
          else {
            prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
            prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
          }
          break;
        case Off:
        case Bypass:
        default:
          prsdata[4] = 0;
          prsdata[5] = 0;
         break;
      }
      prsdata[6] = pLEDstate->Color;
      prsp->len += 5;             // update length for bytes encoded so far
      // response bytes 7-9, override state
      if (prsdata[3] > 1) {
        // have override bytes to report
        prsdata[9] = pLEDstate->Color;
        switch (pLEDstate->pOvrideDesc->action) {
          case On:
            prsdata[7] = 0xff;
            prsdata[8] = 0;
            prsp->len += 3;
            break;
          case Blink:
            if (pLEDstate->pOvrideDesc->delay2 == 0) {
              // lamp test
              prsdata[7] = 0xfb;
              prsdata[8] = 0;
              prsdata[10] = (unsigned char) ((pLEDstate->pOvrideDesc->delay1/10) & 0xff);
              prsp->len += 4;
            }
            else {
              if (pLEDstate->pOvrideDesc->initstate == LEDON) {
                prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
                prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
              }
              else {
                prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
                prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
              }
              prsp->len += 3;
            }
          break;
          case Off:
          case Bypass:
          default:
            prsdata[7] = 0;
            prsdata[8] = 0;
            prsp->len += 3;
        }
      }
      Enable_global_interrupt();
      break;

    case IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID\n");
      // since the AMC specificaiton calls for a max device ID of zero, we expect this
      // command to give a device ID of zero as byte 2 of the command data, which is interpreted
      // as asking for the MMC device record index in the SDR
      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsdata[3] = (unsigned char) (SDR_MMC & 0xff);
      prsdata[4] = (unsigned char) (SDR_MMC >> 8);                // upper byte of MMC SDR record
      prsp->len += 3;
      break;

    case IPMICMD_SE_GET_SENSOR_EVENT_STATUS:
    	sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_EVENT_STATUS\n");
    	pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
    	if (pSensorSDR == NULL) {
    	 // bad sensor ID
    	 IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
    	 break;
    	}
    	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
    	prsdata[2] = 0x40;
    	   if (pSensorSDR->event_reading_type == 0x01) {
    	            // threshold type
    	            prsdata[3] = pSensorSDR->assertion_event_mask[HIGHBYTE] & 0xf;

    	   }
    	   else {
    	        // discrete type
    	          prsdata[3] = pSensorSDR->assertion_event_mask[HIGHBYTE];
    	   }
      prsp->len += 2;

      break;
    case IPMICMD_SE_GET_SENSOR_EVENT_ENABLE:
          sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_EVENT_ENABLE\n");
          pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
          if (pSensorSDR == NULL) {
            // bad sensor ID
            IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
            break;
          }
          if ((pSensorSDR->event_reading_type > 0x0c) || (pSensorSDR->event_reading_type < 0x01) || ((pSensorSDR->sensorcap & 0x3) == 0x3)) {
            // sensor event/reading type not supported or per event enable/disable not allowed for this sensor
            IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
            break;
          }
          prsdata[2] = 0x40 | (SensorData[prqdata[1]].event_msg_ctl & SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK);
          prsdata[3] = pSensorSDR->assertion_event_mask[LOWBYTE];
          prsdata[5] = pSensorSDR->deassertion_event_mask[LOWBYTE];
          if (pSensorSDR->event_reading_type == 0x01) {
            // threshold type
            prsdata[4] = pSensorSDR->assertion_event_mask[HIGHBYTE] & 0xf;
            prsdata[6] = pSensorSDR->deassertion_event_mask[HIGHBYTE] & 0xf;
          }
          else {
            // discrete type
            prsdata[4] = pSensorSDR->assertion_event_mask[HIGHBYTE];
            prsdata[6] = pSensorSDR->deassertion_event_mask[HIGHBYTE];
          }
          prsp->len += 5;
          break;

    default:
      prsp->len = 0;      // nothing supported here yet
     break;
  }
}




//void parse_Custom_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp)
//{
//  const volatile uint8_t* prqdata 	= &preq->buf[IPMI_RQ_DATA_OFFSET-1];   // offset pointer for 1-base refs to req data
//  volatile uint8_t* prsdata 		= &prsp->buf[IPMI_RS_DATA_OFFSET-1];     // offset pointer for 1-base refs to rsp data
//
//  uint8_t  xbuf[NONVOLATILE_MAX_XFER_LEN];
//  uint8_t* xbptr;
//  uint8_t* pmsgdata;
//  uint8_t  xlength, curxlen;
//
//  uint8_t eepsaddr;
//
//  uint64_t systime;
//
//  module_current_req_record_t currentrecbuf;
//
//  switch (IPMI_RQ_CMD(preq->buf)) {
//    case IPMICMD_CUSTOM_SET_BACKEND_PWR:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_BACKEND_PWR\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          backend on-off power enable.  Set to 1 to enable
//      //              backend power, 0 to disable.  Returns completion
//      //              code 0xcc if field has a different value
//      // Response Format:
//      //   1          Completion Code.
//      if (prqdata[1] > 1) {
//        // value out of range
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      pyldmgr_ipmicmd_backend_pwr(preq);
//      break;
//
//    case IPMICMD_CUSTOM_GET_BACKEND_PWR:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_BACKEND_PWR\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          Backend Power Setting (1=enabled, 0=disabled)
//      //   3          Backend Power Pin State (1=enabled, 0=disabled)
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = pyldmgr_state.bkend_pwr_ena;
//      prsdata[3] = get_backend_power_pin;
//      prsp->len += 2;
//      break;
//    case IPMICMD_CUSTOM_SET_PYLD_MGR_SETTING_REC:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_PYLD_MGR_SETTING_REC\n");
//      // Sets control field values for Payload Manager.  Settings are non-volatile.  Fields
//      // not being updated are ignored but should have data bytes present in the command.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Field select mask.  Bit=1 selects associated field for update, bit=0
//      //              leaves field unchanged.  Bit definitions as follows:
//      //              [7] - Default backend auto-power enable
//      //              [6] - Default boot mode
//      //              [5] - FPGA auto-configuration inhibit
//      //              [4] - Backend power shutdown event-enable
//      //              [3] - Backend Monitor global alarm level
//      //              [2:0] - Unused, reserved
//      //   2          Default backend auto-power enable setting (1=enable backend pwr when
//      //              payload power turns on, 0=disable backend power)
//      //   3          Default boot mode (0=trigger mode, 1=maintenance mode)
//      //   4          FPGA auto-configuration inhibit setting (1=inhibit FPGA-auto-configuration
//      //              function, 0=permit FPGA-auto-configuration)
//      //   5          Backend power shutdown event-enable setting (1=send hotswap event if
//      //              backend power is commanded to shut down, 0=do not send event)
//      //   6          Backend Monitor global alarm mask level (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
//      // Response Format:
//      //   1          Completion Code.
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      if (!prqdata[1])
//        break;      // nothing to write
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//       if (preq->len < IPMBREQOVHEAD+6) {
//        // not enough params
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
//        break;
//      }
//      // examine each select bit/field
//      if (prqdata[1] & 0x80) {
//        if (prqdata[2] > 1) {
//          // value out of range
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//          break;
//        }
//        pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena = prqdata[2];
//      }
//      if (prqdata[1] & 0x40) {
//        if (prqdata[3] > 1) {
//          // value out of range
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//          break;
//        }
//        pyldmgr_state.nonvol_settings.default_boot_mode = prqdata[3];
//      }
//      if (prqdata[1] & 0x20) {
//        if (prqdata[4] > 1) {
//          // value out of range
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//          break;
//        }
//        pyldmgr_state.nonvol_settings.fpga_auto_cfg_inhibit = prqdata[4];
//      }
//      if (prqdata[1] & 0x10) {
//        if (prqdata[5] > 1) {
//          // value out of range
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//          break;
//        }
//        pyldmgr_state.nonvol_settings.bkend_pwr_shtdn_evt_ena = prqdata[5];
//      }
//      if (prqdata[1] & 0x08) {
//        if (prqdata[6] > 3) {
//          // value out of range
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//          break;
//        }
//        pyldmgr_state.nonvol_settings.global_mask_level = prqdata[6];
//      }
//      eepspi_write((unsigned char*) &pyldmgr_state.nonvol_settings, PAYLDMGR_AREA_BYTE_OFFSET, 8);
//      break;
//
//    case IPMICMD_CUSTOM_GET_PYLD_MGR_SETTING_REC:
//
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_PYLD_MGR_SETTING_REC\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          Default backend auto-power enable setting (1=enable backend pwr when
//      //              payload power turns on, 0=disable backend power)
//      //   3          Default boot mode (0=trigger mode, 1=maintenance mode)
//      //   4          FPGA auto-configuration inhibit setting (1=inhibit FPGA-auto-configuration
//      //              function, 0=permit FPGA-auto-configuration)
//      //   5          Backend power shutdown event-enable setting (1=send hotswap event if
//      //              backend power is commanded to shut down, 0=do not send event)
//      //   6          Backend Monitor global alarm mask level (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena;
//      prsdata[3] = pyldmgr_state.nonvol_settings.default_boot_mode;
//      prsdata[4] = pyldmgr_state.nonvol_settings.fpga_auto_cfg_inhibit;
//      prsdata[5] = pyldmgr_state.nonvol_settings.bkend_pwr_shtdn_evt_ena;
//      prsdata[6] = pyldmgr_state.nonvol_settings.global_mask_level;
//      prsp->len += 5;
//      break;
//
//    case IPMICMD_CUSTOM_GET_FAULT_STATUS:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_FAULT_STATUS\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          alarm severity level (0=normal, 1=noncritical, 2=critical, 3=nonrecoverable)
//      //   3          alarm source (0=none, 1=temperature sensor, 2=voltage sensor, 3=backend device)
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = (unsigned char) pyldmgr_state.bkend_monitor.cur_alarm_level;
//      prsdata[3] = (unsigned char) pyldmgr_state.bkend_monitor.cur_alarm_source;
//      prsp->len += 2;
//      break;
//
//    case IPMICMD_CUSTOM_SET_BOOT_MODE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_BOOT_MODE\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Boot mode.  Set to 0 for Trigger Mode and 1 for Maintenance mode
//      // Response Format:
//      //   1          Completion Code.
//      if (prqdata[1] > 1) {
//        // illegal mode
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//        break;
//      }
//      set_boot_mode_pin(prqdata[1]);
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      break;
//
//    case IPMICMD_CUSTOM_GET_BOOT_MODE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_BOOT_MODE\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          Current Boot Mode.  Returns 0 for Trigger Mode and 1 for Maintenance mode
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = get_boot_mode_pin;
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_CUSTOM_SET_CURRENT_REQUIREMENT:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_CURRENT_REQUIREMENT\n");
//      // Command updates the current requirement record in the FRU multirecord area in
//      // nonvolatile storage.  The change will take effect the next time the FRU data area
//      // is read by the Carrier Manager.  Current is given in 100 mA units, to a maximum of
//      // 7.0 amps, with a minimum of 0.5A
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          12V Current Requirement in 100 mA units
//      // Response Format:
//      //   1          Completion Code.
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      if ((prqdata[1] < 5) || (prqdata[1] > 70)) {
//        // bad current specification
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//        break;
//      }
//      // load current record into memory and inspect it
//      eepspi_read((unsigned char*) &currentrecbuf, MULTIRECORD_AREA_BYTE_OFFSET, sizeof(currentrecbuf));
//      if ((currentrecbuf.PICMG_recID != 0x16) || (currentrecbuf.mfgID_LSB != 0x5a) || (currentrecbuf.mfgID_MidB != 0x31) ||
//        (currentrecbuf.mfgID_MSB != 0x00)) {
//        // something wrong, but this isn't the current requirement record--so abort
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//      currentrecbuf.current_draw_100mA = prqdata[1];    // set new current requirement
//      currentrecbuf.hdr.recxsum = calc_ipmi_xsum((unsigned char*) &(currentrecbuf.mfgID_LSB), sizeof(module_current_req_record_t)-
//        sizeof(multirecord_header_t));                  // update checksum
//      eepspi_write((unsigned char*) &currentrecbuf, MULTIRECORD_AREA_BYTE_OFFSET, sizeof(currentrecbuf));
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      break;
//
//    case IPMICMD_CUSTOM_SET_ANALOG_SCALE_FACTOR:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_ANALOG_SCALE_FACTOR\n");
//      // Command sets the scaling factors for an ADC input channel in nonvolatile storage.
//      // For changes to be observed in sensor operation, the MMC must be restarted by a power
//      // cycle or IPMI Application Cold Reset command (netFN=06h, Cmd=02h).
//      // BYTE         DATA FIELD
//      //   1          ADC channel number.  If omitted, command returns the number of ADC channels.
//      //              Channels are numbered consecutively, beginning at 0.
//      //   2          Field index.  Fields are indexed as follows:
//      //                0 - Scaling factor M numerator
//      //                1 - Scaling factor M denominator
//      //                2 - Offset B
//      //   3-6        Field value, as 32-bit signed integer, LS byte first
//      // Command Format:
//      //   1          Completion Code.
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      if ((prqdata[1] > MAX_ADC_CH_NUM) || (prqdata[2] > 2)) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//        break;
//      }
//      // compute EEPROM address
//      eepsaddr = ADC_SCALING_AREA_BYTE_OFFSET + prqdata[1]*sizeof(front_end_scaling_factors_t) + prqdata[2]*sizeof(long);
//      // use 'systime' as a scratch pad variable to build a 32-bit version of the new field value
//      systime = prqdata[6];
//      systime = (systime << 8) + prqdata[5];
//      systime = (systime << 8) + prqdata[4];
//      systime = (systime << 8) + prqdata[3];
//      eepspi_write((unsigned char*) &systime, eepsaddr, 4);
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      break;
//
//    case IPMICMD_CUSTOM_GET_ANALOG_SCALE_FACTOR:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_ANALOG_SCALE_FACTOR\n");
//      // Command returns the scaling factors for an ADC input channel.  If no arguments are
//      // specified, it returns the number of ADC channels.
//      // BYTE         DATA FIELD
//      //   1          ADC channel number.  If omitted, command returns the number of ADC channels.
//      //              Channels are numbered consecutively, beginning at 0.
//      // Command Format:
//      //   1          Completion Code.
//      //   2          Returns the ADC channel number specified in the command.  If omitted from
//      //   3-6        Scaling factor (M) numerator, as 32-bit signed integer, LS byte first.
//      //   7-10       Scaling factor (M) denominator, as 32-bit signed integer, LS byte first
//      //   11-14      Offset (B), as 32-bit signed integer, LS byte first.
//      if (preq->len > (IPMBREQOVHEAD)) {
//        // channel number specified
//        if (prqdata[1] > MAX_ADC_CH_NUM) {
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//          break;
//        }
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//        prsdata[2] = prqdata[1];
//        prsdata[3] = ADCscaletbl[prqdata[1]].Mnum & 0xff;
//        prsdata[4] = (ADCscaletbl[prqdata[1]].Mnum >> 8) & 0xff;
//        prsdata[5] = (ADCscaletbl[prqdata[1]].Mnum >> 16) & 0xff;
//        prsdata[6] = (ADCscaletbl[prqdata[1]].Mnum >> 24) & 0xff;
//        prsdata[7] = ADCscaletbl[prqdata[1]].Mdenom & 0xff;
//        prsdata[8] = (ADCscaletbl[prqdata[1]].Mdenom >> 8) & 0xff;
//        prsdata[9] = (ADCscaletbl[prqdata[1]].Mdenom >> 16) & 0xff;
//        prsdata[10] = (ADCscaletbl[prqdata[1]].Mdenom >> 24) & 0xff;
//        prsdata[11] = ADCscaletbl[prqdata[1]].B & 0xff;
//        prsdata[12] = (ADCscaletbl[prqdata[1]].B >> 8) & 0xff;
//        prsdata[13] = (ADCscaletbl[prqdata[1]].B >> 16) & 0xff;
//        prsdata[14] = (ADCscaletbl[prqdata[1]].B >> 24) & 0xff;
//        prsp->len += 13;
//        break;
//      }
//      // channel number not specified, return channel count
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = ADC_CHANNEL_CNT;
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_CUSTOM_SET_SENSOR_ALARM_MASK:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_SENSOR_ALARM_MASK\n");
//      // Command updates the alarm mask for the specified sensor.  Alarms at or below
//      // the specified threshold are masked off such that the power monitor does not
//      // see them.  IPMI sensor events for enabled thresholds are still generated.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Sensor number
//      //   2          Alarm Mask Value (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
//      // Response Format:
//      //   1          Completion Code.
//      if ((prqdata[1] >= MAX_SENSOR_CNT) || (prqdata[2] > (unsigned char) fault)) {
//        // bad sensor number or alarm mask value
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//        break;
//      }
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]] = prqdata[2];     // update alarm mask level
//      eepspi_write(&pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]], PAYLDMGR_AREA_BYTE_OFFSET +
//        (unsigned short) (&pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]]-(unsigned char*) &pyldmgr_state.nonvol_settings), 1);
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      break;
//
//    case IPMICMD_CUSTOM_GET_SENSOR_ALARM_MASK:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_SENSOR_ALARM_MASK\n");
//      // Command returns the alarm mask for the specified sensor.  Alarms at or below
//      // the specified threshold are masked off such that the power monitor does not
//      // see them.  IPMI sensor events for enabled thresholds are still generated.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Sensor number
//      // Response Format:
//      //   1          Completion Code.
//      //   2          Alarm Mask Value (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
//      if (prqdata[1] >= MAX_SENSOR_CNT) {
//        // bad sensor number
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//        break;
//      }
//      prsdata[2] = (unsigned char) pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]];
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_CUSTOM_SET_HANDLE_OVERRIDE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_HANDLE_OVERRIDE\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          [7] - 0b = normal mode, 1b = override mode
//      //              [6:1] - reserved
//      //              [0] - 0b = force handle to report closed (in) state, 1b = force
//      //              handle to report an open (out) state.  This bit is ignored
//      //              if bit 7 is set to 0.
//      //   2          Override duration (optional).  If this byte is present, it specifies
//      //              the duration of the override, in 100ms units, with zero indicating
//      //              that the override is indefinite.  Ignored if normal mode is specified.
//      // Response Format:
//      //   1          Completion Code.
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      if (!(prqdata[1] & 0x80)) {
//        // normal mode
//        clear_handle_position_override();
//        break;
//      }
//      if (preq->len > (IPMBREQOVHEAD+1))
//        xbuf[0] = prqdata[2];     // duration specified
//      else
//        xbuf[0] = 0;        // no duration specified
//      if (prqdata[1] & 0x01)
//        set_handle_position_override(out_open, xbuf[0]);
//      else
//        set_handle_position_override(in_closed, xbuf[0]);
//      break;
//
//    case IPMICMD_CUSTOM_GET_NONVOLATILE_AREA_INFO:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_NONVOLATILE_AREA_INFO\n");
//      // This returns area information for EEPROM-based nonvolatile storage
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Requested page index.  Nonvolatile area information returned
//      //              depends on which page index is specified.
//      // Response Format:
//      //   1          Completion Code.
//      // ----Response bytes for page index 0----------------------------------------
//      //   2          Format code for nonvolatile storage.  Returns 0x00 for this version
//      //   3          Requested page index supplied in command (equals 0 for this section)
//      //   4          EEPROM size in bytes, LS byte
//      //   5          EEPROM size in bytes, MS byte
//      //   6          Hardware Header Area byte offset, LS byte
//      //   7          Hardware Header Area byte offset, MS byte
//      //   8          Hardware Header Area size, 8-byte units
//      //   9          Application Device Info Area byte offset, LS byte
//      //   10         Application Device Info Area byte offset, MS byte
//      //   11         Application Device Info Area size, 8-byte units
//      //   12         FRU Data Area byte offset, LS byte
//      //   13         FRU Data Area byte offset, MS byte
//      //   14         FRU Data Area size, 8-byte units
//      //   15         FPGA Configuration Area byte offset, LS byte
//      //   16         FPGA Configuration Area byte offset, MS byte
//      //   17         FPGA Configuration Area size, 32-byte units
//      // ----Response bytes for page index 1----------------------------------------
//      //   2          Format code for nonvolatile storage.  Returns 0x00 for this version
//      //   3          Requested page index supplied in command (equals 1 for this section)
//      //   4          SDR Area byte offset, LS byte
//      //   5          SDR Area byte offset, MS byte
//      //   6          SDR Area size, 32-byte units
//      //   7          Payload Manager Area byte offset, LS byte
//      //   8          Payload Manager Area byte offset, MS byte
//      //   9          Payload Manager Area size, 8-byte units
//      //   10         ADC Scaling Factor Area byte offset, LS byte
//      //   11         ADC Scaling Factor Area byte offset, MS byte
//      //   12         ADC Scaling Factor Area size, 8-byte units
//      switch (prqdata[1]) {
//        case 0:
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//          prsdata[2] = NONVOLATILE_FORMAT_VERSION;
//          prsdata[3] = 0;
//          prsdata[4] = EEPSIZE & 0xff;
//          prsdata[5] = EEPSIZE >> 8;
//          prsdata[6] = HW_HEADER_BYTE_OFFSET & 0xff;
//          prsdata[7] = HW_HEADER_BYTE_OFFSET >> 8;
//          prsdata[8] = HW_HEADER_SIZE >> 3;
//          prsdata[9] = APP_DEV_ID_BYTE_OFFSET & 0xff;
//          prsdata[10] = APP_DEV_ID_BYTE_OFFSET >> 8;
//          prsdata[11] = APP_DEV_ID_SIZE >> 3;
//          prsdata[12] = COMMON_HEADER_BYTE_OFFSET & 0xff;
//          prsdata[13] = COMMON_HEADER_BYTE_OFFSET >> 8;
//          prsdata[14] = FRU_AREA_SIZE >> 3;
//          prsdata[15] = FPGA_CONFIG_AREA_BYTE_OFFSET & 0xff;
//          prsdata[16] = FPGA_CONFIG_AREA_BYTE_OFFSET >> 8;
//          prsdata[17] = FPGA_CONFIG_AREA_SIZE >> 5;
//          prsp->len += 16;
//          break;
//
//        case 1:
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//          prsdata[2] = NONVOLATILE_FORMAT_VERSION;
//          prsdata[3] = 1;
//          prsdata[4] = SDR_AREA_BYTE_OFFSET & 0xff;
//          prsdata[5] = SDR_AREA_BYTE_OFFSET >> 8;
//          prsdata[6] = SDR_AREA_SIZE >> 5;
//          prsdata[7] = PAYLDMGR_AREA_BYTE_OFFSET & 0xff;
//          prsdata[8] = PAYLDMGR_AREA_BYTE_OFFSET >> 8;
//          prsdata[9] = PAYLDMGR_AREA_SIZE >> 3;
//          prsdata[10] = ADC_SCALING_AREA_BYTE_OFFSET & 0xff;
//          prsdata[11] = ADC_SCALING_AREA_BYTE_OFFSET >> 8;
//          prsdata[12] = ADC_SCALING_AREA_SIZE >> 3;
//          prsp->len += 11;
//          break;
//
//        default:
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//        break;
//      }
//      break;
//
//    case IPMICMD_CUSTOM_RAW_NONVOLATILE_WRITE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_RAW_NONVOLATILE_WRITE\n");
//      // This function performs a raw write of the nonvolatile storage EEPROM.  It should
//      // be used with great care, as it cause corruption of one or more EEPROM storage areas
//      // and affect the function of the module.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Starting EEPROM byte offset, LS byte
//      //   2          Starting EEPROm byte offset, MS byte
//      //   3          Count of Bytes to write (n)
//      //   4..4+(n-1) Write Data
//      // Response Format:
//      //   1          Completion Code.
//      // first check to see if EEPROM is available
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      eepsaddr = prqdata[1] | (prqdata[2] << 8);
//      xlength = prqdata[3];
//      if (((eepsaddr+xlength-1) > EEPSIZE) || (xlength > NONVOLATILE_MAX_XFER_LEN)) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      // transfer data from command to buffer
//      curxlen = xlength;
//      xbptr = &xbuf[0];
//      pmsgdata = (unsigned char*) &prqdata[4];
//      while (curxlen) {
//        *xbptr++ = *pmsgdata++;
//        curxlen--;
//      }
//      eepspi_write(xbuf, eepsaddr, xlength);
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      break;
//
//    case IPMICMD_CUSTOM_RAW_NONVOLATILE_READ:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_RAW_NONVOLATILE_READ\n");
//      // This function performs a raw read of the nonvolatile storage EEPROM.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Starting EEPROM byte offset, LS byte
//      //   2          Starting EEPROM byte offset, MS byte
//      //   3          Count of Bytes to read (n)
//      // Response Format:
//      //   1          Completion Code.
//      //   2..2+(n-1) Read Data
//      // first check to see if EEPROM is available
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      eepsaddr = prqdata[1] | (prqdata[2] << 8);
//      xlength = prqdata[3];
//      if (((eepsaddr+xlength-1) >= EEPSIZE) || (xlength > NONVOLATILE_MAX_XFER_LEN)) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      eepspi_read(xbuf, eepsaddr, xlength);         // read from EEPROM
//      // transfer data from buffer to response msg
//      curxlen = xlength;
//      xbptr = &xbuf[0];
//      pmsgdata = (unsigned char*) &prsdata[2];
//      while (curxlen) {
//        *pmsgdata++ = *xbptr++;
//        curxlen--;
//      }
//      eepspi_write(xbuf, eepsaddr, xlength);
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsp->len += xlength;
//     break;
//
//    case IPMICMD_CUSTOM_CHK_EEPROM_BUSY:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_CHK_EEPROM_BUSY\n");
//      // Function checks the EEPROM status to determine if it is busy with a write access or not.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          Busy status--returns 0x01 if EEPROM busy, 0x00 if it is not
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = eepspi_chk_write_in_progress();
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_CUSTOM_EEPROM_ERASE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_EEPROM_ERASE\n");
//      // Command to erase EEPROM.  In order to avoid accidental erasures of the EEPROM,
//      // the command must be executed twice, each time with a different subfunction code.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Subfunction code.  Use value 0xaa on the first call to have the function
//      //              return a 4-byte erase key.  Use value 0x55 on the second call, along with
//      //              the erase key in bytes 2-5 of the request, to complete the erase operation.
//      //              Note that the erase key is valid for approximately 30 seconds after it is
//      //              issued.
//      //   2          Erase key byte 0.  Ignored on first (0xaa) call, required on second (0x55) call
//      //   3          Erase key byte 1.  Ignored on first (0xaa) call, required on second (0x55) call
//      //   4          Erase key byte 2.  Ignored on first (0xaa) call, required on second (0x55) call
//      //   5          Erase key byte 3.  Ignored on first (0xaa) call, required on second (0x55) call
//      // Response Format:
//      //   1          Completion Code.
//      //   2          Erase key byte 0.  Returned on first (0xaa) call only.
//      //   3          Erase key byte 1.  Returned on first (0xaa) call only.
//      //   4          Erase key byte 2.  Returned on first (0xaa) call only.
//      //   5          Erase key byte 3.  Returned on first (0xaa) call only.
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      switch (prqdata[1]) {
//        case EEP_ERASE_KEY_SUBFUNC:
//          // get new key
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//          EEP_erase_key.issue_time = pyldmgr_state.mmc_uptime;      // take time snapshot
//          // choose some dynamic data values to make the key
//
//          EEP_erase_key.eep_erase_key[0] =  ( LPC_TIM0->TC & 0xff );      // request msg checksum
//          EEP_erase_key.eep_erase_key[1] = pyldmgr_state.mmc_uptime & 0xff;
//          EEP_erase_key.eep_erase_key[2] = ( LPC_TIM0->TC & 0xff );
//          EEP_erase_key.eep_erase_key[3] = (get_rtc_value() >> 8) & 0xff;
//          prsdata[2] = EEP_erase_key.eep_erase_key[0];
//          prsdata[3] = EEP_erase_key.eep_erase_key[1];
//          prsdata[4] = EEP_erase_key.eep_erase_key[2];
//          prsdata[5] = EEP_erase_key.eep_erase_key[3];
//          prsp->len += 4;
//          break;
//
//        case EEP_ERASE_EXECUTE_SUBFUNC:
//          // check key to make sure it is accurate and unexpired
//          if (((pyldmgr_state.mmc_uptime - EEP_erase_key.issue_time) > EEP_KEY_VALID_DURATION) ||
//            (prqdata[2] != EEP_erase_key.eep_erase_key[0]) || (prqdata[3] != EEP_erase_key.eep_erase_key[1]) ||
//            (prqdata[4] != EEP_erase_key.eep_erase_key[2]) || (prqdata[5] != EEP_erase_key.eep_erase_key[3])) {
//            // time expired or bad key
//            IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INSUFFICIENT_PRIVILEGE;
//            break;
//          }
//          // perform erase
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//          eepspi_chip_erase();
//          break;
//
//        default:
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//          break;
//      }
//      break;
//
//
//    case IPMICMD_CUSTOM_GET_TIMESTATS:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_TIMESTATS\n");
//      // Function returns the timer values for MMC up time and Back end hot time, in seconds
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          32-bit mmc up time, LS byte
//      //   3          32-bit mmc up time, bits [15:8
//      //   4          32-bit mmc up time, bits [23:16]
//      //   5          32-bit mmc up time, MS byte
//      //   6          32-bit backend hot time, LS byte
//      //   7          32-bit backend hot time, bits [15:8
//      //   8          32-bit backend hot time, bits [23:16]
//      //   9          32-bit backend hot time, MS byte
//      systime = get_rtc_value();
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = pyldmgr_state.mmc_uptime & 0xff;
//      prsdata[3] = (pyldmgr_state.mmc_uptime >> 8) & 0xff;
//      prsdata[4] = (pyldmgr_state.mmc_uptime >> 16) & 0xff;
//      prsdata[5] = (pyldmgr_state.mmc_uptime >> 24) & 0xff;
//      prsdata[6] = pyldmgr_state.bkend_hottime & 0xff;
//      prsdata[7] = (pyldmgr_state.bkend_hottime >> 8) & 0xff;
//      prsdata[8] = (pyldmgr_state.bkend_hottime >> 16) & 0xff;
//      prsdata[9] = (pyldmgr_state.bkend_hottime >> 24) & 0xff;
//      prsp->len += 8;
//
//      break;
//
//    case IPMICMD_CUSTOM_SET_SYSTIME:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_SYSTIME\n");
//      // This function sets the 32-bit system time kept by the RTC clock and the 32KHz oscillator
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          32-bit system time, LS byte
//      //   2          32-bit system time, bits [15:8
//      //   3          32-bit system time, bits [23:16]
//      //   4          32-bit system time, MS byte
//      // Response Format:
//      //   1          Completion Code.
//      if (preq->len < IPMBREQOVHEAD+4) {
//        // not enough params
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      systime = prqdata[4];
//      systime = (systime << 8) + prqdata[3];
//      systime = (systime << 8) + prqdata[2];
//      systime = (systime << 8) + prqdata[1];
//      set_rtc_value(systime);
//      break;
//
//    case IPMICMD_CUSTOM_GET_SYSTIME:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_SYSTIME\n");
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.
//      //   2          32-bit system time, LS byte
//      //   3          32-bit system time, bits [15:8
//      //   4          32-bit system time, bits [23:16]
//      //   5          32-bit system time, MS byte
//      systime = get_rtc_value();
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = systime & 0xff;
//      prsdata[3] = (systime >> 8) & 0xff;
//      prsdata[4] = (systime >> 16) & 0xff;
//      prsdata[5] = (systime >> 24) & 0xff;
//      prsp->len += 4;
//      break;
//
//    case IPMICMD_CUSTOM_POLL_FPGA_CFG_PORT:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_POLL_FPGA_CFG_PORT\n");
//      // This command polls the SPI1 port for connected FPGA Config Port SPI slaves.  It
//      // asserts the ~SCANSLV signal from the Microcontroller and samples the ~CSx pins
//      // as logic inputs.  Any ~CSx lines observed to go to a logic 0 are considered to
//      // have FPGA Config Ports attached to them.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      //   2          [7:4] - reserved, returns 1111b
//      //              [3] - SPI1 CS3 -- 1b = FPGA port detected, 0b = no FPGA detected
//      //              [2] - SPI1 CS2 -- 1b = FPGA port detected, 0b = no FPGA detected
//      //              [1] - SPI1 CS1 -- 1b = FPGA port detected, 0b = no FPGA detected
//      //              [0] - SPI1 CS0 -- 1b = FPGA port detected, 0b = no FPGA detected
//      if (!spi1_lock()) {
//        // SPI1 unavailable
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = 0xf0 | fpgaspi_slave_detect();
//      spi1_unlock();
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_CUSTOM_FPGA_CFG_RDSR:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_RDSR\n");
//      // BYTE         DATA FIELD
//      // This command reads the FPGA Config Port Status register using the RDSR SPI command
//      // Command Format:
//      //   1          SPI1 Chip Select ID (0-3)
//      // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      //   2          FPGA SPI Configuration Port Status Register
//      //              [7] - Upper Handshake Flag (UHF)
//      //              [6] - Lower Handshake Flag (LHF)
//      //              [5] - Config Ready Flag (CFGRDY)-- reset to 0b by FPGA firmware @ startup,
//      //                    set to 1b after config image written to port
//      //              [4] - Request Config Flag (REQCFG) -- set to 1b by FPGA at init to request
//      //                    config data, cleared by FPGA after detecting CFGRDY set to 1b by SPI write
//      //              [3:0] - reserved
//      if (prqdata[1] > MAX_FPGA_ID) {
//        // bad param
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      if (!spi1_lock()) {
//        // SPI1 unavailable
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = fpgaspi_status_read(prqdata[1]);
//      spi1_unlock();
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_CUSTOM_FPGA_CFG_WRCTL:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_WRCTL\n");
//      // This function writes a byte to the FPGA SPI Config Port Control register
//      // using the WRCTL SPI command.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          SPI1 Chip Select ID (0-3)
//      //   2          Data Byte Written to Control Register
//      //              [7] - 1b = select UHF, 0b = no select
//      //              [6] - 1b = select LHF, 0b = no select
//      //              [5] - 1b = select CFGRDY, 0b = no select
//      //              [4:2] - reserved
//      //              [1] - 1b = selected bits 7:5 should be set, 0b = no change
//      //              [0] - 1b = selected bits 7:5 should be cleared, 0b = no change
//      // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      if (prqdata[1] > MAX_FPGA_ID) {
//        // bad param
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      if (!spi1_lock()) {
//        // SPI1 unavailable
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      fpgaspi_ctl_write(prqdata[1], prqdata[2]);
//      spi1_unlock();
//      break;
//
//    case IPMICMD_CUSTOM_FPGA_CFG_WRITE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_WRITE\n");
//      // This function performs a raw write to the FPGA SPI Config Port.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          SPI1 Chip Select ID (0-3)
//      //   2          Config Port Destination byte address, LS byte
//      //   3          Config Port Destination byte address, MS byte
//      //   4          Count of Bytes to write (n)
//      //   5..5+(n-1) Write Data
//      // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      if ((prqdata[1] > MAX_FPGA_ID) || (prqdata[4] > NONVOLATILE_MAX_XFER_LEN)) {
//        // bad param
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      if (!spi1_lock()) {
//        // SPI1 unavailable
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      eepsaddr = prqdata[2] | (prqdata[3] << 8);
//      // transfer bytes to SPI port
//      fpgaspi_data_write(prqdata[1], &prqdata[5], eepsaddr, prqdata[4]);
//      spi1_unlock();
//      break;
//
//    case IPMICMD_CUSTOM_FPGA_CFG_READ:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_READ\n");
//      // This function performs a raw read from the FPGA SPI Config Port.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          SPI1 Chip Select ID (0-3)
//      //   2          Config Port Source byte address, LS byte
//      //   3          Config Port Source byte address, MS byte
//      //   4          Count of Bytes to read (n)
//       // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      //   2..2+(n-1) Read Data
//      if ((prqdata[1] > MAX_FPGA_ID) || (prqdata[4] > NONVOLATILE_MAX_XFER_LEN)) {
//        // bad param
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
//        break;
//      }
//      if (!spi1_lock()) {
//        // SPI1 unavailable
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      eepsaddr = prqdata[2] | (prqdata[3] << 8);
//      fpgaspi_data_read(prqdata[1], &prsdata[2], eepsaddr, prqdata[4]);
//      prsp->len += prqdata[4];
//      spi1_unlock();
//      break;
//
//    case IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_WRITE:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_WRITE\n");
//      // This function writes the header for the 256 byte Autoconfig section
//      // in the nonvolatile storage (EEPROM).  NOTE:  Any changes to the header
//      // or Autoconfig section of nonvolatile storage will not take effect until
//      // the MMC is reset.
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   1          Config Flags, 2 bits per chip select.  (x0b = config data undefined,
//      //              01b = defined, but autoconfig disabled, 11b = autoconfig enabled)
//      //              [7:6] CS3 autoconfig setting
//      //              [5:4] CS2 autoconfig setting
//      //              [3:2] CS1 autoconfig setting
//      //              [1:0] CS0 autoconfig setting
//      //   2          Byte offset to CS0 config data
//      //   3          Byte offset to CS1 config data
//      //   4          Byte offset to CS2 config data
//      //   5          Byte offset to CS3 config data
//      //   6          Header Checksum
//      // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      eepsaddr = FPGA_CONFIG_AREA_BYTE_OFFSET+1;      // skip past format flag
//      eepspi_write(&prqdata[1], eepsaddr, sizeof(FPGA_config_area_header_t)-1);
//      break;
//
//    case IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_READ:
//      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_READ\n");
//      // This function reads and returns the header for the 256 byte Autoconfig section
//      // in the nonvolatile storage (EEPROM)
//      // BYTE         DATA FIELD
//      // Command Format:
//      //   (no bytes)
//      // Response Format:
//      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
//      //   2          Config Flags, 2 bits per chip select.  (x0b = config data undefined,
//      //              01b = defined, but autoconfig disabled, 11b = autoconfig enabled)
//      //              [7:6] CS3 autoconfig setting
//      //              [5:4] CS2 autoconfig setting
//      //              [3:2] CS1 autoconfig setting
//      //              [1:0] CS0 autoconfig setting
//      //   3          Byte offset to CS0 config data
//      //   4          Byte offset to CS1 config data
//      //   5          Byte offset to CS2 config data
//      //   6          Byte offset to CS3 config data
//      //   7          Header Checksum
//      if (eepspi_chk_write_in_progress()) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      eepsaddr = FPGA_CONFIG_AREA_BYTE_OFFSET+1;      // skip past format flag
//      eepspi_read(&prsdata[2], eepsaddr, sizeof(FPGA_config_area_header_t)-1);
//      prsp->len += sizeof(FPGA_config_area_header_t)-1;
//      break;
//
//    default:
//      prsp->len = 0;      // nothing supported here yet
//  }
//}




////static void parse_IPMI_cmds( const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp )
////{
////  const unsigned char* prqdata = &preq->buf[IPMI_RQ_DATA_OFFSET-1];   // offset pointer for 1-base refs to req data
////  unsigned char* prsdata = &prsp->buf[IPMI_RS_DATA_OFFSET-1];     // offset pointer for 1-base refs to rsp data
////  unsigned char xbuf[NONVOLATILE_MAX_XFER_LEN];
////  unsigned char* xbptr;
////  unsigned char* pmsgdata;
////  unsigned char xlength, curxlen;
////  unsigned short eepsaddr;
////  unsigned long systime;
////  unsigned short fruoffset;
////  unsigned char datalen;
////  unsigned short rsvID, recordID;
////  unsigned char* pSDRdata;
////  unsigned char* prsbyte;
////  unsigned char rdlength;
////
////  int i1;
////
////  App_Device_ID_record_t devIDrec;
////
////  LED_activity_desc_t *pLEDact;
////  const LED_state_rec_t* pLEDstate;
////
////  SDR_entry_hdr_t* pSDRhdr;
////  SDR_type_01h_t* pSensorSDR;
////  module_current_req_record_t currentrecbuf;
////
////
////  switch (IPMI_RQ_CMD(preq->buf)) {
////    case IPMICMD_CUSTOM_SET_BACKEND_PWR:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_BACKEND_PWR\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          backend on-off power enable.  Set to 1 to enable
////      //              backend power, 0 to disable.  Returns completion
////      //              code 0xcc if field has a different value
////      // Response Format:
////      //   1          Completion Code.
////      //if ((*prqdata) > 1) {
////      if ((*prqdata) > 1) {
////        // value out of range
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      pyldmgr_ipmicmd_backend_pwr(preq);
////      break;
////
////    case IPMICMD_CUSTOM_GET_BACKEND_PWR:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_BACKEND_PWR\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          Backend Power Setting (1=enabled, 0=disabled)
////      //   3          Backend Power Pin State (1=enabled, 0=disabled)
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = pyldmgr_state.bkend_pwr_ena;
////      prsdata[3] = get_backend_power_pin;
////      prsp->len += 2;
////      break;
////    case IPMICMD_CUSTOM_SET_PYLD_MGR_SETTING_REC:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_PYLD_MGR_SETTING_REC\n");
////      // Sets control field values for Payload Manager.  Settings are non-volatile.  Fields
////      // not being updated are ignored but should have data bytes present in the command.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Field select mask.  Bit=1 selects associated field for update, bit=0
////      //              leaves field unchanged.  Bit definitions as follows:
////      //              [7] - Default backend auto-power enable
////      //              [6] - Default boot mode
////      //              [5] - FPGA auto-configuration inhibit
////      //              [4] - Backend power shutdown event-enable
////      //              [3] - Backend Monitor global alarm level
////      //              [2:0] - Unused, reserved
////      //   2          Default backend auto-power enable setting (1=enable backend pwr when
////      //              payload power turns on, 0=disable backend power)
////      //   3          Default boot mode (0=trigger mode, 1=maintenance mode)
////      //   4          FPGA auto-configuration inhibit setting (1=inhibit FPGA-auto-configuration
////      //              function, 0=permit FPGA-auto-configuration)
////      //   5          Backend power shutdown event-enable setting (1=send hotswap event if
////      //              backend power is commanded to shut down, 0=do not send event)
////      //   6          Backend Monitor global alarm mask level (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
////      // Response Format:
////      //   1          Completion Code.
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      if (!prqdata[1])
////        break;      // nothing to write
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////       if (preq->len < IPMBREQOVHEAD+6) {
////        // not enough params
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
////        break;
////      }
////      // examine each select bit/field
////      if (prqdata[1] & 0x80) {
////        if (prqdata[2] > 1) {
////          // value out of range
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////          break;
////        }
////        pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena = prqdata[2];
////      }
////      if (prqdata[1] & 0x40) {
////        if (prqdata[3] > 1) {
////          // value out of range
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////          break;
////        }
////        pyldmgr_state.nonvol_settings.default_boot_mode = prqdata[3];
////      }
////      if (prqdata[1] & 0x20) {
////        if (prqdata[4] > 1) {
////          // value out of range
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////          break;
////        }
////        pyldmgr_state.nonvol_settings.fpga_auto_cfg_inhibit = prqdata[4];
////      }
////      if (prqdata[1] & 0x10) {
////        if (prqdata[5] > 1) {
////          // value out of range
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////          break;
////        }
////        pyldmgr_state.nonvol_settings.bkend_pwr_shtdn_evt_ena = prqdata[5];
////      }
////      if (prqdata[1] & 0x08) {
////        if (prqdata[6] > 3) {
////          // value out of range
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////          break;
////        }
////        pyldmgr_state.nonvol_settings.global_mask_level = prqdata[6];
////      }
////      eepspi_write((unsigned char*) &pyldmgr_state.nonvol_settings, PAYLDMGR_AREA_BYTE_OFFSET, 8);
////      break;
////
////    case IPMICMD_CUSTOM_GET_PYLD_MGR_SETTING_REC:
////
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_PYLD_MGR_SETTING_REC\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          Default backend auto-power enable setting (1=enable backend pwr when
////      //              payload power turns on, 0=disable backend power)
////      //   3          Default boot mode (0=trigger mode, 1=maintenance mode)
////      //   4          FPGA auto-configuration inhibit setting (1=inhibit FPGA-auto-configuration
////      //              function, 0=permit FPGA-auto-configuration)
////      //   5          Backend power shutdown event-enable setting (1=send hotswap event if
////      //              backend power is commanded to shut down, 0=do not send event)
////      //   6          Backend Monitor global alarm mask level (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = pyldmgr_state.nonvol_settings.default_bkend_auto_pwr_ena;
////      prsdata[3] = pyldmgr_state.nonvol_settings.default_boot_mode;
////      prsdata[4] = pyldmgr_state.nonvol_settings.fpga_auto_cfg_inhibit;
////      prsdata[5] = pyldmgr_state.nonvol_settings.bkend_pwr_shtdn_evt_ena;
////      prsdata[6] = pyldmgr_state.nonvol_settings.global_mask_level;
////      prsp->len += 5;
////      break;
////
////    case IPMICMD_CUSTOM_GET_FAULT_STATUS:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_FAULT_STATUS\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          alarm severity level (0=normal, 1=noncritical, 2=critical, 3=nonrecoverable)
////      //   3          alarm source (0=none, 1=temperature sensor, 2=voltage sensor, 3=backend device)
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = (unsigned char) pyldmgr_state.bkend_monitor.cur_alarm_level;
////      prsdata[3] = (unsigned char) pyldmgr_state.bkend_monitor.cur_alarm_source;
////      prsp->len += 2;
////      break;
////
////    case IPMICMD_CUSTOM_SET_BOOT_MODE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_BOOT_MODE\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Boot mode.  Set to 0 for Trigger Mode and 1 for Maintenance mode
////      // Response Format:
////      //   1          Completion Code.
////      if (prqdata[1] > 1) {
////        // illegal mode
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////        break;
////      }
////      set_boot_mode_pin(prqdata[1]);
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      break;
////
////    case IPMICMD_CUSTOM_GET_BOOT_MODE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_BOOT_MODE\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          Current Boot Mode.  Returns 0 for Trigger Mode and 1 for Maintenance mode
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = get_boot_mode_pin;
////      prsp->len += 1;
////      break;
////
////    case IPMICMD_CUSTOM_SET_CURRENT_REQUIREMENT:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_CURRENT_REQUIREMENT\n");
////      // Command updates the current requirement record in the FRU multirecord area in
////      // nonvolatile storage.  The change will take effect the next time the FRU data area
////      // is read by the Carrier Manager.  Current is given in 100 mA units, to a maximum of
////      // 7.0 amps, with a minimum of 0.5A
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          12V Current Requirement in 100 mA units
////      // Response Format:
////      //   1          Completion Code.
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      if ((prqdata[1] < 5) || (prqdata[1] > 70)) {
////        // bad current specification
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////        break;
////      }
////      // load current record into memory and inspect it
////      eepspi_read((unsigned char*) &currentrecbuf, MULTIRECORD_AREA_BYTE_OFFSET, sizeof(currentrecbuf));
////      if ((currentrecbuf.PICMG_recID != 0x16) || (currentrecbuf.mfgID_LSB != 0x5a) || (currentrecbuf.mfgID_MidB != 0x31) ||
////        (currentrecbuf.mfgID_MSB != 0x00)) {
////        // something wrong, but this isn't the current requirement record--so abort
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
////        break;
////      }
////      currentrecbuf.current_draw_100mA = prqdata[1];    // set new current requirement
////      currentrecbuf.hdr.recxsum = calc_ipmi_xsum((unsigned char*) &(currentrecbuf.mfgID_LSB), sizeof(module_current_req_record_t)-
////        sizeof(multirecord_header_t));                  // update checksum
////      eepspi_write((unsigned char*) &currentrecbuf, MULTIRECORD_AREA_BYTE_OFFSET, sizeof(currentrecbuf));
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      break;
////
////    case IPMICMD_CUSTOM_SET_ANALOG_SCALE_FACTOR:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_ANALOG_SCALE_FACTOR\n");
////      // Command sets the scaling factors for an ADC input channel in nonvolatile storage.
////      // For changes to be observed in sensor operation, the MMC must be restarted by a power
////      // cycle or IPMI Application Cold Reset command (netFN=06h, Cmd=02h).
////      // BYTE         DATA FIELD
////      //   1          ADC channel number.  If omitted, command returns the number of ADC channels.
////      //              Channels are numbered consecutively, beginning at 0.
////      //   2          Field index.  Fields are indexed as follows:
////      //                0 - Scaling factor M numerator
////      //                1 - Scaling factor M denominator
////      //                2 - Offset B
////      //   3-6        Field value, as 32-bit signed integer, LS byte first
////      // Command Format:
////      //   1          Completion Code.
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      if ((prqdata[1] > MAX_ADC_CH_NUM) || (prqdata[2] > 2)) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////        break;
////      }
////      // compute EEPROM address
////      eepsaddr = ADC_SCALING_AREA_BYTE_OFFSET + prqdata[1]*sizeof(front_end_scaling_factors_t) + prqdata[2]*sizeof(long);
////      // use 'systime' as a scratch pad variable to build a 32-bit version of the new field value
////      systime = prqdata[6];
////      systime = (systime << 8) + prqdata[5];
////      systime = (systime << 8) + prqdata[4];
////      systime = (systime << 8) + prqdata[3];
////      eepspi_write((unsigned char*) &systime, eepsaddr, 4);
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      break;
////
////    case IPMICMD_CUSTOM_GET_ANALOG_SCALE_FACTOR:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_ANALOG_SCALE_FACTOR\n");
////      // Command returns the scaling factors for an ADC input channel.  If no arguments are
////      // specified, it returns the number of ADC channels.
////      // BYTE         DATA FIELD
////      //   1          ADC channel number.  If omitted, command returns the number of ADC channels.
////      //              Channels are numbered consecutively, beginning at 0.
////      // Command Format:
////      //   1          Completion Code.
////      //   2          Returns the ADC channel number specified in the command.  If omitted from
////      //   3-6        Scaling factor (M) numerator, as 32-bit signed integer, LS byte first.
////      //   7-10       Scaling factor (M) denominator, as 32-bit signed integer, LS byte first
////      //   11-14      Offset (B), as 32-bit signed integer, LS byte first.
////      if (preq->len > (IPMBREQOVHEAD)) {
////        // channel number specified
////        if (prqdata[1] > MAX_ADC_CH_NUM) {
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////          break;
////        }
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////        prsdata[2] = prqdata[1];
////        prsdata[3] = ADCscaletbl[prqdata[1]].Mnum & 0xff;
////        prsdata[4] = (ADCscaletbl[prqdata[1]].Mnum >> 8) & 0xff;
////        prsdata[5] = (ADCscaletbl[prqdata[1]].Mnum >> 16) & 0xff;
////        prsdata[6] = (ADCscaletbl[prqdata[1]].Mnum >> 24) & 0xff;
////        prsdata[7] = ADCscaletbl[prqdata[1]].Mdenom & 0xff;
////        prsdata[8] = (ADCscaletbl[prqdata[1]].Mdenom >> 8) & 0xff;
////        prsdata[9] = (ADCscaletbl[prqdata[1]].Mdenom >> 16) & 0xff;
////        prsdata[10] = (ADCscaletbl[prqdata[1]].Mdenom >> 24) & 0xff;
////        prsdata[11] = ADCscaletbl[prqdata[1]].B & 0xff;
////        prsdata[12] = (ADCscaletbl[prqdata[1]].B >> 8) & 0xff;
////        prsdata[13] = (ADCscaletbl[prqdata[1]].B >> 16) & 0xff;
////        prsdata[14] = (ADCscaletbl[prqdata[1]].B >> 24) & 0xff;
////        prsp->len += 13;
////        break;
////      }
////      // channel number not specified, return channel count
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = ADC_CHANNEL_CNT;
////      prsp->len += 1;
////      break;
////
////    case IPMICMD_CUSTOM_SET_SENSOR_ALARM_MASK:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_SENSOR_ALARM_MASK\n");
////      // Command updates the alarm mask for the specified sensor.  Alarms at or below
////      // the specified threshold are masked off such that the power monitor does not
////      // see them.  IPMI sensor events for enabled thresholds are still generated.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Sensor number
////      //   2          Alarm Mask Value (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
////      // Response Format:
////      //   1          Completion Code.
////      if ((prqdata[1] >= MAX_SENSOR_CNT) || (prqdata[2] > (unsigned char) fault)) {
////        // bad sensor number or alarm mask value
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////        break;
////      }
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]] = prqdata[2];     // update alarm mask level
////      eepspi_write(&pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]], PAYLDMGR_AREA_BYTE_OFFSET +
////        (unsigned short) (&pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]]-(unsigned char*) &pyldmgr_state.nonvol_settings), 1);
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      break;
////
////    case IPMICMD_CUSTOM_GET_SENSOR_ALARM_MASK:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_SENSOR_ALARM_MASK\n");
////      // Command returns the alarm mask for the specified sensor.  Alarms at or below
////      // the specified threshold are masked off such that the power monitor does not
////      // see them.  IPMI sensor events for enabled thresholds are still generated.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Sensor number
////      // Response Format:
////      //   1          Completion Code.
////      //   2          Alarm Mask Value (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
////      if (prqdata[1] >= MAX_SENSOR_CNT) {
////        // bad sensor number
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////        break;
////      }
////      prsdata[2] = (unsigned char) pyldmgr_state.nonvol_settings.sensor_mask_level[prqdata[1]];
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsp->len += 1;
////      break;
////
////    case IPMICMD_CUSTOM_SET_HANDLE_OVERRIDE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_HANDLE_OVERRIDE\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          [7] - 0b = normal mode, 1b = override mode
////      //              [6:1] - reserved
////      //              [0] - 0b = force handle to report closed (in) state, 1b = force
////      //              handle to report an open (out) state.  This bit is ignored
////      //              if bit 7 is set to 0.
////      //   2          Override duration (optional).  If this byte is present, it specifies
////      //              the duration of the override, in 100ms units, with zero indicating
////      //              that the override is indefinite.  Ignored if normal mode is specified.
////      // Response Format:
////      //   1          Completion Code.
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      if (!(prqdata[1] & 0x80)) {
////        // normal mode
////        clear_handle_position_override();
////        break;
////      }
////      if (preq->len > (IPMBREQOVHEAD+1))
////        xbuf[0] = prqdata[2];     // duration specified
////      else
////        xbuf[0] = 0;        // no duration specified
////      if (prqdata[1] & 0x01)
////        set_handle_position_override(out_open, xbuf[0]);
////      else
////        set_handle_position_override(in_closed, xbuf[0]);
////      break;
////
////    case IPMICMD_CUSTOM_GET_NONVOLATILE_AREA_INFO:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_NONVOLATILE_AREA_INFO\n");
////      // This returns area information for EEPROM-based nonvolatile storage
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Requested page index.  Nonvolatile area information returned
////      //              depends on which page index is specified.
////      // Response Format:
////      //   1          Completion Code.
////      // ----Response bytes for page index 0----------------------------------------
////      //   2          Format code for nonvolatile storage.  Returns 0x00 for this version
////      //   3          Requested page index supplied in command (equals 0 for this section)
////      //   4          EEPROM size in bytes, LS byte
////      //   5          EEPROM size in bytes, MS byte
////      //   6          Hardware Header Area byte offset, LS byte
////      //   7          Hardware Header Area byte offset, MS byte
////      //   8          Hardware Header Area size, 8-byte units
////      //   9          Application Device Info Area byte offset, LS byte
////      //   10         Application Device Info Area byte offset, MS byte
////      //   11         Application Device Info Area size, 8-byte units
////      //   12         FRU Data Area byte offset, LS byte
////      //   13         FRU Data Area byte offset, MS byte
////      //   14         FRU Data Area size, 8-byte units
////      //   15         FPGA Configuration Area byte offset, LS byte
////      //   16         FPGA Configuration Area byte offset, MS byte
////      //   17         FPGA Configuration Area size, 32-byte units
////      // ----Response bytes for page index 1----------------------------------------
////      //   2          Format code for nonvolatile storage.  Returns 0x00 for this version
////      //   3          Requested page index supplied in command (equals 1 for this section)
////      //   4          SDR Area byte offset, LS byte
////      //   5          SDR Area byte offset, MS byte
////      //   6          SDR Area size, 32-byte units
////      //   7          Payload Manager Area byte offset, LS byte
////      //   8          Payload Manager Area byte offset, MS byte
////      //   9          Payload Manager Area size, 8-byte units
////      //   10         ADC Scaling Factor Area byte offset, LS byte
////      //   11         ADC Scaling Factor Area byte offset, MS byte
////      //   12         ADC Scaling Factor Area size, 8-byte units
////      switch (prqdata[1]) {
////        case 0:
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////          prsdata[2] = NONVOLATILE_FORMAT_VERSION;
////          prsdata[3] = 0;
////          prsdata[4] = EEPSIZE & 0xff;
////          prsdata[5] = EEPSIZE >> 8;
////          prsdata[6] = HW_HEADER_BYTE_OFFSET & 0xff;
////          prsdata[7] = HW_HEADER_BYTE_OFFSET >> 8;
////          prsdata[8] = HW_HEADER_SIZE >> 3;
////          prsdata[9] = APP_DEV_ID_BYTE_OFFSET & 0xff;
////          prsdata[10] = APP_DEV_ID_BYTE_OFFSET >> 8;
////          prsdata[11] = APP_DEV_ID_SIZE >> 3;
////          prsdata[12] = COMMON_HEADER_BYTE_OFFSET & 0xff;
////          prsdata[13] = COMMON_HEADER_BYTE_OFFSET >> 8;
////          prsdata[14] = FRU_AREA_SIZE >> 3;
////          prsdata[15] = FPGA_CONFIG_AREA_BYTE_OFFSET & 0xff;
////          prsdata[16] = FPGA_CONFIG_AREA_BYTE_OFFSET >> 8;
////          prsdata[17] = FPGA_CONFIG_AREA_SIZE >> 5;
////          prsp->len += 16;
////          break;
////
////        case 1:
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////          prsdata[2] = NONVOLATILE_FORMAT_VERSION;
////          prsdata[3] = 1;
////          prsdata[4] = SDR_AREA_BYTE_OFFSET & 0xff;
////          prsdata[5] = SDR_AREA_BYTE_OFFSET >> 8;
////          prsdata[6] = SDR_AREA_SIZE >> 5;
////          prsdata[7] = PAYLDMGR_AREA_BYTE_OFFSET & 0xff;
////          prsdata[8] = PAYLDMGR_AREA_BYTE_OFFSET >> 8;
////          prsdata[9] = PAYLDMGR_AREA_SIZE >> 3;
////          prsdata[10] = ADC_SCALING_AREA_BYTE_OFFSET & 0xff;
////          prsdata[11] = ADC_SCALING_AREA_BYTE_OFFSET >> 8;
////          prsdata[12] = ADC_SCALING_AREA_SIZE >> 3;
////          prsp->len += 11;
////          break;
////
////        default:
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
////        break;
////      }
////      break;
////
////    case IPMICMD_CUSTOM_RAW_NONVOLATILE_WRITE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_RAW_NONVOLATILE_WRITE\n");
////      // This function performs a raw write of the nonvolatile storage EEPROM.  It should
////      // be used with great care, as it cause corruption of one or more EEPROM storage areas
////      // and affect the function of the module.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Starting EEPROM byte offset, LS byte
////      //   2          Starting EEPROm byte offset, MS byte
////      //   3          Count of Bytes to write (n)
////      //   4..4+(n-1) Write Data
////      // Response Format:
////      //   1          Completion Code.
////      // first check to see if EEPROM is available
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      eepsaddr = prqdata[1] | (prqdata[2] << 8);
////      xlength = prqdata[3];
////      if (((eepsaddr+xlength-1) > EEPSIZE) || (xlength > NONVOLATILE_MAX_XFER_LEN)) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      // transfer data from command to buffer
////      curxlen = xlength;
////      xbptr = &xbuf[0];
////      pmsgdata = (unsigned char*) &prqdata[4];
////      while (curxlen) {
////        *xbptr++ = *pmsgdata++;
////        curxlen--;
////      }
////      eepspi_write(xbuf, eepsaddr, xlength);
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      break;
////
////    case IPMICMD_CUSTOM_RAW_NONVOLATILE_READ:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_RAW_NONVOLATILE_READ\n");
////      // This function performs a raw read of the nonvolatile storage EEPROM.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Starting EEPROM byte offset, LS byte
////      //   2          Starting EEPROM byte offset, MS byte
////      //   3          Count of Bytes to read (n)
////      // Response Format:
////      //   1          Completion Code.
////      //   2..2+(n-1) Read Data
////      // first check to see if EEPROM is available
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      eepsaddr = prqdata[1] | (prqdata[2] << 8);
////      xlength = prqdata[3];
////      if (((eepsaddr+xlength-1) >= EEPSIZE) || (xlength > NONVOLATILE_MAX_XFER_LEN)) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      eepspi_read(xbuf, eepsaddr, xlength);         // read from EEPROM
////      // transfer data from buffer to response msg
////      curxlen = xlength;
////      xbptr = &xbuf[0];
////      pmsgdata = (unsigned char*) &prsdata[2];
////      while (curxlen) {
////        *pmsgdata++ = *xbptr++;
////        curxlen--;
////      }
////      eepspi_write(xbuf, eepsaddr, xlength);
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsp->len += xlength;
////     break;
////
////    case IPMICMD_CUSTOM_CHK_EEPROM_BUSY:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_CHK_EEPROM_BUSY\n");
////      // Function checks the EEPROM status to determine if it is busy with a write access or not.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          Busy status--returns 0x01 if EEPROM busy, 0x00 if it is not
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = eepspi_chk_write_in_progress();
////      prsp->len += 1;
////      break;
////
////    case IPMICMD_CUSTOM_EEPROM_ERASE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_EEPROM_ERASE\n");
////      // Command to erase EEPROM.  In order to avoid accidental erasures of the EEPROM,
////      // the command must be executed twice, each time with a different subfunction code.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Subfunction code.  Use value 0xaa on the first call to have the function
////      //              return a 4-byte erase key.  Use value 0x55 on the second call, along with
////      //              the erase key in bytes 2-5 of the request, to complete the erase operation.
////      //              Note that the erase key is valid for approximately 30 seconds after it is
////      //              issued.
////      //   2          Erase key byte 0.  Ignored on first (0xaa) call, required on second (0x55) call
////      //   3          Erase key byte 1.  Ignored on first (0xaa) call, required on second (0x55) call
////      //   4          Erase key byte 2.  Ignored on first (0xaa) call, required on second (0x55) call
////      //   5          Erase key byte 3.  Ignored on first (0xaa) call, required on second (0x55) call
////      // Response Format:
////      //   1          Completion Code.
////      //   2          Erase key byte 0.  Returned on first (0xaa) call only.
////      //   3          Erase key byte 1.  Returned on first (0xaa) call only.
////      //   4          Erase key byte 2.  Returned on first (0xaa) call only.
////      //   5          Erase key byte 3.  Returned on first (0xaa) call only.
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      switch (prqdata[1]) {
////        case EEP_ERASE_KEY_SUBFUNC:
////          // get new key
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////          EEP_erase_key.issue_time = pyldmgr_state.mmc_uptime;      // take time snapshot
////          // choose some dynamic data values to make the key
////
////          EEP_erase_key.eep_erase_key[0] =  ( LPC_TIM0->TC & 0xff );      // request msg checksum
////          EEP_erase_key.eep_erase_key[1] = pyldmgr_state.mmc_uptime & 0xff;
////          EEP_erase_key.eep_erase_key[2] = ( LPC_TIM0->TC & 0xff );
////          EEP_erase_key.eep_erase_key[3] = (get_rtc_value() >> 8) & 0xff;
////          prsdata[2] = EEP_erase_key.eep_erase_key[0];
////          prsdata[3] = EEP_erase_key.eep_erase_key[1];
////          prsdata[4] = EEP_erase_key.eep_erase_key[2];
////          prsdata[5] = EEP_erase_key.eep_erase_key[3];
////          prsp->len += 4;
////          break;
////
////        case EEP_ERASE_EXECUTE_SUBFUNC:
////          // check key to make sure it is accurate and unexpired
////          if (((pyldmgr_state.mmc_uptime - EEP_erase_key.issue_time) > EEP_KEY_VALID_DURATION) ||
////            (prqdata[2] != EEP_erase_key.eep_erase_key[0]) || (prqdata[3] != EEP_erase_key.eep_erase_key[1]) ||
////            (prqdata[4] != EEP_erase_key.eep_erase_key[2]) || (prqdata[5] != EEP_erase_key.eep_erase_key[3])) {
////            // time expired or bad key
////            IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INSUFFICIENT_PRIVILEGE;
////            break;
////          }
////          // perform erase
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////          eepspi_chip_erase();
////          break;
////
////        default:
////          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////          break;
////      }
////      break;
////
////
////    case IPMICMD_CUSTOM_GET_TIMESTATS:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_TIMESTATS\n");
////      // Function returns the timer values for MMC up time and Back end hot time, in seconds
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          32-bit mmc up time, LS byte
////      //   3          32-bit mmc up time, bits [15:8
////      //   4          32-bit mmc up time, bits [23:16]
////      //   5          32-bit mmc up time, MS byte
////      //   6          32-bit backend hot time, LS byte
////      //   7          32-bit backend hot time, bits [15:8
////      //   8          32-bit backend hot time, bits [23:16]
////      //   9          32-bit backend hot time, MS byte
////      systime = get_rtc_value();
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = pyldmgr_state.mmc_uptime & 0xff;
////      prsdata[3] = (pyldmgr_state.mmc_uptime >> 8) & 0xff;
////      prsdata[4] = (pyldmgr_state.mmc_uptime >> 16) & 0xff;
////      prsdata[5] = (pyldmgr_state.mmc_uptime >> 24) & 0xff;
////      prsdata[6] = pyldmgr_state.bkend_hottime & 0xff;
////      prsdata[7] = (pyldmgr_state.bkend_hottime >> 8) & 0xff;
////      prsdata[8] = (pyldmgr_state.bkend_hottime >> 16) & 0xff;
////      prsdata[9] = (pyldmgr_state.bkend_hottime >> 24) & 0xff;
////      prsp->len += 8;
////
////      break;
////
////    case IPMICMD_CUSTOM_SET_SYSTIME:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_SET_SYSTIME\n");
////      // This function sets the 32-bit system time kept by the RTC clock and the 32KHz oscillator
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          32-bit system time, LS byte
////      //   2          32-bit system time, bits [15:8
////      //   3          32-bit system time, bits [23:16]
////      //   4          32-bit system time, MS byte
////      // Response Format:
////      //   1          Completion Code.
////      if (preq->len < IPMBREQOVHEAD+4) {
////        // not enough params
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      systime = prqdata[4];
////      systime = (systime << 8) + prqdata[3];
////      systime = (systime << 8) + prqdata[2];
////      systime = (systime << 8) + prqdata[1];
////      set_rtc_value(systime);
////      break;
////
////    case IPMICMD_CUSTOM_GET_SYSTIME:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_GET_SYSTIME\n");
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.
////      //   2          32-bit system time, LS byte
////      //   3          32-bit system time, bits [15:8
////      //   4          32-bit system time, bits [23:16]
////      //   5          32-bit system time, MS byte
////      systime = get_rtc_value();
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = systime & 0xff;
////      prsdata[3] = (systime >> 8) & 0xff;
////      prsdata[4] = (systime >> 16) & 0xff;
////      prsdata[5] = (systime >> 24) & 0xff;
////      prsp->len += 4;
////      break;
////
////    case IPMICMD_CUSTOM_POLL_FPGA_CFG_PORT:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_POLL_FPGA_CFG_PORT\n");
////      // This command polls the SPI1 port for connected FPGA Config Port SPI slaves.  It
////      // asserts the ~SCANSLV signal from the Microcontroller and samples the ~CSx pins
////      // as logic inputs.  Any ~CSx lines observed to go to a logic 0 are considered to
////      // have FPGA Config Ports attached to them.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      //   2          [7:4] - reserved, returns 1111b
////      //              [3] - SPI1 CS3 -- 1b = FPGA port detected, 0b = no FPGA detected
////      //              [2] - SPI1 CS2 -- 1b = FPGA port detected, 0b = no FPGA detected
////      //              [1] - SPI1 CS1 -- 1b = FPGA port detected, 0b = no FPGA detected
////      //              [0] - SPI1 CS0 -- 1b = FPGA port detected, 0b = no FPGA detected
////      if (!spi1_lock()) {
////        // SPI1 unavailable
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = 0xf0 | fpgaspi_slave_detect();
////      spi1_unlock();
////      prsp->len += 1;
////      break;
////
////    case IPMICMD_CUSTOM_FPGA_CFG_RDSR:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_RDSR\n");
////      // BYTE         DATA FIELD
////      // This command reads the FPGA Config Port Status register using the RDSR SPI command
////      // Command Format:
////      //   1          SPI1 Chip Select ID (0-3)
////      // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      //   2          FPGA SPI Configuration Port Status Register
////      //              [7] - Upper Handshake Flag (UHF)
////      //              [6] - Lower Handshake Flag (LHF)
////      //              [5] - Config Ready Flag (CFGRDY)-- reset to 0b by FPGA firmware @ startup,
////      //                    set to 1b after config image written to port
////      //              [4] - Request Config Flag (REQCFG) -- set to 1b by FPGA at init to request
////      //                    config data, cleared by FPGA after detecting CFGRDY set to 1b by SPI write
////      //              [3:0] - reserved
////      if (prqdata[1] > MAX_FPGA_ID) {
////        // bad param
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      if (!spi1_lock()) {
////        // SPI1 unavailable
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      prsdata[2] = fpgaspi_status_read(prqdata[1]);
////      spi1_unlock();
////      prsp->len += 1;
////      break;
////
////    case IPMICMD_CUSTOM_FPGA_CFG_WRCTL:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_WRCTL\n");
////      // This function writes a byte to the FPGA SPI Config Port Control register
////      // using the WRCTL SPI command.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          SPI1 Chip Select ID (0-3)
////      //   2          Data Byte Written to Control Register
////      //              [7] - 1b = select UHF, 0b = no select
////      //              [6] - 1b = select LHF, 0b = no select
////      //              [5] - 1b = select CFGRDY, 0b = no select
////      //              [4:2] - reserved
////      //              [1] - 1b = selected bits 7:5 should be set, 0b = no change
////      //              [0] - 1b = selected bits 7:5 should be cleared, 0b = no change
////      // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      if (prqdata[1] > MAX_FPGA_ID) {
////        // bad param
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      if (!spi1_lock()) {
////        // SPI1 unavailable
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      fpgaspi_ctl_write(prqdata[1], prqdata[2]);
////      spi1_unlock();
////      break;
////
////    case IPMICMD_CUSTOM_FPGA_CFG_WRITE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_WRITE\n");
////      // This function performs a raw write to the FPGA SPI Config Port.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          SPI1 Chip Select ID (0-3)
////      //   2          Config Port Destination byte address, LS byte
////      //   3          Config Port Destination byte address, MS byte
////      //   4          Count of Bytes to write (n)
////      //   5..5+(n-1) Write Data
////      // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      if ((prqdata[1] > MAX_FPGA_ID) || (prqdata[4] > NONVOLATILE_MAX_XFER_LEN)) {
////        // bad param
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      if (!spi1_lock()) {
////        // SPI1 unavailable
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      eepsaddr = prqdata[2] | (prqdata[3] << 8);
////      // transfer bytes to SPI port
////      fpgaspi_data_write(prqdata[1], &prqdata[5], eepsaddr, prqdata[4]);
////      spi1_unlock();
////      break;
////
////    case IPMICMD_CUSTOM_FPGA_CFG_READ:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_READ\n");
////      // This function performs a raw read from the FPGA SPI Config Port.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          SPI1 Chip Select ID (0-3)
////      //   2          Config Port Source byte address, LS byte
////      //   3          Config Port Source byte address, MS byte
////      //   4          Count of Bytes to read (n)
////       // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      //   2..2+(n-1) Read Data
////      if ((prqdata[1] > MAX_FPGA_ID) || (prqdata[4] > NONVOLATILE_MAX_XFER_LEN)) {
////        // bad param
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
////        break;
////      }
////      if (!spi1_lock()) {
////        // SPI1 unavailable
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
////      eepsaddr = prqdata[2] | (prqdata[3] << 8);
////      fpgaspi_data_read(prqdata[1], &prsdata[2], eepsaddr, prqdata[4]);
////      prsp->len += prqdata[4];
////      spi1_unlock();
////      break;
////
////    case IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_WRITE:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_WRITE\n");
////      // This function writes the header for the 256 byte Autoconfig section
////      // in the nonvolatile storage (EEPROM).  NOTE:  Any changes to the header
////      // or Autoconfig section of nonvolatile storage will not take effect until
////      // the MMC is reset.
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   1          Config Flags, 2 bits per chip select.  (x0b = config data undefined,
////      //              01b = defined, but autoconfig disabled, 11b = autoconfig enabled)
////      //              [7:6] CS3 autoconfig setting
////      //              [5:4] CS2 autoconfig setting
////      //              [3:2] CS1 autoconfig setting
////      //              [1:0] CS0 autoconfig setting
////      //   2          Byte offset to CS0 config data
////      //   3          Byte offset to CS1 config data
////      //   4          Byte offset to CS2 config data
////      //   5          Byte offset to CS3 config data
////      //   6          Header Checksum
////      // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      eepsaddr = FPGA_CONFIG_AREA_BYTE_OFFSET+1;      // skip past format flag
////      eepspi_write(&prqdata[1], eepsaddr, sizeof(FPGA_config_area_header_t)-1);
////      break;
////
////    case IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_READ:
////      sio_filt_putstr(TXTFILT_IPIM_CUSTOM_REQ, 1, "IPMICMD_CUSTOM_FPGA_CFG_NONVOL_HDR_READ\n");
////      // This function reads and returns the header for the 256 byte Autoconfig section
////      // BYTE         DATA FIELD
////      // Command Format:
////      //   (no bytes)
////      // Response Format:
////      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
////      //   2          Config Flags, 2 bits per chip select.  (x0b = config data undefined,
////      //              01b = defined, but autoconfig disabled, 11b = autoconfig enabled)
////      //              [7:6] CS3 autoconfig setting
////      //              [5:4] CS2 autoconfig setting
////      //              [3:2] CS1 autoconfig setting
////      //              [1:0] CS0 autoconfig setting
////      //   3          Byte offset to CS0 config data
////      //   4          Byte offset to CS1 config data
////      //   5          Byte offset to CS2 config data
////      //   6          Byte offset to CS3 config data
////      //   7          Header Checksum
////
////      if (eepspi_chk_write_in_progress()) {
////        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
////        break;
////      }
////      eepsaddr = FPGA_CONFIG_AREA_BYTE_OFFSET+1;      // skip past format flag
////      eepspi_read(&prsdata[2], eepsaddr, sizeof(FPGA_config_area_header_t)-1);
////      prsp->len += sizeof(FPGA_config_area_header_t)-1;
////      break;
//    case IPMICMD_PICMG_GET_PICMG_PROP:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_PICMG_PROP\n");
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = NETFN_PICMG_IDENTIFIER;
//      prsdata[3] = 0x14;                      // per AMC Spec 3.1.5
//      prsdata[4] = 0;                         // max FRU device ID
//      prsdata[5] = 0;                         // FRU device ID for device containing this MMC
//      prsp->len += 4;
//      break;
//
//    case IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES\n");
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = NETFN_PICMG_IDENTIFIER;
//      // capable of cold reset, warm reset, and graceful reboot
//      prsdata[3] = 0x06;
//      prsp->len += 2;
//      break;
//
//    case IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID\n");
//      // since the AMC specificaiton calls for a max device ID of zero, we expect this
//      // command to give a device ID of zero as byte 2 of the command data, which is interpreted
//      // as asking for the MMC device record index in the SDR
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = NETFN_PICMG_IDENTIFIER;
//      prsdata[3] = (unsigned char) (SDR_MMC & 0xff);
//      prsdata[4] = (unsigned char) (SDR_MMC >> 8);                // upper byte of MMC SDR record
//      prsp->len += 3;
//      break;
//
//
//    case IPMICMD_STOR_READ_FRU_DATA:
//
//    	sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_READ_FRU_DATA\n");
//
//      fruoffset = prqdata[2] + (prqdata[3] << 8);
//      datalen = prqdata[4];
//      if (datalen > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-1)) {
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = datalen;
//
//
//     uint8_t len = datalen;
//     uint8_t i = 0;
//      for(; len>0; len--)
//      {
//    	  prsdata[ 3+i ] = 	fru_buf[COMMON_HEADER_BYTE_OFFSET+fruoffset+i];
//    	  i++;
//      }
//
//      prsp->len += datalen+1;
//      break;
//
//    case IPMICMD_STOR_WRITE_FRU_DATA:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_WRITE_FRU_DATA\n");
//
//      fruoffset = prqdata[2] + (prqdata[3] << 8);
//      datalen = prqdata[4];
//
//      if (datalen > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-1))
//      {
//       	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
//        break;
//      }
//
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//
//      prsdata[2] = datalen;
//      len = datalen;
//      i = 0;
//
//      for(; len>0; len--)
//        {
//      	  fru_buf[COMMON_HEADER_BYTE_OFFSET+fruoffset+i] = prsdata[ 4+i ];
//      	  i++;
//        }
//
//      prsp->len += 1;
//      break;
//
//    case IPMICMD_SE_GET_DEVICE_SDR_INFO:
//     sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_DEVICE_SDR_INFO\n");
//
//     IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//
//     if (IPMB_RQ_rsLUN_GET(preq->buf) != 0)
//        prsdata[2] = 0;								// no sensors or devices for LUN != 0
//     else
//     {
//        if (preq->len <= IPMBREQOVHEAD)
//        {
//      	  // operation data byte missing, so by definition return the number of sensors
//          prsdata[2] = SDRstate.sensor_cnt;
//        }
//        else
//        {
//      	  if ( prqdata[1] & 0x1 )
//      		prsdata[2] = SDRstate.SDR_cnt;
//          else
//            prsdata[2] = SDRstate.sensor_cnt;
//        }
//      }
//
//      prsdata[3] = 0x01;				// per AMC.0 3.11.1, static sensors, LUN 0 only
//      prsp->len += 2;
//
//      break;
//
//    case IPMICMD_SE_GET_DEVICE_SDR:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_DEVICE_SDR\n");
//      rsvID = prqdata[2] << 8 | prqdata[1];			// reservation ID for this command
//      if ((prqdata[5] != 0) && (rsvID != SDRstate.SDRreservationID)) {
//        // reservation ID doesn't match currently active one, so send an error completion code
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RSV_ID;
//        break;
//      }
//      recordID = prqdata[4] << 8 | prqdata[3];
//      pSDRdata = get_SDR_entry_addr(recordID);
//      if (!pSDRdata) {
//        // record ID out of range
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//      pSDRhdr = (SDR_entry_hdr_t*) pSDRdata;
//      if (prqdata[6] == 0xff)
//    	rdlength = pSDRhdr->reclength - prqdata[5];
//      else
//        rdlength = prqdata[6];
//      if (rdlength > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-3)) {
//    	  IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
//    	  break;
//      }
//  	  else {
//  	    // return read data
//  	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//  	    recordID = get_SDR_next_recID(pSDRhdr);
//  	    prsdata[2] = recordID & 0xff;
//  	    prsdata[3] = recordID >> 8;
//        prsp->len += 2+rdlength;
// 	      prsbyte = &prsdata[4];
// 	      pSDRdata += prqdata[5];         // advance data pointer to starting offset
// 	      while (rdlength) {
// 	        *prsbyte++ = *pSDRdata++;
// 	        rdlength--;
// 	      }
//      }
//      break;
//
//    case IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY\n");
//      // bump SDR repository reservation counter and return the value
//      SDRstate.SDRreservationID++;
//	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = SDRstate.SDRreservationID & 0xff;
//      prsdata[3] = SDRstate.SDRreservationID >> 8;
//      prsp->len += 2;
//      break;
//
//    case IPMICMD_SE_GET_SENSOR_HYSTERESIS:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_HYSTERESIS\n");
//      // find SDR number from sensor number
//      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
//      if (pSensorSDR == NULL) {
//        // bad sensor ID
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//      IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsp->len += 2;
//      if (((pSensorSDR->sensorcap & 0x30) < 0x10) || ((pSensorSDR->sensorcap & 0x30) > 0x20)) {
//        // hysteresis not readable on this sensor
//        prsdata[2] = 0x00;
//        prsdata[3] = 0x00;
//      }
//      else {
//        prsdata[2] = pSensorSDR->pos_thr_hysteresis;
//        prsdata[3] = pSensorSDR->neg_thr_hysteresis;
//      }
//      break;
//
//    case IPMICMD_SE_SET_SENSOR_THRESHOLD:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_SET_SENSOR_THRESHOLD\n");
//      // first check to see if EEPROM is available
//      if (eepspi_chk_write_in_progress()) {
//    	  IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
//        break;
//      }
//      // This command will update the sensor thresholds in both the RAM and EEPROM-based SDR records for
//      // the specified sensor.
//      // Here's the trick:  we're taking advantage of the fact that the thresholds all fall within the
//      // same 32-byte block in the EEPROM, so we can build a buffer image of the current thresholds,
//      // update those that are changed by the command, and write the settings back to EEPROM with a single
//      // write operation, such that the EEPROM updates itself while the response is returned.  We avoid the
//      // situation where we have to wait for one or more block writes to complete before finishing the
//      // others, which would put the EEPROM internal update time for the earlier writes in the command
//      // processing path, which we want to avoid--lest the controller decide to reissue the commands
//
//      // find SDR number from sensor number
//      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
//      if (pSensorSDR == NULL) {
//        // bad sensor ID
//      	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//      // initialize buffer with current or new threshold values
//      pSDRdata = (unsigned char*) &pSensorSDR->upper_nonrecover_thr;
//      for (i1=0; i1<6; i1++)
//        if ((prqdata[2] & (1 << i1)) & pSensorSDR->settable_threshold_mask)
//          // new value for threshold
//          pSDRdata[5-i1] = prqdata[3+i1];
// 	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      eepspi_write(pSDRdata, SDR_AREA_BYTE_OFFSET+(unsigned short) (pSDRdata - (unsigned char*) &SDRtbl), 6);
//      break;
//
//    case IPMICMD_SE_GET_SENSOR_THRESHOLD:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_THRESHOLD\n");
//      // find SDR number from sensor number
//      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
//      if (pSensorSDR == NULL) {
//        // bad sensor ID
//      	IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      prsdata[2] = pSensorSDR->readable_threshold_mask;
//      prsdata[8] = pSensorSDR->upper_nonrecover_thr;
//      prsdata[7] = pSensorSDR->upper_critical_thr;
//      prsdata[6] = pSensorSDR->upper_noncritical_thr;
//      prsdata[5] = pSensorSDR->lower_nonrecover_thr;
//      prsdata[4] = pSensorSDR->lower_critical_thr;
//      prsdata[3] = pSensorSDR->lower_noncritical_thr;
//      prsp->len += 7;
//      break;
//
//    case IPMICMD_SE_GET_SENSOR_READING:
//      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_READING\n");
//      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
//      if (pSensorSDR == NULL) {
//        // bad sensor ID
//        IPMI_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
//        break;
//      }
//      // access sensor data table to get sensor value
//	    IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//      if (prqdata[1] == HOTSWAP_SENSOR) {
//	      // return AMC-specified response for hotswap sensor
//	      prsdata[2] = 0x00;
//        prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;   // scanning always enabled, plus event status
//	    prsdata[4] = SensorData[HOTSWAP_SENSOR].readout_value;
//	    }
//	    else {
//	      // return IPMI-specified response for temperature or voltage sensors
//	      prsdata[2] = SensorData[prqdata[1]].readout_value;
//	      prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;		// scanning always enabled, plus event status
//	      prsdata[4] = SensorData[prqdata[1]].comparator_status;
//      }
//      prsp->len += 3;
//      break;
//
//
//    case IPMICMD_PICMG_GET_FRU_LED_STATE:
//          sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_FRU_LED_STATE\n");
//          prsdata[2] = NETFN_PICMG_IDENTIFIER;
//          if (prqdata[3] >= IPMI_LED_CNT) {
//            IPMI_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
//            prsp->len += 1;
//            break;
//          }
//          // need to poke around in the LED state, so turn off interrupts for a little
//          // bit while we see what's going on with the LED
//          Disable_global_interrupt();
//          pLEDstate = &LEDstate[prqdata[3]];      // pointer to LED state
//          IPMI_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
//          // response byte 3, LED states
//          prsdata[3] = 0x00;
//          if (pLEDstate->LEDstate == Local_Control)
//            prsdata[3] |= 0x01;
//          else {
//            if (pLEDstate->pOvrideDesc->delay2 == 0)
//              prsdata[3] |= 0x04;         // lamp test
//            else
//              prsdata[3] |= 0x02;         // override
//          }
//          // response byte 4/5/6, local control
//          switch (pLEDstate->pLocalDesc->action) {
//            case On:
//              prsdata[4] = 0xff;
//              prsdata[5] = 0;
//              break;
//            case Blink:
//              if (pLEDstate->pLocalDesc->initstate == LEDON) {
//                prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
//                prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
//              }
//              else {
//                prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
//                prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
//              }
//              break;
//            case Off:
//            case Bypass:
//            default:
//              prsdata[4] = 0;
//              prsdata[5] = 0;
//          }
//          prsdata[6] = pLEDstate->Color;
//          prsp->len += 5;             // update length for bytes encoded so far
//          // response bytes 7-9, override state
//          if (prsdata[3] > 1) {
//            // have override bytes to report
//            prsdata[9] = pLEDstate->Color;
//            switch (pLEDstate->pOvrideDesc->action) {
//              case On:
//                prsdata[7] = 0xff;
//                prsdata[8] = 0;
//                prsp->len += 3;
//                break;
//              case Blink:
//                if (pLEDstate->pOvrideDesc->delay2 == 0) {
//                  // lamp test
//                  prsdata[7] = 0xfb;
//                  prsdata[8] = 0;
//                  prsdata[10] = (unsigned char) ((pLEDstate->pOvrideDesc->delay1/10) & 0xff);
//                  prsp->len += 4;
//                }
//                else {
//                  if (pLEDstate->pOvrideDesc->initstate == LEDON) {
//                    prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
//                    prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
//                  }
//                  else {
//                    prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
//                    prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
//                  }
//                  prsp->len += 3;
//                }
//              break;
//              case Off:
//              case Bypass:
//              default:
//                prsdata[7] = 0;
//                prsdata[8] = 0;
//                prsp->len += 3;
//            }
//          }
//          Enable_global_interrupt();
//          break;
//
//
//    default:
//      prsp->len = 0;      // nothing supported here yet
//  }
//}
