#include <stdio.h>
#include <string.h>

#include "../include/swevent.h"
#include "../include/timer_callback.h"
#include "../include/sio_usart.h"

#include "../include/ipmi_i2c_driver.h"

#include "../include/cardType.h"
#include "../include/cardStatus.h"
#include "../include/comExpress.h"

#include "../include/rtc.h"
#include "../include/ipmb_svc.h"
#include "../include/ipmi_cmd_parser.h"
#include "../include/LEDdrivers.h"


static void raw_byte_char_dump(const volatile uint8_t* puc, const volatile uint8_t len);

static void process_rsp_msg(volatile ipmb_msg_desc_t* pmsg);

static int reserve_req_msg_table_entry(uint8_t msg_rqSEQ);

static uint8_t xmt_retry_timer_callback(event evID, void* arg);

static void check_req_msg_table(void);

static void check_incoming_msgs(void);

static int match_rsp_to_req(uint8_t rsp_rqSeq, uint8_t rsp_rsSA);

static void free_req_msg_table_entry(int entry_index);

static void request_msg_dump(volatile ipmb_msg_desc_t* pmsg);


volatile uint8_t event_rcvr_ipmbl_addr;
volatile uint8_t event_rcvr_lun;

volatile uint8_t rqSEQ;
volatile req_msg_tbl_entry_t ReqMsgTbl[REQ_MSG_TBL_SIZE];


void ipmb_init(void)
{
  rqSEQ = 1;
  memset((void*) &ReqMsgTbl, 0, sizeof(req_msg_tbl_entry_t));

  event_rcvr_ipmbl_addr = DEFAULT_EVT_RCVR_IPMBL_ADDR;

  register_timer_callback(xmt_retry_timer_callback, TEVENT_100MSEC);
}


void ipmb_init_req_hdr(volatile uint8_t* pmsg, volatile uint8_t rsSA, volatile uint8_t netFn,
                       volatile uint8_t rsLUN, volatile uint8_t rqLUN)
{
  IPMB_RQ_rsSA(pmsg) = rsSA;
  IPMB_RQ_netFN_SET(pmsg, netFn);
  IPMB_RQ_rsLUN_SET(pmsg, rsLUN);
  IPMB_RQ_HXSUM(pmsg) = calc_ipmi_xsum(pmsg, 2);
  IPMB_RQ_rqSA(pmsg) = ipmi_i2c_state.ipmbl_addr;
  IPMB_RQ_rqSEQ_SET(pmsg, rqSEQ++);
  IPMB_RQ_rqLUN_SET(pmsg, rqLUN);
}


void ipmb_init_rsp_hdr(volatile uint8_t* pmsg, const volatile uint8_t* preq)
{
  // build response header from request header
  IPMB_RS_rqSA(pmsg) = IPMB_RQ_rqSA(preq);
  IPMB_RS_netFN_SET(pmsg, (IPMB_RQ_netFN_GET(preq) | 1));
  IPMB_RS_rqLUN_SET(pmsg, IPMB_RQ_rqLUN_GET(preq));
  IPMB_RS_HXSUM(pmsg) = calc_ipmi_xsum(pmsg,2);
  IPMB_RS_rsSA(pmsg) = IPMB_RQ_rsSA(preq);
  IPMB_RS_rqSEQ_SET(pmsg, IPMB_RQ_rqSEQ_GET(preq));
  IPMB_RS_rsLUN_SET(pmsg, IPMB_RQ_rsLUN_GET(preq));
  IPMI_RS_CMD(pmsg)= IPMI_RQ_CMD(preq);
}


unsigned char calc_ipmi_xsum(const unsigned char* pbuf, unsigned short len)
{
  unsigned char  sum     = 0;
  const unsigned char* pcur    = pbuf;
  volatile unsigned short  remlen  = len;

  while (remlen)
  {
    sum += *pcur++;
    remlen--;
  }

  return ( (~sum) + 1 );
}


void ipmb_service(void)
{
  // function checks for any incoming messages and also if any retries need to be sent on
  // outgoing requests
  check_incoming_msgs();
  check_req_msg_table();

}


static void check_incoming_msgs(void)
{
  // this function checks the ipmi_i2c queue for incoming IPMI requests, and posts
  // events to allow the assigned callbacks to run
  volatile ipmb_msg_desc_t curmsg;
  volatile uint8_t xsumval;

  int i1;

  // zero out message buffer before get
  memset((void*) curmsg.buf, 0, sizeof(curmsg.buf));

  if (!get_ipmi_i2c_msg(curmsg.buf, &curmsg.len, IPMIMAXMSGLEN, &curmsg.systime))
    // queue empty
    return;

  // pulse LED1 to indicate IPMI RCV activity
  program_LED(LED1_TBL_IDX, Local_Control, &LED_50ms_Flash_Activity);

  // check if first byte is zero.  If it is, then this is a broadcast command.  Discard
  // the zero by shifting all of the higher bytes down and decrementing the count
  if (curmsg.buf[0] == 0x00)
  {
    curmsg.len--;
    for (i1=0; i1<curmsg.len; i1++)
      curmsg.buf[i1] = curmsg.buf[i1+1];
  }

  // check minimum/maximum message length
  if (curmsg.len < IPMIMINMSGLEN)
  {
	  sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "?Error, incoming IPMI message too short\n");
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, "  Raw message:  ");
	  raw_byte_char_dump(curmsg.buf, curmsg.len);
	  return;
  }

  // perform checksums on header and body
  xsumval = calc_ipmi_xsum(curmsg.buf, 2);
  if (xsumval != curmsg.buf[2])
  {
	// header xsum mismatch
	  sprintf(spbuf, "?Error, IPMI message header xsum error\n  Calculated value=%02X, Data value = %02X\n", xsumval, curmsg.buf[2]);
	  sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, spbuf);
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, "  Raw Message:  ");
	  raw_byte_char_dump(curmsg.buf, curmsg.len);
	  return;
  }

  xsumval = calc_ipmi_xsum(curmsg.buf+3, curmsg.len-4);

  if (xsumval != curmsg.buf[curmsg.len-1])
  {
	// body xsum mismatch
	  sprintf(spbuf, "?Error, IPMI message body xsum error\n  Calculated value=%02X, Data value = %02X\n", xsumval, curmsg.buf[curmsg.len-1]);
	  sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, spbuf);
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, " Raw Message:  ");

	  raw_byte_char_dump(curmsg.buf, curmsg.len);

   return;
  }

  // check to see if this is request or response
  if (IPMB_RQ_netFN_GET(curmsg.buf) & 1)
    process_rsp_msg(&curmsg);
  else
    // even numbered function, this is a request
    post_swevent(IPMBEV_REQ_RCVD, (void*) &curmsg);
}


static void raw_byte_char_dump(const volatile uint8_t* puc, const volatile uint8_t len)
{
  const volatile uint8_t* pcur = puc;
  volatile uint8_t curlen = len;

  while (curlen)
    {
	  sprintf(spbuf, "%02X ", *pcur++);
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 0, spbuf);
	  curlen--;
  }

  sio_filt_putstr(TXTFILT_DBG_DETAIL, 0, "\n");

}


int ipmb_send_request(volatile ipmb_msg_desc_t* pmsg, volatile ptrMsgCallback rspCallback)
{
  // attempts to load a message into the request table and send it via the I2C interface
  // returns 1 if request loaded into table, 0 otherwise
  volatile req_msg_tbl_entry_t* preq;
  int i1;
  int tblidx = reserve_req_msg_table_entry(IPMB_RQ_rqSEQ_GET(pmsg->buf));

  if (tblidx == -1)
  {
    sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "?Error, request message table full\n");
    return (0);				// no entries available
  }

  // copy message into table
  preq = &ReqMsgTbl[tblidx];

  for (i1=0; i1<pmsg->len; i1++)
    preq->msg.buf[i1] = pmsg->buf[i1];

  preq->msg.len = pmsg->len;
  preq->prspcallback = rspCallback;				// copy callback pointer
  request_msg_dump(pmsg);

  // try to transmit message
  if (!put_ipmi_i2c_msg(preq->msg.buf, preq->msg.len))
  {
    sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, request message NOT enqueued\n  ");
  }
  else
  {
	  preq->timer = 0;				// zero out timer
	  preq->xmtcnt++;				// bump transmit count
  }

  return (1);
}


int ipmb_send_response(volatile ipmb_msg_desc_t* pmsg)
{
  // sends a response to the I2C driver.  Returns 1 if driver accepts message, 0 otherwise
  if (pmsg->len)
  {
    // add data checksum
    pmsg->buf[pmsg->len-1] = calc_ipmi_xsum(pmsg->buf+3, pmsg->len-4);

    if (!put_ipmi_i2c_msg(pmsg->buf, pmsg->len))
    {
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, response message NOT enqueued\n  ");
      raw_byte_char_dump(pmsg->buf, pmsg->len);

      return (0);
    }

    return (1);
  }

  return (0);
}


static void process_rsp_msg(volatile ipmb_msg_desc_t* pmsg)
{
  // first look for a match in the request msg table
  volatile uint64_t systime = get_rtc_value();
  volatile req_msg_tbl_entry_t* preq;
  volatile int tblidx = match_rsp_to_req(IPMB_RS_rqSEQ_GET(pmsg->buf), IPMB_RS_rsSA(pmsg->buf));

  if (tblidx == -1)
  {
    sprintf(spbuf, "(systime 0x%08lx)  Unmatched received response:\n", systime);
    sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, spbuf);
    ipmb_msg_dump(pmsg, TXTFILT_DBG_DETAIL);
    post_swevent(IPMBEV_UNMATCHED_RSP_MSG, (void*) pmsg);

    return;
  }

  preq = &ReqMsgTbl[tblidx];

  sprintf(spbuf, "(systime 0x%08lx  rqSEQ=0x%02x) Received Response, Completion Code=0x%02x", systime,
          IPMB_RS_rqSEQ_GET(pmsg->buf), IPMI_RS_CCODE(pmsg->buf));

  sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);

  if (IPMI_RS_CCODE(pmsg->buf) == IPMI_RS_NORMAL_COMP_CODE)
    sio_filt_putstr(TXTFILT_EVENTS, 0, " (normal)\n");
  else
    sio_filt_putstr(TXTFILT_EVENTS, 0, "\n");

  // if a callback is defined, make the call for further processing of the response message
  if (preq->prspcallback != NULL)
	(*preq->prspcallback)((void*) &(preq->msg), (void*) pmsg);			// request is arg1, response is arg2

  free_req_msg_table_entry(tblidx);										// delete request from table
}


static int reserve_req_msg_table_entry(uint8_t msg_rqSEQ)
{
  // finds an unused location in the request message table and reserves it
  // returns the table index if successful, otherwise -1
  int i1;
  volatile req_msg_tbl_entry_t* preq = &ReqMsgTbl[0];

  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++)
  {
      if (!(preq->ocflag_rqSeq & 0x80))
      {
	 // this one is free
	 preq->msg.len = 0;				// no message in buffer yet
	 preq->xmtcnt = 0;
	 preq->ocflag_rqSeq = 0x80 | msg_rqSEQ;		// set reservation key based on rqSEQ value

	 return (i1);
      }

      preq++;
  }

  return (-1);			// no slots open
}


static uint8_t xmt_retry_timer_callback(event evID, void* arg)
{
  unsigned char i1;
  req_msg_tbl_entry_t* preq = &ReqMsgTbl[0];

  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++)
    {
	// bump the timer for each occupied entry in the request table
	if ((preq->ocflag_rqSeq & 0x80) && (preq->xmtcnt>0))
	{
	  // this table entry is occupied and has been transmitted--bump the response timer count
          preq->timer++;
	  preq++;					// to next entry
	}
    }

  return (1);
}

static void check_req_msg_table(void)
{
  // check the request message table for messages that need to be transmitted--or retransmitted
  uint8_t i1;

  volatile uint8_t xmtcnt, timerval;

  volatile req_msg_tbl_entry_t* preq;

  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++)
  {
	preq = &ReqMsgTbl[i1];

	if ((!(preq->ocflag_rqSeq & 0x80)) || (preq->msg.len == 0))
	  continue;					// this table position is currently empty or not ready to be transmitted

	xmtcnt = preq->xmtcnt;
	timerval = preq->timer;

    if (!xmtcnt)
     {
      // this request hasn't been transmitted yet, so send it
      if (!put_ipmi_i2c_msg(preq->msg.buf, preq->msg.len))
        {
          sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, request message NOT enqueued\n  ");
        }
      else
        {
          preq->timer = 0;			// zero out timer
          preq->xmtcnt = 1;			// initialize transmit count
        }

      return;
     }

     if (timerval < REQ_MSG_TIMEOUT_LIMIT)
      // message still waiting for response to last xmt, skip it
      continue;
     if (xmtcnt >= REQ_MSG_XMT_TRY_LIMIT)
      {
      // this message has been around too long without getting a response--delete it
         sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? No ack received for request message, giving up on retries\n");
         post_swevent(IPMBEV_UNACK_REQ_MSG, (void*) &preq->msg);
         free_req_msg_table_entry(i1);
       continue;
      }
    // message needs retransmit
     if (!put_ipmi_i2c_msg(preq->msg.buf, preq->msg.len))
       {
         sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, request message NOT enqueued\n  ");
       }
     else
      {
  	 preq->timer = 0;				// zero out timer
  	 preq->xmtcnt++;				// bump transmit count
      }
     return;
  }
}


static int match_rsp_to_req(uint8_t rsp_rqSeq, uint8_t rsp_rsSA)
{
  // scans request message table, looking for a request with the matching rqSeq and rsSA
  // returns index of entry if it finds one, otherwise -1 if it doesn't
  volatile uint8_t key = 0x80 | (rsp_rqSeq & 0x3f);			// matching key
  int i1;

  req_msg_tbl_entry_t* preq = &ReqMsgTbl[0];

  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++)
  {
      if (preq->ocflag_rqSeq == key)
        if (IPMB_RQ_rsSA(preq->msg.buf) == rsp_rsSA)
          // have match also on rsSA
           return (i1);
    preq++;
  }

  return (-1);
}


static void free_req_msg_table_entry(int entry_index)
{
  // function to free up an entry in the request message table
  if ((entry_index <0) || (entry_index >= REQ_MSG_TBL_SIZE))
	return;			// out of range
  ReqMsgTbl[entry_index].ocflag_rqSeq = 0x00;				// clear occupied flag
}

void ipmb_msg_dump(const volatile ipmb_msg_desc_t* pmsg, int volatile filtcode)
{
  // formatted dump of message to console
  int remlen;

  if (!(IPMB_RQ_netFN_GET(pmsg->buf) & 1))
    {
      sio_filt_putstr(filtcode, 1, "rsSA  netFn  rsLUN  Hxsum  rqSA  rqSeq  rqLUN  cmd  Mxsum\n");
      sprintf(spbuf, " %02x    %02x      %01x     %02x     %02x    %02x      %01x    %02x    %02x\n",
      IPMB_RQ_rsSA(pmsg->buf), IPMB_RQ_netFN_GET(pmsg->buf), IPMB_RQ_rsLUN_GET(pmsg->buf),
      IPMB_RQ_HXSUM(pmsg->buf), IPMB_RQ_rqSA(pmsg->buf), IPMB_RQ_rqSEQ_GET(pmsg->buf),
      IPMB_RQ_rqLUN_GET(pmsg->buf), IPMI_RQ_CMD(pmsg->buf), pmsg->buf[pmsg->len-1]);
      sio_filt_putstr(filtcode, 1, spbuf);
      remlen = pmsg->len-IPMI_RQ_DATA_OFFSET-1;
      sio_filt_putstr(filtcode, 1, "Data Dump:\n");

    if (remlen)
     {
      raw_byte_char_dump(&pmsg->buf[IPMI_RQ_DATA_OFFSET], remlen);
      sio_filt_putstr(filtcode, 0, "\n");
     }
    else
      sio_filt_putstr(filtcode, 0, "(no data)\n\n");
  }
  else
   {
      sio_filt_putstr(filtcode, 1, "rqSA  netFn  rqLUN  Hxsum  rsSA  rqSeq  rsLUN  cmd  ccode  Mxsum\n");
      sprintf(spbuf, " %02x    %02x      %01x     %02x     %02x    %02x      %01x    %02x    %02x    %02x\n",
      IPMB_RS_rqSA(pmsg->buf), IPMB_RS_netFN_GET(pmsg->buf), IPMB_RS_rqLUN_GET(pmsg->buf),
      IPMB_RS_HXSUM(pmsg->buf), IPMB_RS_rsSA(pmsg->buf), IPMB_RS_rqSEQ_GET(pmsg->buf),
      IPMB_RS_rsLUN_GET(pmsg->buf), IPMI_RS_CMD(pmsg->buf), IPMI_RS_CCODE(pmsg->buf), pmsg->buf[pmsg->len-1]);
      sio_filt_putstr(filtcode, 1, spbuf);
      sio_filt_putstr(filtcode, 1, "Data Dump:\n");
      remlen = pmsg->len-IPMI_RS_DATA_OFFSET-1;

      if (remlen)
        {
          raw_byte_char_dump(&pmsg->buf[IPMI_RS_DATA_OFFSET], remlen);
          sio_filt_putstr(filtcode, 0, "\n");
        }
      else
        sio_filt_putstr(filtcode, 0, "(no data)\n\n");
    }
}


void ipmb_set_event_rcvr_ipmb_addr(unsigned char ipmbl_addr)
{
  event_rcvr_ipmbl_addr = ipmbl_addr;
}


uint8_t ipmb_get_event_rcvr_ipmb_addr(void)
{
  return (event_rcvr_ipmbl_addr);
}


void ipmb_set_event_rcvr_lun(unsigned char lun)
{
  event_rcvr_lun = lun;
}


uint8_t ipmb_get_event_rcvr_lun(void)
{
  return (event_rcvr_lun);
}


static void request_msg_dump(volatile ipmb_msg_desc_t* pmsg)
{
  const char* Offset_Decode_str[] = {"Lower Non-Critical going low",
    "Lower Non-Critical going high",
    "Lower Critical going low",
    "Lower Critical going high",
    "Lower Non-Recoverable going low",
    "Lower Non-Recoverable going high",
    "Upper Non-Critical going low",
    "Upper Non-Critical going high",
    "Upper Critical going low",
    "Upper Critical going high",
    "Upper Non-Recoverable going low",
    "Upper Non-Recoverable going high",
    "Code 12",
    "Code 13",
    "Code 14",
    "Code 15"};

  const char* Hotswap_Event_str[] = {"Handle closed",
    "Handle opened",
    "Quiesced",
    "Backend power failure",
    "Backend power shutdown"};

  const char* Sensor_Event_str[] = {"Temperature",
    "Voltage"};
  // dump of request message
  // if request is a sensor event, it is a formatted dump
  // if it is some other kind of message, it is a unformatted dump

  int code_offset;
  unsigned long systime = get_rtc_value();

  volatile uint8_t* prqdata = &pmsg->buf[IPMI_RQ_DATA_OFFSET-1];   // offset pointer for 1-base refs to req data

  if ((IPMB_RQ_netFN_GET(pmsg->buf) == NETFN_SE) && (IPMI_RQ_CMD(pmsg->buf) == IPMICMD_SE_PLATFORM_EVENT))
    {
    // this is a sensor event request message
    if (prqdata[2] == 0xf2)
      {
      // sensor type = hotswap sensor
        sprintf(spbuf, "(systime 0x%08lx  rqSEQ=0x%02x) Hotswap event:  ", systime, IPMB_RQ_rqSEQ_GET(pmsg->buf));
        sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);

        if (prqdata[5] < 5)
        {
         sprintf(spbuf, "%s\n", Hotswap_Event_str[prqdata[5]]);

#ifdef AMC_CPU_COM_Express
        if(prqdata[5] != 1)
          {
            commExpressLoggedIn = 0x00;
            COM_Express_initialized = 0x00;
          }
#endif
          }
        else
          sprintf(spbuf, "Unknown code 0x%01x\n", prqdata[5] & 0x0f);

        sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
    }
    else
      {
        // threshold sensor
        sprintf(spbuf, "(systime 0x%08lx  rqSEQ=0x%02x) Sensor %i ", systime, IPMB_RQ_rqSEQ_GET(pmsg->buf), prqdata[3]);
        sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);

        if ((prqdata[2] > 0) && (prqdata[2] < 3))
          sprintf(spbuf, "%s ", Sensor_Event_str[prqdata[2]-1]);
        else
          sprintf(spbuf, "(Unknown sensor type 0x%02x) ", prqdata[2]);

        sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);

        if (prqdata[4] & 0x80)
          sio_filt_putstr(TXTFILT_EVENTS, 0, "deassertion, ");
        else
          sio_filt_putstr(TXTFILT_EVENTS, 0, "assertion, ");

        code_offset = prqdata[5] & 0xf;

        switch (prqdata[4] & 0x7f)
        {
          case 0x01:
            sprintf(spbuf, "threshold event:  Offset=%i (%s), Reading=0x%02x, Thr=0x%02x\n", code_offset, Offset_Decode_str[code_offset], prqdata[6], prqdata[7]);
            sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
          break;

          default:
            sio_filt_putstr(TXTFILT_EVENTS, 0, "(unknown event type)\n");
          break;
        }
    }
  }
  else
    {
      sprintf(spbuf, "(systime 0x%08lx) Sending Request:\n", systime);
      sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);
      ipmb_msg_dump(pmsg, TXTFILT_EVENTS);
    }
}

