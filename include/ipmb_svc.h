

#ifndef IPMB_SVC_H_
#define IPMB_SVC_H_

// ipmb header macros
#define IPMB_RQ_rsSA(p)				(*(p+0))
#define IPMB_RQ_netFN_GET(p)		(((*(p+1)) >> 2) & 0x3f)
#define IPMB_RQ_netFN_SET(p, v)		{*(p+1) = (*(p+1) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RQ_rsLUN_GET(p)		(*(p+1) & 0x3)
#define IPMB_RQ_rsLUN_SET(p, v)		{*(p+1) = (*(p+1) & 0xfc) | (v & 0x3); }
#define IPMB_RQ_HXSUM(p)			(*(p+2))
#define IPMB_RQ_rqSA(p)				(*(p+3))
#define IPMB_RQ_rqSEQ_SET(p, v)		{*(p+4) = (*(p+4) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RQ_rqSEQ_GET(p)		(((*(p+4)) >> 2) & 0x3f)
#define IPMB_RQ_rqLUN_GET(p)		(*(p+4) & 0x3)
#define IPMB_RQ_rqLUN_SET(p, v)		{*(p+4) = (*(p+4) & 0xfc) | (v & 0x3); }
#define IPMI_RQ_CMD(p)				(*(p+5))
#define IPMI_RQ_DATA_OFFSET			(6)

#define IPMB_RS_rqSA(p)				(*(p+0))
#define IPMB_RS_netFN_GET(p)		(((*(p+1)) >> 2) & 0x3f)
#define IPMB_RS_netFN_SET(p, v)		{*(p+1) = (*(p+1) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RS_rqLUN_GET(p)		(*(p+1) & 0x3)
#define IPMB_RS_rqLUN_SET(p, v)		{*(p+1) = (*(p+1) & 0xfc) | (v & 0x3); }
#define IPMB_RS_HXSUM(p)			(*(p+2))
#define IPMB_RS_rsSA(p)				(*(p+3))
#define IPMB_RS_rqSEQ_SET(p, v)		{*(p+4) = (*(p+4) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RS_rqSEQ_GET(p)		(((*(p+4)) >> 2) & 0x3f)
#define IPMB_RS_rsLUN_GET(p)		(*(p+4) & 0x3)
#define IPMB_RS_rsLUN_SET(p, v)		{*(p+4) = (*(p+4) & 0xfc) | (v & 0x3); }
#define IPMI_RS_CMD(p)				(*(p+5))
#define IPMI_RS_CCODE(p)			(*(p+6))
#define IPMI_RS_DATA_OFFSET			(6)

#define IPMIMINMSGLEN			(7)			// minimum size for valid ipmi message
#define IPMIMAXMSGLEN			(32)		// max IPMI msg length on IPMB
#define IPMBREQOVHEAD			(7)			// bytes of IPMB overhead in request message
#define IPMBRSPOVHEAD			(8)			// bytes of IPMB overhead in response message

#define DEFAULT_EVT_RCVR_IPMBL_ADDR			(0x20)			// default ipmb-l address for event receiver


// request message table
#define REQ_MSG_TBL_SIZE                        (8)                     // size of request message table
#define REQ_MSG_TIMEOUT_LIMIT           (2)            // 100 ms ticks until message times out
#define REQ_MSG_XMT_TRY_LIMIT           (1)                     // max number of times message transmitted until we give up


typedef uint8_t(*ptrMsgCallback)(void*, void*);

typedef struct {
  uint8_t buf[IPMIMAXMSGLEN];
  unsigned short len;
  unsigned long systime;
} ipmb_msg_desc_t;


typedef struct {
  uint8_t ocflag_rqSeq;                           // entry occupied flag (bit 7) plus msg rqSeq in bit 5-0
  ipmb_msg_desc_t msg;                                  // message buffer and length field
  uint8_t xmtcnt;                                 // times transmitted
  uint8_t timer;                                  // 100ms ticks since last transmit
  ptrMsgCallback prspcallback;                          // pointer to callback function
} req_msg_tbl_entry_t;






unsigned char calc_ipmi_xsum(const unsigned char* pbuf, unsigned short len);

void ipmb_init_req_hdr(volatile uint8_t* pmsg, volatile uint8_t rsSA, volatile uint8_t netFn,
                       volatile uint8_t rsLUN, volatile uint8_t rqLUN);

void ipmb_init_rsp_hdr(volatile uint8_t* pmsg, const volatile uint8_t *preq);

int ipmb_send_request(volatile ipmb_msg_desc_t* pmsg, volatile ptrMsgCallback rspCallback);

int ipmb_send_response(volatile ipmb_msg_desc_t* pmsg);

void ipmb_init(void);

void ipmb_service(void);

void ipmb_msg_dump(const volatile ipmb_msg_desc_t* pmsg, int volatile filtcode);

void ipmb_set_event_rcvr_ipmb_addr(uint8_t ipmbl_addr);

uint8_t ipmb_get_event_rcvr_ipmb_addr(void);

void ipmb_set_event_rcvr_lun(uint8_t lun);

uint8_t ipmb_get_event_rcvr_lun(void);




#endif /* IPMB_SVC_H_ */
