
#include "../include/cmcpins.h"
#include "../include/gpio.h"
#include "../include/utils.h"

#include "../include/i2c_flags.h"
#include "../include/timer_callback.h"

#include "../include/ipmi_i2c_driver.h"

#include "../include/ipmb_svc.h"
#include "../stdPeriphLibs/lpc17xx_i2c.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"

#include "../include/sio_usart.h"
#include "../include/rtc.h"

volatile char txBuff[100];
volatile uint8_t* txptr;
volatile ipmi_i2c_state_record_t ipmi_i2c_state;

#define EORBUFSIZE              (8)                             // number of elements in end-of-record buffers

// storage for xmt and rcv queue end-of-record buffers
volatile short eor_xbuf[EORBUFSIZE];
volatile short eor_rbuf[EORBUFSIZE];
volatile uint32_t eor_rbuf_stamp[EORBUFSIZE];
volatile short eor_xcnt = 0;
short eor_xwidx = 0;
volatile short eor_xridx = 0;
volatile short eor_rcnt = 0;
volatile short eor_rwidx = 0;
short eor_rridx = 0;

#define IPMI_I2C_BUFSIZE      (256)                           // number of bytes in TWI transmit and receive buffers

#define IPMI_I2C LPC_I2C0
#define IPMI_CLK 100000

// storage for ipmi_i2c xmt and rcv data buffers
volatile unsigned char ipmi_i2c_xbuf[IPMI_I2C_BUFSIZE];
volatile unsigned char ipmi_i2c_rbuf[IPMI_I2C_BUFSIZE];
volatile uint8_t ipmi_i2c_xcnt = 0;
uint8_t ipmi_i2c_xwidx = 0;
volatile uint8_t ipmi_i2c_xridx = 0;
volatile uint8_t ipmi_i2c_rcnt = 0;
volatile uint8_t ipmi_i2c_rwidx = 0;
uint8_t ipmi_i2c_rridx = 0;
volatile uint8_t ipmi_i2c_xfirst, ipmi_i2c_xlast, ipmi_i2c_xcur, ipmi_i2c_xmtcnt;              // index variables for transmitting message off of the queue

typedef struct {
  enum GApin ga2;
  enum GApin ga1;
  enum GApin ga0;
  unsigned char ipmb_addr;
  unsigned char slotid;
} ga_decode_tbl_entry_t;

#define GA_DECODE_TBL_SIZE                      (27)

const ga_decode_tbl_entry_t ga_decode_tbl[GA_DECODE_TBL_SIZE] = {
                {Gpin, Gpin, Gpin, 0x70, 0xff},
                {Gpin, Gpin, Upin, 0x72, 0x01},
                {Gpin, Upin, Gpin, 0x74, 0x02},
                {Gpin, Upin, Upin, 0x76, 0x03},
                {Upin, Gpin, Gpin, 0x78, 0x04},
                {Upin, Gpin, Upin, 0x7a, 0x05},
                {Upin, Upin, Gpin, 0x7c, 0x06},
                {Upin, Upin, Ppin, 0x7e, 0x07},
                {Upin, Ppin, Upin, 0x80, 0x08},
                {Upin, Ppin, Ppin, 0x82, 0x09},
                {Ppin, Upin, Upin, 0x84, 0x0a},
                {Ppin, Upin, Ppin, 0x86, 0x0b},
                {Ppin, Ppin, Upin, 0x88, 0x0c},
                {Gpin, Gpin, Ppin, 0x8a, 0xff},
                {Gpin, Upin, Ppin, 0x8c, 0xff},
                {Gpin, Ppin, Gpin, 0x8e, 0xff},
                {Gpin, Ppin, Upin, 0x90, 0xff},
                {Gpin, Ppin, Ppin, 0x92, 0xff},
                {Upin, Gpin, Ppin, 0x94, 0xff},
                {Upin, Ppin, Gpin, 0x96, 0xff},
                {Ppin, Gpin, Gpin, 0x98, 0xff},
                {Ppin, Gpin, Upin, 0x9a, 0xff},
                {Ppin, Gpin, Ppin, 0x9c, 0xff},
                {Ppin, Upin, Gpin, 0x9e, 0xff},
                {Ppin, Ppin, Gpin, 0xa0, 0xff},
                {Upin, Upin, Upin, 0xa2, 25},
                {Ppin, Ppin, Ppin, 0xa4, 26} };


 static void put_xmt_eor(short eor_loc);
 static short peek_xmt_next_eor(void);
 static void delete_xmt_eor(void);
 static void put_rcv_eor(short eor_loc);
 static void delete_rcv_eor(void);


 static void put_xmt_byte(unsigned char val);
 static void put_rcv_byte(unsigned char val);
 static unsigned char get_rcv_byte(void);

 static void unload_xmt_msg(void);

 static short get_ipmi_i2c_xmt_next_msg_len(void);

 static uint8_t ipmi_i2c_100uscallback(event evID, void* arg);
 static void get_ipmb_address(volatile unsigned char* ipmbl_addr, volatile unsigned char* slotid);

static void I2CCONSET( uint32_t flags );
static void I2CCONCLR( uint32_t flags );
static void I2CDAT_WRITE( uint8_t val );
static uint8_t I2CDAT_READ( void );

/* Conrol bit manipulation macros */
static void I2CCONSET(uint32_t flags)
{
  IPMI_I2C->I2CONSET = flags;
}

static void I2CCONCLR( uint32_t flags)
{
  IPMI_I2C->I2CONCLR = flags;
}


static void I2CDAT_WRITE( uint8_t val )
{
  IPMI_I2C->I2DAT = val;
}

static uint8_t I2CDAT_READ( )
{
  uint8_t val = 0x00;
  val = (uint8_t)(0x0FF&IPMI_I2C->I2DAT);
  return (val);
}


void I2C0_IRQHandler( void )
{
  NVIC_ClearPendingIRQ(I2C0_IRQn);

 short tempsh;
 unsigned char curbyte;
 uint32_t status;

 unsigned char databyte;

 status = IPMI_I2C->I2STAT;

 switch (status) {

   /* Master transmitter/receiver mode common */
   case I2STAT_START_SENT:
     /* A Start condition has been transmitted as a result of
      * i2c_master_read/write. The slave address + R/W bit will
      * be transmitted, an ACK bit received is expected next.
      *====================================================
      ** load I2DAT with the slave address and the data
      * direction bit (SLA+W or SLA+R). The SI bit in I2CON must
      * then be reset before the serial transfer can continue.
      */
     curbyte = ipmi_i2c_xbuf[ipmi_i2c_xcur];                          // get address byte
     adv_buf_idx(ipmi_i2c_xcur, IPMI_I2C_BUFSIZE);
     ipmi_i2c_xmtcnt--;                                          // decrement transmit count

     I2CDAT_WRITE( curbyte );                                // send destination address (write operation)
     debug_pins_toggle(0);
     I2CCONCLR( I2C_CTRL_FL_SI | I2C_CTRL_FL_STA );

     break;

   case I2STAT_SLAW_SENT_NOT_ACKED:
     ipmi_i2c_start_slave_listen();
     //ipmb_service( );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

   case I2STAT_REP_START_SENT:
     /* not supported - we should not be here */
     ipmi_i2c_start_slave_listen();
     //I2CCONSET( I2C_CTRL_FL_STO );
     //I2CCONCLR( I2C_CTRL_FL_SI );
     break;

   case I2STAT_ARBITRATION_LOST:
     /* Arbitration has been lost during either Slave
      * Address + Write or data. The bus has been released.
      *
      * Due to the nature of the i2c design, we may not know
      * that we may lose the arbitration until the first
      * different bit of either the SLA+RW or data.
      */

     // master loss of arbitration interrupt
     ipmi_i2c_state.master_holdoff_cnt = MASTER_HOLDOFF_DELAY;
     ipmi_i2c_start_slave_listen( );                                              // switch back to slave mode for awhile

     break;

     /* Master transmitter mode */
   case I2STAT_SLAW_SENT_ACKED:
     /* Previous state was State I2STAT_START_SENT or
      * State I2STAT_REP_START_SENT. Slave Address +
      * Write has been transmitted, ACK has been received.
      * The first data byte will be transmitted, an ACK
      * bit will be received.
      * context->ws still points to an outgoing ws
      */
     /* write first byte of data */
     curbyte = ipmi_i2c_xbuf[ipmi_i2c_xcur];                           // get first data byte
     adv_buf_idx(ipmi_i2c_xcur, IPMI_I2C_BUFSIZE);
     ipmi_i2c_xmtcnt--;                                                           // decrement transmit count

     I2CDAT_WRITE( curbyte );                                                // write first data byte--this should start ipmi_i2c transfer

     I2CCONCLR( I2C_CTRL_FL_SI | I2C_CTRL_FL_STA | I2C_CTRL_FL_STO );

     break;

   case I2STAT_MASTER_DATA_SENT_ACKED:
     /* Data has been transmitted, ACK has been received.
      * If the transmitted data was the last data byte
      * then transmit a Stop condition, otherwise transmit
      * the next data byte. */
     // continue sending message, unload when complete

     if (ipmi_i2c_xmtcnt)
       {
         // still some bytes to transmit
         curbyte = ipmi_i2c_xbuf[ipmi_i2c_xcur];                         // get first data byte
         adv_buf_idx(ipmi_i2c_xcur, IPMI_I2C_BUFSIZE);
         ipmi_i2c_xmtcnt--;                                                         // decrement transmit count

         I2CDAT_WRITE( curbyte );

         //  I2CCONSET( I2C_CTRL_FL_AA );
         I2CCONCLR( I2C_CTRL_FL_SI );
       }
     else
       {
         // nothing more to send, arm the transmit complete interrupt
         I2CCONSET( I2C_CTRL_FL_AA | I2C_CTRL_FL_STO );
         I2CCONCLR( I2C_CTRL_FL_SI );

         debug_pins_toggle(0);

         if (ipmi_i2c_state.state == master_write)
           unload_xmt_msg();                                                             // removed completed message from transmit queue


         if (eor_xcnt)
           ipmi_i2c_start_master_mode_write();                                // transmit next message
         else {
           ipmi_i2c_start_slave_listen_fromisr();                                             // go back to slave mode listening
           I2CCONSET( I2C_CTRL_FL_AA | I2C_CTRL_FL_STO );
           I2CCONCLR( I2C_CTRL_FL_SI);
         }

       }
     break;

   case I2STAT_MASTER_DATA_SENT_NOT_ACKED:
     /* Data has been transmitted, NOT ACK received.
      * Send a STOP condition & enter not adressed slave mode.
      */

	 debug_pins_toggle(0);
     ipmi_i2c_start_slave_listen_fromisr();
//     I2CCONSET( I2C_CTRL_FL_AA | I2C_CTRL_FL_STO );
     I2CCONSET( I2C_CTRL_FL_AA | I2C_CTRL_FL_STO );
     I2CCONCLR( I2C_CTRL_FL_SI);



     break;

   case I2STAT_STOP_START_RCVD:

     if (ipmi_i2c_state.state == slave_write)
       {
         // add end-of-record entry, marking end of received message (back up one spot to get last byte index)
         if (ipmi_i2c_rwidx)
           tempsh = ipmi_i2c_rwidx-1;
         else
           tempsh = IPMI_I2C_BUFSIZE-1;
         put_rcv_eor(tempsh);                                                    // put index of last message byte in end-of-record queue
       }

     ipmi_i2c_start_slave_listen();

     /* Set AA flag & clear SI */
     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

     /* Slave receiver mode*/
   case I2STAT_SLAW_RCVD_ACKED:
     /* Own SLA+W has been rcvd; ACK has been returned. */
     ipmi_i2c_state.state = slave_write;

     /* set AA flag to send ACK & clear interrupt bit SI */
     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );

     put_rcv_byte( ipmi_i2c_state.ipmbl_addr );           // put slave address at front of message as rsSA field of message

     break;

   case I2STAT_SLAR_RCVD_ACKED:
     ipmi_i2c_state.state = slave_read;
     I2CDAT_WRITE(0xFF);
     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

   case I2STAT_SLAVE_DATA_SENT_ACKED:

     I2CDAT_WRITE(0xFF);
     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

   case I2STAT_SLAVE_DATA_RCVD_ACKED:
   case I2STAT_GENERAL_CALL_DATA_RCVD_ACKED:

     databyte = I2CDAT_READ();
     put_rcv_byte( databyte );               // put slave address at front of message as rsSA field of message

     /* set AA flag to get next data byte */
     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );

     break;

   case I2STAT_ARB_LOST_SLAW_RCVD_ACKED:
   case I2STAT_ARB_LOST_GENERAL_CALL_RCVD_ACKED:

     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );
     ipmi_i2c_state.master_holdoff_cnt = MASTER_HOLDOFF_DELAY;
     ipmi_i2c_start_slave_listen( );                                              // switch back to slave mode for awhile

     break;

   case I2STAT_GENERAL_CALL_DATA_RCVD_NOT_ACKED:
   case I2STAT_SLAVE_DATA_RCVD_NOT_ACKED:
     /* Previously addressed with own Slave Address. Data
      * has been received and NOT ACK has been returned.
      * Received data will not be saved. Master will send
      * a STOP. */

     I2CCONSET( I2C_CTRL_FL_AA );
     I2CCONCLR( I2C_CTRL_FL_SI );
     ipmi_i2c_start_slave_listen( );                                              // switch back to slave mode for awhile
 //    ipmb_service();

     break;

   default:
     ipmi_i2c_start_slave_listen( );  // switch back to slave mode for awhile

     break;
   }
// ipmb_service();
}



void ipmi_i2c_init(void)
{

  I2C_OWNSLAVEADDR_CFG_Type I2C_OwnSlaveAddr_CFG_Struct;
  ipmi_i2c_state.master_holdoff_cnt = 0;

  //ipmi_i2c_state.xfer_cnt = 0;
  get_ipmb_address(&(ipmi_i2c_state.ipmbl_addr), &(ipmi_i2c_state.slotid));

  //do general initialization of ipmi_i2c, but leave in disabled state
  I2C_Init(IPMI_I2C, IPMI_CLK);

  I2C_OwnSlaveAddr_CFG_Struct.SlaveAddr_7bit = ipmi_i2c_state.ipmbl_addr;
  I2C_OwnSlaveAddr_CFG_Struct.GeneralCallState = ENABLE;
  I2C_OwnSlaveAddr_CFG_Struct.SlaveAddrChannel = 0;
  I2C_OwnSlaveAddr_CFG_Struct.SlaveAddrMaskValue = 0xFE;

  I2C_SetOwnSlaveAddr(IPMI_I2C, &I2C_OwnSlaveAddr_CFG_Struct);

  register_timer_callback(ipmi_i2c_100uscallback, TEVENT_100USEC);                    // register callback for 200usec service routine

  I2C_IntCmd(IPMI_I2C, ENABLE);

}


unsigned int get_ipmi_i2c_rcv_msg_cnt(void)
{
  return ( (unsigned int) eor_rcnt );
}

uint32_t get_ipmi_i2c_rcv_next_msg_timestamp(void)
{
	  if (!eor_rcnt)
	        return (0);

	  return eor_rbuf_stamp[eor_rridx];

}

short get_ipmi_i2c_rcv_next_msg_len(void)
{
  // examines next message on rcv queue and determines its length
  // returns 0 if there are no complete messages on queue
  short eoridx;
  if (!eor_rcnt)
        return (0);
  eoridx = eor_rbuf[eor_rridx];
  // calculate length of msg on queue
  if (ipmi_i2c_rridx > eoridx)
     return (IPMI_I2C_BUFSIZE + 1 + eoridx - ipmi_i2c_rridx);                        // message wraps around in ipmi_i2c buffer
  else
        return (eoridx + 1 - ipmi_i2c_rridx);
}


static short get_ipmi_i2c_xmt_next_msg_len(void)
{
  // examines next message on xmt queue and determines its length
  // returns 0 if there are no complete messages on queue
  short eoridx;
  short retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  if (!eor_xcnt)
        retval = 0;
  else
    {
    eoridx = eor_xbuf[eor_xridx];
    // calculate length of msg on queue
    if (ipmi_i2c_xridx > eoridx)
      retval = IPMI_I2C_BUFSIZE + 1 + eoridx - ipmi_i2c_xridx;                     // message wraps around in ipmi_i2c buffer
    else
          retval = eoridx + 1 - ipmi_i2c_xridx;
    }

  Int_Restore(giflag);

  return (retval);
}


unsigned int get_ipmi_i2c_msg(volatile unsigned char* rbuf, volatile unsigned short* rlen, volatile unsigned short bufsize, uint32_t * timestamp)
{
  // check receive message queue, and copy oldest message to the supplied buffer
  // returns 1 if message is dequeued and copied, and 0 if there is no message
  // NOTE:  if the message is longer than the buffer, then function dequeues the
  //   *entire* message from the buffer, but only copies as many bytes as indicated
  //   by the 'bufsize' argument.  Remaining bytes are discarded.  This should cause
  //   checksum errors in the message
  short msglen, getcnt, copycnt;
  volatile unsigned char* bptr;

  msglen = get_ipmi_i2c_rcv_next_msg_len();                                          // get length of next message in buffer
  if (!msglen)
          // no complete messages to get
          return (0);

  getcnt = 0;
  bptr = rbuf;
  // figure number of bytes to copy to buffer
  if (msglen > bufsize)
          copycnt = bufsize;
  else
          copycnt = msglen;
  while (getcnt < copycnt)
      {
          // copy characters to buffer
          *bptr++ = get_rcv_byte();
          getcnt++;
      }
  *rlen = getcnt;
  *timestamp = get_ipmi_i2c_rcv_next_msg_timestamp();
 // debug_pins_set(1,0);

//////////////////////////////////////////////////////
//       It could make system hang
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//  while (getcnt < msglen)
//          // flush excess characters if any
//    get_rcv_byte();
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
  delete_rcv_eor();                                                                                     // remove entry from eor queue--this drops the rcv msg cnt
  return (1);
}

void unload_xmt_msg(void)
{
  // this function unloads the next transmit message from the front of the transmit queue
  // this is done by updating the index variables and count
  short eor_val;
  Bool giflag = Is_global_interrupt_enabled();
  if (!eor_xcnt)
        // no complete message to unload
        return;
  Int_Disable(giflag);
  ipmi_i2c_xcnt -= get_ipmi_i2c_xmt_next_msg_len();       // decrement count by length of message
  eor_val = peek_xmt_next_eor();                        // get end-of-record mark from xmt queue
  delete_xmt_eor();                                                     // remove end-of-record mark from queue
  ipmi_i2c_xridx = eor_val;
  adv_buf_idx(ipmi_i2c_xridx, IPMI_I2C_BUFSIZE);
  Int_Restore(giflag);
}

unsigned int put_ipmi_i2c_msg(volatile unsigned char* xbuf, volatile unsigned short xlen)
{
  // function returns 1 if message is copied to end of transmit queue,
  // otherwise returns 0
  short putcnt;
  unsigned char* bptr;
  Bool giflag = Is_global_interrupt_enabled();

  // check for space in transmit queue
  if (xlen > chk_ipmi_i2c_xbuf_space())
        // not enough space, abort put
        return (0);

  Int_Disable(giflag);
  bptr = xbuf;
  putcnt = 0;
  while (putcnt < xlen-1) {
        // put all but last character on queue
        put_xmt_byte(*bptr);
        bptr++;
        putcnt++;
  }
  // write current value of xmt queue write index to transmit eor queue, then put the last character
  put_xmt_eor(ipmi_i2c_xwidx);                       // this is index of last byte to be written to queue
  put_xmt_byte(*bptr);

  if (OK_to_enter_master_mode()) {
	//debug_pins_set(0,1);
    ipmi_i2c_start_master_mode_write();
  }
  Int_Restore(giflag);
  return (1);
}


unsigned int chk_ipmi_i2c_xbuf_space(void) {
  // function returns the amount of space in bytes currently available in transmit buffer
  unsigned int retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  if (ipmi_i2c_xwidx >= ipmi_i2c_xridx)
        retval = IPMI_I2C_BUFSIZE-(ipmi_i2c_xwidx-ipmi_i2c_xridx);
  else
        retval = ipmi_i2c_xridx-ipmi_i2c_xwidx;
  Int_Restore(giflag);
  return (retval);
}




 static void put_xmt_eor(short eor_loc)
 {
  Bool giflag;
  if (eor_xcnt >= EORBUFSIZE-1)
        // no room on queue
        return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  eor_xbuf[eor_xwidx] = eor_loc;
  eor_xcnt++;
  adv_buf_idx(eor_xwidx, EORBUFSIZE);
  Int_Restore(giflag);
}


 static short peek_xmt_next_eor(void) {
  short retval;
  Bool giflag;
  if (!eor_xcnt)
        // nothing in queue
        return (-1);
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  retval = eor_xbuf[eor_xridx];
  Int_Restore(giflag);
  return (retval);

}


 static void delete_xmt_eor(void)
 {
  Bool giflag;
  if (!eor_xcnt)
        // nothing in queue
        return;

  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  eor_xcnt--;
  adv_buf_idx(eor_xridx, EORBUFSIZE);

  Int_Restore(giflag);
}


 static void put_rcv_eor(short eor_loc)
 {
  Bool giflag;

  if (eor_rcnt >= EORBUFSIZE-1)
        // no room on queue
        return;

  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  eor_rbuf[eor_rwidx] = eor_loc;
  eor_rbuf_stamp[eor_rwidx] = get_rtc_value();
 // debug_pins_set(1,1);
  eor_rcnt++;
  adv_buf_idx(eor_rwidx, EORBUFSIZE);

  Int_Restore(giflag);
}


 static void delete_rcv_eor(void)
 {
  Bool giflag;
  if (!eor_rcnt)
        // nothing in queue
        return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  eor_rcnt--;
  adv_buf_idx(eor_rridx, EORBUFSIZE);
  Int_Restore(giflag);
}


 static void put_xmt_byte(unsigned char val)
 {
  Bool giflag;
  if (ipmi_i2c_xcnt == IPMI_I2C_BUFSIZE)
        // no room on queue
        return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  ipmi_i2c_xbuf[ipmi_i2c_xwidx] = val;
  ipmi_i2c_xcnt++;
  adv_buf_idx(ipmi_i2c_xwidx, IPMI_I2C_BUFSIZE);
  Int_Restore(giflag);
}


 static void put_rcv_byte(unsigned char val)
 {
  Bool giflag;
  if (ipmi_i2c_rcnt == IPMI_I2C_BUFSIZE)
        // no room on queue
        return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  ipmi_i2c_rbuf[ipmi_i2c_rwidx] = val;
  ipmi_i2c_rcnt++;
  adv_buf_idx(ipmi_i2c_rwidx, IPMI_I2C_BUFSIZE);
  Int_Restore(giflag);
}


 unsigned char get_rcv_byte(void) {
  unsigned char retval;
  Bool giflag;
  if (!ipmi_i2c_rcnt)
        // nothing in queue
        return (-1);
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  retval = ipmi_i2c_rbuf[ipmi_i2c_rridx];
  ipmi_i2c_rcnt--;
  adv_buf_idx(ipmi_i2c_rridx, IPMI_I2C_BUFSIZE);
  Int_Restore(giflag);
  return (retval);
}


 void ipmi_i2c_start_slave_listen_fromisr(void) {
   // configure ipmi_i2c for slave mode listen
   ipmi_i2c_state.state = slave_listen;
 }


void ipmi_i2c_start_slave_listen(void) {
  // configure ipmi_i2c for slave mode listen
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  ipmi_i2c_state.state = slave_listen;

  I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_STO | I2C_CTRL_FL_SI );
  I2CCONSET( I2C_CTRL_FL_I2EN | I2C_CTRL_FL_AA );
  // enable slave access interrupt

  Int_Restore(giflag);
}


int OK_to_enter_master_mode(void)
{
  // function returns 1 if it is OK to switch into master mode and begin a write operation
  // returns 0 otherwise
  int retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  if ((ipmi_i2c_state.state == slave_listen) && (ipmi_i2c_state.master_holdoff_cnt == 0) && (eor_xcnt > 0))
    retval = 1;
  else
    retval = 0;
  Int_Restore(giflag);
  return (retval);
}


void ipmi_i2c_start_master_mode_write(void) {
  // function starts a master mode write
  // IMPORTANT--function assumes all necessary checks (ipmi_i2c state, xmt queue, holdoff ctr) have been
  // performed as necessary *before* making this call
//  unsigned char curbyte;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  ipmi_i2c_state.state = master_write;                                       // update state machine state

  ipmi_i2c_xfirst = ipmi_i2c_xridx;                                       // set first index of message at tail of queue
  ipmi_i2c_xmtcnt = get_ipmi_i2c_xmt_next_msg_len();      // set count to message length
  ipmi_i2c_xlast = peek_xmt_next_eor();                      // set last index at the end-of-record
  ipmi_i2c_xcur = ipmi_i2c_xfirst;                                        // set current index variable

  //Sending Start bit next steps will be done in IRQ
  Int_Restore(giflag);
  I2CCONCLR( I2C_CTRL_FL_STA | I2C_CTRL_FL_SI | I2C_CTRL_FL_STO | I2C_CTRL_FL_AA  );
  I2CCONSET( I2C_CTRL_FL_STA | I2C_CTRL_FL_I2EN );


}


uint8_t ipmi_i2c_100uscallback(event evID, void* arg) {
  // 100usec interval callback for ipmi_i2c driver
  // This callback is used to provide a holdoff time in which the ipmi_i2c interface
  // remains in slave mode, after a master mode loss-of-arbitration event.
  if (ipmi_i2c_state.master_holdoff_cnt) {
        ipmi_i2c_state.master_holdoff_cnt--;
    if (!ipmi_i2c_state.master_holdoff_cnt)
      // loss-of-arbitration holdoff expired, check to see if ipmi_i2c ready for transmit
      if (OK_to_enter_master_mode())
        ipmi_i2c_start_master_mode_write();                  // ipmi_i2c idle in slave listen, msg pending, so start transmit
  }
  return (1);
}

#define P1DELAYCNT              (1000)                          // for loop interations waiting for P1 assertion to take effect

void get_ipmb_address(volatile unsigned char* ipmbl_addr, volatile unsigned char* slotid)
{
  int i1;
  uint8_t ga0t0, ga1t0, ga2t0, ga0t1, ga1t1, ga2t1;
  //enum GApin ga0val, ga1val, ga2val;
  //unsigned long readpvr;
  const ga_decode_tbl_entry_t* pdectbl;



  gpio_clr_gpio_pin( IPMI_P1, IPMI_P1_PORT );

  ga0t0 = gpio_get_pin_value( IPMI_GA0, IPMI_GA_PORT );
  ga1t0 = gpio_get_pin_value( IPMI_GA1, IPMI_GA_PORT );
  ga2t0 = gpio_get_pin_value( IPMI_GA2, IPMI_GA_PORT );

  gpio_set_gpio_pin( IPMI_P1, IPMI_P1_PORT );

  ga0t1 = gpio_get_pin_value( IPMI_GA0, IPMI_GA_PORT );
  ga1t1 = gpio_get_pin_value( IPMI_GA1, IPMI_GA_PORT );
  ga2t1 = gpio_get_pin_value( IPMI_GA2, IPMI_GA_PORT );

  // compare test results to compute IPMI  and slot addresses
  if ((ga0t0 == 0) && (ga0t1 == 1))
        // ga0 unconnected
        ipmi_i2c_state.ga0val = Upin;
  else if ((ga0t0 == 1) && (ga0t1 == 1))
        // ga0 pulled up to power
        ipmi_i2c_state.ga0val = Ppin;
  else
        ipmi_i2c_state.ga0val = Gpin;
  if ((ga1t0 == 0) && (ga1t1 == 1))
        // ga1 unconnected
        ipmi_i2c_state.ga1val = Upin;
  else if ((ga1t0 == 1) && (ga1t1 == 1))
        // ga1 pulled up to power
        ipmi_i2c_state.ga1val = Ppin;
  else
        ipmi_i2c_state.ga1val = Gpin;
  if ((ga2t0 == 0) && (ga2t1 == 1))
        // ga2 unconnected
        ipmi_i2c_state.ga2val = Upin;
  else if ((ga2t0 == 1) && (ga2t1 == 1))
        // ga2 pulled up to power
        ipmi_i2c_state.ga2val = Ppin;
  else
        ipmi_i2c_state.ga2val = Gpin;
  // scan table to get addresses
  for (i1=0; i1<GA_DECODE_TBL_SIZE; i1++) {
        pdectbl = &ga_decode_tbl[i1];
        if ((ipmi_i2c_state.ga0val == pdectbl->ga0) && (ipmi_i2c_state.ga1val == pdectbl->ga1) && (ipmi_i2c_state.ga2val == pdectbl->ga2)) {
          // have match
        	//printf("IPMI addr %02x\r\n", pdectbl->ipmb_addr);
          *ipmbl_addr = pdectbl->ipmb_addr;
          *slotid = pdectbl->slotid;

          return;
        }
  }
  // because the table fully defines the possible search space, we should never get here.  But if we do,
  // return some consistent (if distressing) values
  *ipmbl_addr = 0xff;
  *slotid = 0xff;
}

