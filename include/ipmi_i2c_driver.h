#ifndef IPMI_I2C_DRIVER_H_
#define IPMI_I2C_DRIVER_H_

#define MASTER_HOLDOFF_DELAY                    (2)             // number of 200usec ticks to hold off master mode on loss-of-arbitration
#define adv_buf_idx(i,b)                        { i = (b-1-i) ? i+1 : 0; }

enum IPMI_I2C_STATE {disabled = 0, slave_listen, slave_write, slave_read, master_write, failed};
enum GApin {Gpin = 0, Upin, Ppin};

typedef struct {
  enum IPMI_I2C_STATE state;
  unsigned char master_holdoff_cnt;
  unsigned char ipmbl_addr;
  unsigned char slotid;
  enum GApin ga0val;
  enum GApin ga1val;
  enum GApin ga2val;
} ipmi_i2c_state_record_t;

extern volatile ipmi_i2c_state_record_t ipmi_i2c_state;

void ipmi_i2c_init(void);

unsigned int get_ipmi_i2c_rcv_msg_cnt(void);

short get_ipmi_i2c_rcv_next_msg_len(void);

unsigned int get_ipmi_i2c_msg(volatile unsigned char* rbuf, volatile unsigned short* rlen, volatile unsigned short bufsize, uint32_t * timestamp);

unsigned int put_ipmi_i2c_msg(volatile unsigned char* xbuf, volatile unsigned short xlen);

unsigned int chk_ipmi_i2c_xbuf_space(void);

void ipmi_i2c_start_slave_listen(void);

int OK_to_enter_master_mode(void);

void ipmi_i2c_start_master_mode_write(void);

void I2C0_IRQHandler( void );

#endif
