#ifndef SIO_USART_H_
#define SIO_USART_H_

#include <stdint.h>

#define UART_TX_BUFF_SIZE (256)

#define SPRINTBUFSIZE (128)

// console text filter flags
#define TXTFILT_EVENTS            (0x01)      // outgoing IPMI event messages
#define TXTFILT_IPMI_STD_REQ      (0x02)      // incoming IPMI standard requests
#define TXTFILT_IPIM_CUSTOM_REQ   (0x04)      // incoming IPMI custom requests
#define TXTFILT_DBG_SUMMARY       (0x40)      // debug level 1 messages (summary)
#define TXTFILT_DBG_DETAIL        (0x80)      // debug level 2 messages (detail)
#define TXTFILT_INFO              (0x08)      // general notifications

typedef struct
{
  uint8_t currPtr;
  uint8_t lastPtr;
  uint8_t txbuf[UART_TX_BUFF_SIZE];

}UartTxStruct_t;


extern  volatile char spbuf[SPRINTBUFSIZE];

void sio_init(void);

void sio_putstr(char *cptr);

void sio_filt_putstr(const unsigned char filtcode, int addtag, char *cptr);

void UART3_IRQHandler(void);

#endif /* SIO_USART_H_ */
