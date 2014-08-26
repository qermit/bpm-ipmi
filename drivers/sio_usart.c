
#include "LPC17xx.h"

#include "../stdPeriphLibs/lpc17xx_uart.h"
#include "../stdPeriphLibs/lpc17xx_pinsel.h"

#include "../include/sio_usart.h"
#include "../include/utils.h"

#include "../include/interruptPriorities.h"

#define checkIfFifoEmpty        ( ( LPC_UART3->LSR & 0x20 ) ? 1 : 0)
#define resetFifo               ( LPC_UART3->FCR |= 0x02 )
#define checkIfDataToSend       ( ( uartDataToSend.currPtr != uartDataToSend.lastPtr ) ? 1 : 0)

LPC_UART_TypeDef *SERIAL_USART = (LPC_UART_TypeDef *)LPC_UART3;

volatile char spbuf[SPRINTBUFSIZE];

volatile UartTxStruct_t uartDataToSend;

uint8_t filtermask = TXTFILT_EVENTS | TXTFILT_IPMI_STD_REQ |  TXTFILT_IPIM_CUSTOM_REQ | /*
  TXTFILT_DBG_SUMMARY | TXTFILT_DBG_DETAIL |*/ TXTFILT_INFO;              // everything turned on by default


static void sendDataToFifo( void );


void UART3_IRQHandler(void)
{
  volatile uint32_t intId = UART_GetIntId(SERIAL_USART);

  if(intId & 0x02)
    sendDataToFifo();

  NVIC_ClearPendingIRQ(UART3_IRQn);

}


void sio_init(void)
{
	//LPC_USART_3
	PINSEL_CFG_Type PinSelCfg;
	UART_CFG_Type UartCFG_Struct;
	UART_FIFO_CFG_Type UART_FIFO_CFG_Struct;


	// P4_28
	PinSelCfg.Portnum = PINSEL_PORT_4;
	PinSelCfg.Pinnum = PINSEL_PIN_28;
	PinSelCfg.Funcnum = PINSEL_FUNC_3;
	PinSelCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinSelCfg.Pinmode = PINSEL_PINMODE_PULLUP;
	PINSEL_ConfigPin(&PinSelCfg);
	// P4_29
	PinSelCfg.Pinnum = PINSEL_PIN_29;
	PINSEL_ConfigPin(&PinSelCfg);

	uartDataToSend.currPtr = 0;
	uartDataToSend.lastPtr = 0;
	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 9600bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	*/
	UART_ConfigStructInit(&UartCFG_Struct);

	/* Set Baudrate to 115200 */
	UartCFG_Struct.Baud_rate = 115200;

	/* Initialize UART3 peripheral with given to corresponding parameter */
	UART_Init(SERIAL_USART, &UartCFG_Struct);

	/* Initialize FIFOConfigStruct to default state:
	 * 				- FIFO_DMAMode = DISABLE
	 * 				- FIFO_Level = UART_FIFO_TRGLEV0
	 * 				- FIFO_ResetRxBuf = ENABLE
	 * 				- FIFO_ResetTxBuf = ENABLE
	 * 				- FIFO_State = ENABLE
	 */
	UART_FIFOConfigStructInit(&UART_FIFO_CFG_Struct);

	/* Initialize FIFO for UART3 peripheral */
	UART_FIFOConfig(SERIAL_USART, &UART_FIFO_CFG_Struct);

	/*  Enable UART Transmit */
	UART_TxCmd(SERIAL_USART, ENABLE);

	UART_IntConfig(SERIAL_USART, UART_INTCFG_THRE, ENABLE);

	NVIC_SetPriorityGrouping(UART3_PriorGrup);
	NVIC_SetPriority(UART3_IRQn, UART3_Prior);

	NVIC_EnableIRQ(UART3_IRQn);


}


static void sendDataToFifo( void )
{
  uint8_t byteSend = 0;

  if( checkIfFifoEmpty  & checkIfDataToSend )
    {
      resetFifo;

      while( checkIfDataToSend & (byteSend < UART_TX_FIFO_SIZE) )
        {
          UART_SendByte(SERIAL_USART, (uartDataToSend.txbuf[uartDataToSend.currPtr++]) );
          byteSend++;
        }
    }
}


void sio_putstr(char *cptr)
{
  uint8_t byteSend = 0;

  Disable_global_interrupt();

  if( checkIfFifoEmpty  & (!checkIfDataToSend) )
    { // nothing in queue data can be transmited directly
      // to uart FIFO
      resetFifo;

      while( (*cptr!='\0') & (byteSend < UART_TX_FIFO_SIZE) )
        {
          UART_SendByte(SERIAL_USART, (*cptr++) );
          byteSend++;
        }
    }
  //FIFO full save data in buffer
  while(*cptr)
    {
      uartDataToSend.txbuf[uartDataToSend.lastPtr++] = (*cptr++);
      byteSend = 0xff;
    }

  if(byteSend == 0xff)
    {
      uartDataToSend.txbuf[uartDataToSend.lastPtr] = '\0';
    }

  Enable_global_interrupt();

  return;
}



void sio_filt_putstr(const unsigned char filtcode, int addtag, char *cptr)
{
// fiters the string put operation, per the filtcode argument and the current mask settings.
// The string put occurs only if the argument is in the active mask.  If the addtag argment is
// nonzero, then a prefix string indicating the message type is added.
//  unsigned char mask = filtcode & filtermask;

  if (addtag)
    {
      switch(filtcode)
      {
      case (TXTFILT_EVENTS):
          sio_putstr("[IPMIEVT] ");
      break;

      case (TXTFILT_IPMI_STD_REQ):
          sio_putstr("[IPMISTD] ");
      break;

      case ( TXTFILT_IPIM_CUSTOM_REQ):
          sio_putstr("[IPMICSTM] ");
      break;

      case ( TXTFILT_DBG_SUMMARY):
          sio_putstr("[DBGSUM] ");
      break;

      case ( TXTFILT_DBG_DETAIL):
          sio_putstr("[DBGDTL] ");
      break;

      case ( TXTFILT_INFO):
          sio_putstr("[INFO] ");
      break;

      default:
        break;
      }
    }

  sio_putstr(cptr);
}
