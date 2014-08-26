
#ifndef EJECTHANDLE_H_
#define EJECTHANDLE_H_

#include "events.h"

typedef enum {in_closed=0, out_open, indeterminate} Eject_Hdl_Pos;


extern volatile Eject_Hdl_Pos          HandleStableState;

extern volatile ptrDriverISRCallback   hdlchgcallbackptr;

extern volatile uint8_t ejcetHandleFlag;


void eject_handle_init(void);

void EINT3_IRQHandler(void);

void ejcetHandleChange( void );

void register_eject_handle_change_callback(ptrDriverISRCallback funcaddr);

Eject_Hdl_Pos get_eject_handle_stable_state(void);


#endif /* EJECTHANDLE_H_ */
