
#ifndef UTILS_H_
#define UTILS_H_

#include "../stdPeriphLibs/lpc_types.h"

void Disable_global_interrupt(void);
void Enable_global_interrupt(void);

Bool Is_global_interrupt_enabled(void);
void Int_Disable(Bool giflag);
void Int_Restore(Bool giflag);

void Enable_Watchdog(void);
void Disable_Watchdog(void);

#endif /* UTILS_H_ */
