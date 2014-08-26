/*
 * interruptPriorities.h
 *
 *  Created on: 12-09-2013
 *      Author: Bartek
 */

#ifndef INTERRUPTPRIORITIES_H_
#define INTERRUPTPRIORITIES_H_

#define IPMI_I2C_PriorGrup      0
#define	IPMI_I2C_Prior		1

#define TIMER0_PriorGrup        0
#define TIMER0_Prior            2  // software call backs

#define IPMI_EJCET_PriorGrup    0
#define IPMI_EJCET_Prior        3


#define I2C1_PriorGrup          1
#define I2C1_Prior              0

#define TIMER3_PriorGrup        1
#define TIMER3_Prior            1


#define UART3_PriorGrup         3
#define UART3_Prior             1


#endif /* INTERRUPTPRIORITIES_H_ */
