#ifndef GPIO_INIT_H_
#define GPIO_INIT_H_

#include <stdint.h>


void gpio_init();

void gpio_set_gpio_pin(unsigned int pin, uint8_t port);
void gpio_clr_gpio_pin(unsigned int pin, uint8_t port);
void gpio_tgl_gpio_pin(unsigned int pin, uint8_t port);
void gpio_disable_gpio_pin_output(unsigned int pin, uint8_t port);
uint8_t gpio_get_pin_value(unsigned int pin, uint8_t port);

//Ctrl pins
void reset_FPGA( void );
uint8_t checkDONEpin( void );
uint8_t checkPGOODpin( void );
uint8_t checkMasterRESETpin( void );

void setDC_DC_ConvertersON( void );
void setDC_DC_ConvertersOFF( void );

uint8_t getRTM_Presence( void );
void setRTM_ConnectorON( void );
void setRTM_ConnectorOFF( void );

#endif /* GPIO_INIT_H_ */
