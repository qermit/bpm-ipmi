

#ifndef LEDDRIVERS_H_
#define LEDDRIVERS_H_

#include "gpio.h"

// macros for driving LEDs directly from code.  Useful for debugging, but recommended method
// for production code is to use activity descriptors & the timer-based driver
#define LEDOFF				(1)
#define LEDON				(0)

#define LED_on(pin)			(gpio_clr_gpio_pin(pin, LED_PORT))
#define LED_off(pin)		(gpio_set_gpio_pin(pin, LED_PORT))
#define LED_tog(pin)		(gpio_tgl_gpio_pin(pin, LED_PORT))
#define LED_set_state(pin, val)		{if (val==LEDOFF) LED_off(pin); else LED_on(pin); }

#define LED_is_off(pin)     (gpio_get_pin_value(pin, LED_PORT))
#define LED_is_on(pin)      (!gpio_get_pin_value(pin, LED_PORT))
#define LED_get_state(pin)	((uint8_t) gpio_get_pin_value(pin, LED_PORT))


// PICMG-defined color codes for set/get LED state commands
#define LEDCOLOR_BLUE					(1)
#define LEDCOLOR_RED					(2)
#define LEDCOLOR_GREEN					(3)
#define LEDCOLOR_AMBER					(4)
#define LEDCOLOR_ORANGE					(5)
#define LEDCOLOR_WHITE					(6)
#define LEDCOLOR_NOCHANGE				(0xe)
#define LEDCOLOR_DEFAULT				(0xf)

#define LED_CNT							(3)				// was 7 changed for 3
#define IPMI_LED_CNT					(3)				// count of IPMI LEDs in table
#define IPMI_BLUELED_TBL_IDX			(0)
#define IPMI_LED1_TBL_IDX				(1)
#define IPMI_LED2_TBL_IDX				(2)
#define LED0_TBL_IDX					(3)
#define LED1_TBL_IDX					(4)
#define LED2_TBL_IDX					(5)
#define LED3_TBL_IDX					(6)

typedef enum {Local_Control=0, Override} LEDstate_t;

typedef enum {On=0, Off, Blink, Bypass} LEDactivity_t;

typedef struct {
  LEDactivity_t action;
  uint8_t initstate;						// initial state, either LEDOFF or LEDON
  uint8_t delay1;					// period in initial state in 10ms units
  uint8_t delay2;					// period in opposite state in 10ms units, 0=skip
} LED_activity_desc_t;


typedef struct {
  LEDstate_t LEDstate;
  const LED_activity_desc_t* pLocalDesc;
  const LED_activity_desc_t* pOvrideDesc;
  uint8_t LocalPscale;
  uint8_t OvridePscale;
  uint8_t Color;
  uint8_t Pin;							// GPIO pin identifier
} LED_state_rec_t;


extern const LED_activity_desc_t LED_Off_Activity;
extern const LED_activity_desc_t LED_On_Activity;
extern const LED_activity_desc_t LED_50ms_Flash_Activity;
extern const LED_activity_desc_t LED_1Hz_Blink_Activity;
extern const LED_activity_desc_t LED_driver_bypass;
extern const LED_activity_desc_t LED_2Hz_Blink_Activity;
extern const LED_activity_desc_t LED_Noncritical_Activity;

extern volatile LED_state_rec_t LEDstate[LED_CNT];

void program_LED(uint8_t LEDID, LEDstate_t newLEDstate, const LED_activity_desc_t* pLEDact);

void LED_init(void);


#endif /* LEDDRIVERS_H_ */
