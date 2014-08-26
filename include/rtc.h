
#ifndef RTC_H_
#define RTC_H_



extern volatile uint32_t systemUpTime;

//void rtc_init(void);

uint32_t get_rtc_value(void);

void set_rtc_value(uint32_t systime);

//void TIMER1_IRQHandler (void);



#endif /* RTC_H_ */
