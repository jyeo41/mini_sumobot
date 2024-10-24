#ifndef SYSTICK_H_
#define SYSTICK_H_
#include <stm32f4xx.h>

void systick_initialize(void);
void systick_delay_ms(uint32_t delay);

#endif /* SYSTICK_H_ */
