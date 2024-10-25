#ifndef LED_H_
#define LED_H_

#include "gpio.h"

void led_initialize(void);
void led_toggle(gpio_pin_names_e led_color);
#endif /* LED_H_ */
