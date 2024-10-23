#ifndef GPIO_H_
#define GPIO_H_

#include <stm32f4xx.h>
#include <stdint.h>

typedef enum {
	LED_GREEN,
	LED_ORANGE,
	LED_RED,
	LED_BLUE,
}gpio_pin_names_e;

typedef enum {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE,
	GPIO_MODE_ANALOG,
}gpio_mode_e;

typedef struct {
	GPIO_TypeDef* port;
	uint8_t pin_number;
	uint8_t mode;
}gpio_pin_t;

void gpio_mode_set(gpio_pin_names_e pin, gpio_mode_e mode);
void gpio_data_output_toggle(gpio_pin_names_e pin_name);
void gpio_data_output_set(gpio_pin_names_e pin_name);
void gpio_data_output_clear(gpio_pin_names_e pin_name);
#endif /* GPIO_H_ */
