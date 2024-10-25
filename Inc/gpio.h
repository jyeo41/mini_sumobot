#ifndef GPIO_H_
#define GPIO_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

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

/* Struct that holds all the attributes of a single pin. This struct is the basis for abstracting away
 * direct register reads and writes in the custom GPIO HAL.
 * Pin name is needed because it'll allow accessing the named pin from the gpio_board_pins[] array */
typedef struct {
	gpio_pin_names_e pin_name;
	GPIO_TypeDef* port;
	uint8_t pin_number;
	uint8_t mode;
}gpio_pin_t;

void gpio_default_initialize(void);
bool gpio_config_compare(gpio_pin_names_e pin_to_check,
			 gpio_pin_names_e expected_pin_name,
			 const GPIO_TypeDef* expected_port,
			 uint8_t expected_pin_number,
			 uint8_t expected_mode);
void gpio_mode_set(gpio_pin_names_e pin, gpio_mode_e mode);
void gpio_data_output_toggle(gpio_pin_names_e pin_name);
void gpio_data_output_set(gpio_pin_names_e pin_name);
void gpio_data_output_clear(gpio_pin_names_e pin_name);
#endif /* GPIO_H_ */
