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
	UART2_BOARD_TX,
}gpio_pin_names_e;

typedef enum {
	GPIO_MODE_INPUT,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE,
	GPIO_MODE_ANALOG,
}gpio_mode_e;

typedef enum {
	GPIO_AF0,
	GPIO_AF1,
	GPIO_AF2,
	GPIO_AF3,
	GPIO_AF4,
	GPIO_AF5,
	GPIO_AF6,
	GPIO_AF7,
	GPIO_AF8,
	GPIO_AF9,
	GPIO_AF10,
	GPIO_AF11,
	GPIO_AF12,
	GPIO_AF13,
	GPIO_AF14,
	GPIO_AF15,
}gpio_alternate_function_e;

/* Struct that holds all the attributes of a single pin. This struct is the basis for abstracting away
 * direct register reads and writes in the custom GPIO HAL.
 * Pin name is needed because it'll allow accessing the named pin from the gpio_board_pins[] array 
 *
 * Pin name, port, and pin number are all set to constant because these should not change once set.
 * "GPIO_TypeDef* const" means its a CONSTANT POINTER read from right to left:
 *	This is an important distinction from "const GPIO_TYPEDEF*".
 *	GPIO_TypeDef* const is what we want because it means the values inside of the registers of the ports can be changed,
 *	but the actual pointer to the port itself cannot be.
 *
 *	GPIO_TypeDef* const port = GPIOA // this is the initialization
 *	port = GPIOD			 // this is INVALID because the port shouldn't be changed once set to that pin
 */
typedef struct {
	const gpio_pin_names_e pin_name;
	GPIO_TypeDef* const port;
	const uint8_t pin_number;
	uint8_t mode;
}gpio_pin_t;

void gpio_default_initialize(void);
bool gpio_config_compare(const gpio_pin_names_e pin_to_check,
			 const gpio_pin_names_e expected_pin_name,
			 const GPIO_TypeDef* const expected_port,
			 const uint8_t expected_pin_number,
			 const uint8_t expected_mode);
void gpio_mode_set(gpio_pin_names_e pin, gpio_mode_e mode);
void gpio_alternate_function_set(gpio_pin_names_e pin_name, gpio_alternate_function_e af);
void gpio_data_output_toggle(gpio_pin_names_e pin_name);
void gpio_data_output_set(gpio_pin_names_e pin_name);
void gpio_data_output_clear(gpio_pin_names_e pin_name);
#endif /* GPIO_H_ */
