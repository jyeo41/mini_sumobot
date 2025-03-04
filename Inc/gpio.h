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
	UART2_BOARD_RX,
	IR_RECEIVER,
	ADC123_CHANNEL10,
	ADC123_CHANNEL11,
	ADC123_CHANNEL12,
	ADC123_CHANNEL13,
	I2C2_SCL,
	I2C2_SDA,
	VL53L0X_GPIO1_MIDDLE,
	VL53L0X_XSHUT_MIDDLE,
	VL53L0X_XSHUT_LEFT,
	VL53L0X_XSHUT_RIGHT,
    PWM_LEFT_TIM3_CHANNEL1,
    PWM_RIGHT_TIM3_CHANNEL2,
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
	GPIO_AF_NONE,
}gpio_alternate_function_e;

/* gpio_pin_t setting to enable or disable internal pullup/pulldown resistor.
 * Used with gpio input pins.
 */
typedef enum {
	GPIO_RESISTOR_DISABLED,
	GPIO_RESISTOR_PULLUP,
	GPIO_RESISTOR_PULLDOWN,
}gpio_resistor_e;

typedef enum {
	GPIO_OTYPE_PUSHPULL,
	GPIO_OTYPE_OPENDRAIN,
}gpio_otype_e;

typedef enum {
    GPIO_SPEED_LOW,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH,
}gpio_speed_e;

typedef enum {
	GPIO_INTERRUPT_TRIGGER_FALLING,
	GPIO_INTERRUPT_TRIGGER_RISING,
}gpio_interrupt_edge_trigger_e;

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
 *	port = GPIOD			 // this is INVALID because the port shouldn't be changed once set to that pin.
 *
 */
typedef struct {
	GPIO_TypeDef* const port;
	const uint8_t pin_number;
	gpio_mode_e mode;
	gpio_resistor_e resistor;
	gpio_otype_e otype;
    gpio_speed_e speed;
}gpio_pin_t;

void gpio_default_initialize(void);
void gpio_configure_pin(gpio_pin_names_e pin_name, gpio_mode_e mode,
			gpio_alternate_function_e af, gpio_resistor_e resistor, gpio_otype_e otype, gpio_speed_e speed);
bool gpio_config_compare(const gpio_pin_names_e pin_to_check,
			 const GPIO_TypeDef* const expected_port,
			 const uint8_t expected_pin_number,
			 const gpio_mode_e expected_mode,
			 const gpio_resistor_e expected_resistor,
			 const gpio_otype_e expected_otype,
             const gpio_speed_e expected_speed);

void gpio_mode_set(gpio_pin_names_e pin_name, gpio_mode_e mode);
void gpio_alternate_function_set(gpio_pin_names_e pin_name, gpio_alternate_function_e af);
void gpio_resistor_set(gpio_pin_names_e pin_name, gpio_resistor_e resistor);
void gpio_speed_set(gpio_pin_names_e pin_name, gpio_speed_e speed);
void gpio_otype_set(gpio_pin_names_e pin_name, gpio_otype_e otype);
void gpio_interrupt_set(gpio_pin_names_e pin_name,
			gpio_interrupt_edge_trigger_e edge_trigger, 
			IRQn_Type IRQn,
			uint32_t priority);

void gpio_data_output_toggle(gpio_pin_names_e pin_name);
void gpio_data_output_set(gpio_pin_names_e pin_name);
void gpio_data_output_clear(gpio_pin_names_e pin_name);
#endif /* GPIO_H_ */
