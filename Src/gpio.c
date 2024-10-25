#include <stm32f4xx.h>
#include <stdbool.h>
#include "gpio.h"

#define GPIO_BOARD_PINS_LENGTH sizeof(gpio_board_pins)/sizeof(gpio_board_pins[0])

/* Array to map high level pin names (an enum) such as LED_GREEN to the pin's attributes (a struct).
 * This centralizes all the info one would need about a pin, such as the port, pin number, etc.
 * This array is the core implementation of abstracting away the low level register interactions from the user.
 *
 * These are the default pin configurations the board starts with, and using gpio functions such as
 *  - gpio_mode_set()
 *
 * The application can change some of the attributes at runtime. The pin name, port, and pin number should be immutable.
 */
static gpio_pin_t gpio_board_pins[] = {
    [LED_GREEN] =   {LED_GREEN, GPIOD, 12, GPIO_MODE_OUTPUT},
    //[LED_ORANGE] =  {LED_ORANGE, GPIOD, 13, GPIO_MODE_OUTPUT},
    [LED_RED] =     {LED_RED, GPIOD, 14, GPIO_MODE_OUTPUT},
    [LED_BLUE] =    {LED_BLUE, GPIOD, 15, GPIO_MODE_OUTPUT},
};

/* Mapping pin numbers to their corresponding MODE register as a 2D array.
 * With STM32F4xx series, some of the GPIO registers use a multi-width bit-field.
 * The MODER register uses a 2-bit field for each corresponding pin.
 * Having arrays to represent these bitfields makes it easy to access them generically.
 * 
 * Example:
 *      gpio_mode_register_bits[pin.pin_number][0]
 *
 *      The above represents accesing bit field 0 for that specific pin number in the mode register
 */
static const uint32_t gpio_mode_register_bits[][2] = {
    [12] = {GPIO_MODER_MODER12_0, GPIO_MODER_MODER12_1},
    [13] = {GPIO_MODER_MODER13_0, GPIO_MODER_MODER13_1},
    [14] = {GPIO_MODER_MODER14_0, GPIO_MODER_MODER14_1},
    [15] = {GPIO_MODER_MODER15_0, GPIO_MODER_MODER15_1},
};

/* The BSRR register is unique in that it uses 2 bits to set and reset the data register ODR atomically.
 * However, the bits are not a 2-bit field like the MODER register.
 * Bits 0-16 are to SET the corresponding pin in the ODR register, and bits 17-31 are to RESET the corresponding pin.
 * So for pin n, the respective bits would be bit n & bit n + 16
 * The CMSIS headers already have defines to account for these offsets, so mapping them into arrays makes it easier to work with.
 */
static const uint32_t gpio_bsrr_register_bits[][2] = {
    [12] = {GPIO_BSRR_BS12, GPIO_BSRR_BR12},
    [13] = {GPIO_BSRR_BS13, GPIO_BSRR_BR13},
    [14] = {GPIO_BSRR_BS14, GPIO_BSRR_BR14},
    [15] = {GPIO_BSRR_BS15, GPIO_BSRR_BR15},
};

static const uint32_t gpio_odr_register_bit[] = {
    [12] = GPIO_ODR_OD12,
    [13] = GPIO_ODR_OD13,
    [14] = GPIO_ODR_OD14,
    [15] = GPIO_ODR_OD15,
};

void gpio_default_initialize(void)
{
    volatile uint8_t i;
    for (i = 0; i < GPIO_BOARD_PINS_LENGTH; i++) {
        gpio_mode_set(gpio_board_pins[i].pin_name, gpio_board_pins[i].mode);
    }
}

/* Make all the incoming parameters constants because they should be read only.
 * This is to explicitly state these values are to be only used for comparison purposes. */
bool gpio_config_compare(const gpio_pin_names_e pin_to_check,
			 const gpio_pin_names_e expected_pin_name,
			 const GPIO_TypeDef* const expected_port,
			 const uint8_t expected_pin_number,
			 const uint8_t expected_mode)
{
    const gpio_pin_t test_pin = gpio_board_pins[pin_to_check];

    return (test_pin.pin_name == expected_pin_name) &&
            (test_pin.port == expected_port) &&
            (test_pin.pin_number == expected_pin_number) &&
            (test_pin.mode == expected_mode);
}

void gpio_mode_set(gpio_pin_names_e pin_name, gpio_mode_e desired_mode)
{
    gpio_pin_t pin = gpio_board_pins[pin_name];
    
    switch(desired_mode) {
        /* Input mode is 0b00 */
        case GPIO_MODE_INPUT:
            pin.port->MODER &= ~(gpio_mode_register_bits[pin.pin_number][0]);
            pin.port->MODER &= ~(gpio_mode_register_bits[pin.pin_number][1]);
            break;
        /* GPIO Output mode is 0b01 */
        case GPIO_MODE_OUTPUT:
            pin.port->MODER |= gpio_mode_register_bits[pin.pin_number][0];
            pin.port->MODER &= ~(gpio_mode_register_bits[pin.pin_number][1]);
            break;
        /* Alternate function mode is 0b10 */
        case GPIO_MODE_ALTERNATE:
            pin.port->MODER &= ~(gpio_mode_register_bits[pin.pin_number][0]);
            pin.port->MODER |= gpio_mode_register_bits[pin.pin_number][1];
            break;
        /* Analog mode is 0b11 */
        case GPIO_MODE_ANALOG:
            pin.port->MODER |= gpio_mode_register_bits[pin.pin_number][0];
            pin.port->MODER |= gpio_mode_register_bits[pin.pin_number][1];
            break;
    }
    gpio_board_pins[pin_name].mode = desired_mode;
}

/* Atomic write to the data register for GPIO Output pins.
 * Check if the bit is set in the ODR register, and if it is,
 * clear it by writing to the higher order bit in BSRR_BRx.
 * 
 * Else, set it by writing to the lower order bit in BSRR_BSx.
 * BR means bit reset and BS means bit set. Writing a 1 to either of these bits
 * clears or sets the corresponding bits in ODR register atomically.
 */
void gpio_data_output_toggle(gpio_pin_names_e pin_name)
{
    const gpio_pin_t pin = gpio_board_pins[pin_name];
    if (pin.port->ODR & gpio_odr_register_bit[pin.pin_number]) {
        gpio_data_output_clear(pin_name);
    } else {
        gpio_data_output_set(pin_name);
    }
}

void gpio_data_output_set(gpio_pin_names_e pin_name)
{
    const gpio_pin_t pin = gpio_board_pins[pin_name];

    pin.port->BSRR |= (gpio_bsrr_register_bits[pin.pin_number][0]);
}

void gpio_data_output_clear(gpio_pin_names_e pin_name)
{
    const gpio_pin_t pin = gpio_board_pins[pin_name];
    
    pin.port->BSRR |= (gpio_bsrr_register_bits[pin.pin_number][1]);
}
