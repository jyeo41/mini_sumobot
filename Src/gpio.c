#include <stm32f4xx.h>
#include "gpio.h"

static const gpio_pin_t gpio_board_pins[] = {
    [LED_GREEN] = {GPIOD, 12, GPIO_MODE_OUTPUT}
};

static const uint32_t gpio_mode_register_bits[][2] = {
    [12] = {GPIO_MODER_MODER12_0, GPIO_MODER_MODER12_1}
};


void gpio_mode_set(gpio_pin_names_e pin_name, gpio_mode_e mode)
{
    const gpio_pin_t pin = gpio_board_pins[pin_name];
    
    switch(mode) {
        case GPIO_MODE_INPUT:
            pin.port->MODER &= ~(pin.pin_number) & ~(pin.pin_number << 1);
            break;
        case GPIO_MODE_OUTPUT:
            pin.port->MODER |= gpio_mode_register_bits[pin.pin_number][0];
            pin.port->MODER &= ~(gpio_mode_register_bits[pin.pin_number][1]);
            break;
        case GPIO_MODE_ALTERNATE:
            pin.port->MODER |= (pin.pin_number << 1);
            break;
        case GPIO_MODE_ANALOG:
            pin.port->MODER |= pin.pin_number | (pin.pin_number << 1);
            break;
    }
}
