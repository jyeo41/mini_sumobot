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
    [LED_GREEN] =       {LED_GREEN, GPIOD, 12, GPIO_MODE_OUTPUT},
    [LED_ORANGE] =      {LED_ORANGE, GPIOD, 13, GPIO_MODE_OUTPUT},
    [LED_RED] =         {LED_RED, GPIOD, 14, GPIO_MODE_OUTPUT},
    [LED_BLUE] =        {LED_BLUE, GPIOD, 15, GPIO_MODE_OUTPUT},
    [UART2_BOARD_TX] =  {UART2_BOARD_TX, GPIOA, 2, GPIO_MODE_ALTERNATE},
    [IR_RECEIVER] =     {IR_RECEIVER, GPIOB, 0, GPIO_MODE_INPUT},
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
    [0] = {GPIO_MODER_MODER0_0, GPIO_MODER_MODER0_1},
    [1] = {GPIO_MODER_MODER1_0, GPIO_MODER_MODER1_1},
    [2] = {GPIO_MODER_MODER2_0, GPIO_MODER_MODER2_1},
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

void gpio_alternate_function_set(gpio_pin_names_e pin_name, gpio_alternate_function_e af)
{
    gpio_pin_t pin = gpio_board_pins[pin_name];

    /* Pins 0:7 use GPIOx_AFRL register.
     * Pins 8:15 use GPIOx_AFRH register.
     * AFR[0] represents AFRL.
     * AFR[1] represents AFRH.
     * 
     * The left shift of (4 * pin number) is because each pin is represented by a
     *  4 bit-field that can range from AF0 to AF15, so 16 total values, 2^4.
     *  This means pin 0 is from 3:0, pin 1 is 4:7, etc so the shift has to be
     *  4 * pin number.
     * For AFRH register, we have to mod by 32 otherwise 4 * pin 8 would shift 32 to the left,
     *  which is out of bounds and pin 8 is the "0th" pin of that register.
     */

    /* pin number is an unsigned int, so unnecessary to test pin.pin_number >= 0 */
    if (pin.pin_number <= 7) {
        pin.port->AFR[0] |= (af << (4 * pin.pin_number));
    } else {
        pin.port->AFR[1] |= (af << ((4 * pin.pin_number) % 32 ));
    }
}

void gpio_interrupt_set(gpio_pin_names_e pin_name,
			gpio_interrupt_edge_trigger_e edge_trigger, 
			IRQn_Type IRQn,
			uint32_t priority)
{
    /* GPIO Pin Interrupt Configuration
     * 1. Pin must be in input mode.
     * 2. Configure the edge trigger (EXTI->FTSR/RTSR/RFSR).
     * 3. Configure GPIO Port Selection: Memory and Bus Architecture Table 1 (SYSCFG_EXTICR)
     * 3. Enable interrupt delivery from peripheral side (EXTI->IMR).
     * 4. Identify the IRQ Number for said interrupt
     * 5. Configure the IRQ Priority for the IRQ Number
     * 6. Enable interrupt reception from processor side for the IRQ number
     * 7. Implement IRQ handler */

    gpio_pin_t pin = gpio_board_pins[pin_name];
    
    /* Variable to extract out the correct EXTICRn register.
     * Reference Manual, Chapter 9 System Configuration Controller, 9.2.3 */
    uint8_t exti_cr_number = 0;
    if (pin.pin_number <= 3) {
        exti_cr_number = 1;
    } else if (pin.pin_number <= 7) {
        exti_cr_number = 2;
    } else if (pin.pin_number <= 11) {
        exti_cr_number = 3;
    } else {
        exti_cr_number = 4;
    }

    /* Variable to map the desired port selection to an unsigned integer from 0 - 8 */
    uint8_t port_selection = 0;
    if (pin.port == GPIOA) {
        port_selection = 0;
    } else if (pin.port == GPIOB) {
        port_selection = 1;
    } else if (pin.port == GPIOC) {
        port_selection = 2;
    } else if (pin.port == GPIOD) {
        port_selection = 3;
    } else if (pin.port == GPIOE) {
        port_selection = 4;
    } else if (pin.port == GPIOF) {
        port_selection = 5;
    } else if (pin.port == GPIOG) {
        port_selection = 6;
    } else if (pin.port == GPIOH) {
        port_selection = 7;
    } else if (pin.port == GPIOI) {
        port_selection = 8;
    }

    /* First enable the SYSCFG clock in the RCC APG2ENR register.
     *
     * The exti_cr_number - 1 is an offset because the array starts from index 0.
     * 4 * pin.pin_number is because each EXTIn is 4 bits wide.
     * % 16 is because each SYSCFG_EXTICRn register only uses lower 16 bits.
     *
     * Example: Configuring GPIO PB7 
     *  Pin 7 is EXTICR2
     *  Need to shift 0b0001 to the left 12
     *
     *  4 * pin number 7 = 28 % 16 = 12
     */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[exti_cr_number - 1] |= (port_selection << ((4 * pin.pin_number) % 16));

    /* Set falling or rising edge trigger*/
    switch(edge_trigger) {
        case GPIO_INTERRUPT_TRIGGER_FALLING:
            EXTI->FTSR |= (1 << pin.pin_number);
            EXTI->RTSR &= ~(1 << pin.pin_number);
            break;
        case GPIO_INTERRUPT_TRIGGER_RISING:
            EXTI->RTSR |= (1 << pin.pin_number);
            EXTI->FTSR &= ~(1 << pin.pin_number);
            break;
    }

    /* Enable interrupt delivery from peripheral side */
    EXTI->IMR |= (1 << pin.pin_number);

    /* Enable interrupt receiving from processor side */
    NVIC_EnableIRQ(IRQn);
    NVIC_SetPriority(IRQn, priority);
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
