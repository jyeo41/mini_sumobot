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
 * The application can change some of the attributes at runtime however, the pin name, port, and pin number should be immutable.
 */
static gpio_pin_t gpio_board_pins[] = {
    [LED_GREEN] =           {GPIOD, 12, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [LED_ORANGE] =          {GPIOD, 13, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [LED_RED] =             {GPIOD, 14, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [LED_BLUE] =            {GPIOD, 15, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [UART2_BOARD_TX] =      {GPIOA, 2, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [UART2_BOARD_RX] =      {GPIOA, 3, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [IR_RECEIVER] =         {GPIOA, 15, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [ADC123_CHANNEL10] =    {GPIOC, 0, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [ADC123_CHANNEL11] =    {GPIOC, 1, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [ADC123_CHANNEL12] =    {GPIOC, 2, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [ADC123_CHANNEL13] =    {GPIOC, 3, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
    [I2C2_SCL] =            {GPIOB, 10, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_PULLUP, GPIO_OTYPE_OPENDRAIN},
    [I2C2_SDA] =            {GPIOB, 11, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_PULLUP, GPIO_OTYPE_OPENDRAIN},
    [VL53L0X_XSHUT] =       {GPIOA, 5, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL},
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
    [3] = {GPIO_MODER_MODER3_0, GPIO_MODER_MODER3_1},
    [4] = {GPIO_MODER_MODER4_0, GPIO_MODER_MODER4_1},
    [5] = {GPIO_MODER_MODER5_0, GPIO_MODER_MODER5_1},
    [6] = {GPIO_MODER_MODER6_0, GPIO_MODER_MODER6_1},
    [7] = {GPIO_MODER_MODER7_0, GPIO_MODER_MODER7_1},
    [8] = {GPIO_MODER_MODER8_0, GPIO_MODER_MODER8_1},
    [9] = {GPIO_MODER_MODER9_0, GPIO_MODER_MODER9_1},
    [10] = {GPIO_MODER_MODER10_0, GPIO_MODER_MODER10_1},
    [11] = {GPIO_MODER_MODER11_0, GPIO_MODER_MODER11_1},
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
    [0] = {GPIO_BSRR_BS0, GPIO_BSRR_BR0},
    [1] = {GPIO_BSRR_BS1, GPIO_BSRR_BR1},
    [2] = {GPIO_BSRR_BS2, GPIO_BSRR_BR2},
    [3] = {GPIO_BSRR_BS3, GPIO_BSRR_BR3},
    [4] = {GPIO_BSRR_BS4, GPIO_BSRR_BR4},
    [5] = {GPIO_BSRR_BS5, GPIO_BSRR_BR5},
    [6] = {GPIO_BSRR_BS6, GPIO_BSRR_BR6},
    [7] = {GPIO_BSRR_BS7, GPIO_BSRR_BR7},
    [8] = {GPIO_BSRR_BS8, GPIO_BSRR_BR8},
    [9] = {GPIO_BSRR_BS9, GPIO_BSRR_BR9},
    [10] = {GPIO_BSRR_BS10, GPIO_BSRR_BR10},
    [11] = {GPIO_BSRR_BS11, GPIO_BSRR_BR11},
    [12] = {GPIO_BSRR_BS12, GPIO_BSRR_BR12},
    [13] = {GPIO_BSRR_BS13, GPIO_BSRR_BR13},
    [14] = {GPIO_BSRR_BS14, GPIO_BSRR_BR14},
    [15] = {GPIO_BSRR_BS15, GPIO_BSRR_BR15},
};

/* Using _Pos defines because we use these defines to shift a 0, 1, or 2 to their appropriate position */
static const uint32_t gpio_pupdr_register_masks[] = {
    [0] = GPIO_PUPDR_PUPD0_Pos,
    [1] = GPIO_PUPDR_PUPD1_Pos,
    [2] = GPIO_PUPDR_PUPD2_Pos,
    [3] = GPIO_PUPDR_PUPD3_Pos,
    [4] = GPIO_PUPDR_PUPD4_Pos,
    [5] = GPIO_PUPDR_PUPD5_Pos,
    [6] = GPIO_PUPDR_PUPD6_Pos,
    [7] = GPIO_PUPDR_PUPD7_Pos,
    [8] = GPIO_PUPDR_PUPD8_Pos,
    [9] = GPIO_PUPDR_PUPD9_Pos,
    [10] = GPIO_PUPDR_PUPD10_Pos,
    [11] = GPIO_PUPDR_PUPD11_Pos,
    [12] = GPIO_PUPDR_PUPD12_Pos,
    [13] = GPIO_PUPDR_PUPD13_Pos,
    [14] = GPIO_PUPDR_PUPD14_Pos,
    [15] = GPIO_PUPDR_PUPD15_Pos,
};

static const uint16_t gpio_odr_register_bit[] = {
    [0] = GPIO_ODR_OD0,
    [1] = GPIO_ODR_OD1,
    [2] = GPIO_ODR_OD2,
    [3] = GPIO_ODR_OD3,
    [4] = GPIO_ODR_OD4,
    [5] = GPIO_ODR_OD5,
    [6] = GPIO_ODR_OD6,
    [7] = GPIO_ODR_OD7,
    [8] = GPIO_ODR_OD8,
    [9] = GPIO_ODR_OD9,
    [10] = GPIO_ODR_OD10,
    [11] = GPIO_ODR_OD11,
    [12] = GPIO_ODR_OD12,
    [13] = GPIO_ODR_OD13,
    [14] = GPIO_ODR_OD14,
    [15] = GPIO_ODR_OD15,
};

/* Function to call other gpio setting helper functions. */
void gpio_configure_pin(gpio_pin_names_e pin_name, gpio_mode_e mode,
			gpio_alternate_function_e af, gpio_resistor_e resistor, gpio_otype_e otype)
{
    gpio_mode_set(pin_name, mode);
    if (mode == GPIO_MODE_ALTERNATE) {
        gpio_alternate_function_set(pin_name, af);
    }
    gpio_resistor_set(pin_name, resistor);
    gpio_otype_set(pin_name, otype);
}

/* Make all the incoming parameters constants because they should be read only.
 * This is to explicitly state these values are to be only used for comparison purposes. */
bool gpio_config_compare(const gpio_pin_names_e pin_to_check,
			 const GPIO_TypeDef* const expected_port,
			 const uint8_t expected_pin_number,
			 const gpio_mode_e expected_mode,
			 const gpio_resistor_e expected_resistor,
                         const gpio_otype_e expected_otype)
{
    const gpio_pin_t test_pin = gpio_board_pins[pin_to_check];

    return ((test_pin.port == expected_port) &&
            (test_pin.pin_number == expected_pin_number) &&
            (test_pin.mode == expected_mode) &&
            (test_pin.resistor == expected_resistor) &&
            (test_pin.otype == expected_otype));
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

void gpio_resistor_set(gpio_pin_names_e pin_name, gpio_resistor_e resistor)
{
    gpio_pin_t pin = gpio_board_pins[pin_name];

    switch (resistor) {
        case GPIO_RESISTOR_DISABLED:
            pin.port->PUPDR |= (0 << gpio_pupdr_register_masks[pin.pin_number]);
            break;
        case GPIO_RESISTOR_PULLUP:
            pin.port->PUPDR |= (1 << gpio_pupdr_register_masks[pin.pin_number]);
            break;
        case GPIO_RESISTOR_PULLDOWN:
            pin.port->PUPDR |= (2 << gpio_pupdr_register_masks[pin.pin_number]);
            break;
    }
    gpio_board_pins[pin_name].resistor = resistor;
}

/* Setting the output type in OTYPER register. */
void gpio_otype_set(gpio_pin_names_e pin_name, gpio_otype_e otype)
{
    gpio_pin_t pin = gpio_board_pins[pin_name];

    switch (otype) {
        case GPIO_OTYPE_PUSHPULL:
            pin.port->OTYPER |= (0 << pin.pin_number);
            break;
        case GPIO_OTYPE_OPENDRAIN:
            pin.port->OTYPER |= (1 << pin.pin_number);
            break;
    }
    gpio_board_pins[pin_name].otype = otype;
}

#if 0
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
#endif

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
