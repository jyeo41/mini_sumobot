#include "i2c.h"
#include "gpio.h"
#include "assert_handler.h"
#include <stm32f4xx.h>
#include <stdbool.h>

static bool initialized = false;

void i2c_initialize(void)
{
    /* 1. Enable I2C and GPIO clock.
     * 2. Configure I2C pins for alternate function.
     *      - Set AF in MODER
     *      - Set Open Drain (requirement for I2C pins) in OTYPER
     *      - Set HIGH speed
     *      - Set internal pull ups for both pins
     *      - Set AFx in AFR register, check which one it is in datasheet.
     * 3. Reset the I2C
     * 4. Program peripheral input clock in I2C_CR2 to generate timings
     * 5. Configure clock control registers
     * 6. Configure rise time register
     * 7. Enable the peripheral in I2C_CR1.
     */
    ASSERT(!initialized);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    gpio_configure_pin(I2C2_SCL, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_PULLUP, GPIO_OTYPE_OPENDRAIN);
    gpio_configure_pin(I2C2_SDA, GPIO_MODE_ALTERNATE, GPIO_AF4, GPIO_RESISTOR_PULLUP, GPIO_OTYPE_OPENDRAIN);
    ASSERT(gpio_config_compare(I2C2_SCL, GPIOB, 10, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_PULLUP, GPIO_OTYPE_OPENDRAIN));
    ASSERT(gpio_config_compare(I2C2_SDA, GPIOB, 11, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_PULLUP, GPIO_OTYPE_OPENDRAIN));

    initialized = true;
}
