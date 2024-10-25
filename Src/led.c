#include <stm32f4xx.h>
#include <stdbool.h>
#include "led.h"
#include "gpio.h"
#include "assert_handler.h"

static bool initialized = false;

/* Driver layer that interacts with the custom GPIO HAL underneath it.
 * All the LED initialization should happen in this function.
 */
void led_initialize(void)
{
    /* Assert to make sure the initialize function has been called only once */
    ASSERT(!initialized);

    /* Need to enable the clock connected to GPIO Port D */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    
    gpio_mode_set(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_mode_set(LED_RED, GPIO_MODE_OUTPUT);
    gpio_mode_set(LED_BLUE, GPIO_MODE_OUTPUT);

    /* Sanity check to make sure the pin configurations are set the way they should be */
    ASSERT(gpio_config_compare(LED_GREEN, LED_GREEN, GPIOD, 12, GPIO_MODE_OUTPUT));
    ASSERT(gpio_config_compare(LED_RED, LED_RED, GPIOD, 14, GPIO_MODE_OUTPUT));
    ASSERT(gpio_config_compare(LED_BLUE, LED_BLUE, GPIOD, 15, GPIO_MODE_OUTPUT));

    /* Once LEDs are initialized, update the boolean value to true */
    initialized = true;
}

/* Function to toggle the desired LED color using the color as the input parameter.
 * The gpio_pin_names_e enum holds all the names of the pins being used for this project in "gpio.h"
 */
void led_toggle(gpio_pin_names_e led_color)
{
    /* Assert to make sure the LEDs have been initialized before trying to use the LEDs */
    ASSERT(initialized);
    gpio_data_output_toggle(led_color);
}
