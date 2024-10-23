#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
/* User Manual UM1472 states:

User Green LED:    I/O PD12
User Orange LED: I/O PD13
User Red LED:    I/O PD14
User Blue LED:    I/O PD15*/

void led_initialize(void)
{
    /* Need to enable the clock connected to GPIO Port D */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}

/* PD 12 */
void led_green_toggle(void)
{
    gpio_data_output_toggle(LED_GREEN);
}

/* Orange LED PD 13 */
void led_orange_toggle(void)
{
    gpio_data_output_toggle(LED_ORANGE);
}

void led_red_toggle(void)
{
    gpio_data_output_toggle(LED_RED);
}

void led_blue_toggle(void)
{
    gpio_data_output_toggle(LED_BLUE);
}
