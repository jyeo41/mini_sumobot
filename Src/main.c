#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
#include "systick.h"

int main(void)
{
	//volatile unsigned int i;
	led_initialize();
	systick_initialize();
	gpio_mode_set(LED_GREEN, GPIO_MODE_OUTPUT);
	gpio_mode_set(LED_ORANGE, GPIO_MODE_OUTPUT);
	gpio_mode_set(LED_RED, GPIO_MODE_OUTPUT);
	gpio_mode_set(LED_BLUE, GPIO_MODE_OUTPUT);

	while (1) {
		systick_delay_ms(500);
		led_green_toggle();
		systick_delay_ms(500);
		led_orange_toggle();
		systick_delay_ms(500);
		led_red_toggle();
		systick_delay_ms(500);
		led_blue_toggle();
	}
}
