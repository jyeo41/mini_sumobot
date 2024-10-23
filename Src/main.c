#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"

int main(void)
{
	volatile unsigned int i;
	led_initialize();
	gpio_mode_set(LED_GREEN, GPIO_MODE_OUTPUT);

	while (1) {
		for (i = 0; i < 1000000; i++) {}
		led_green_toggle();
	}
}
