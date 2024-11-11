#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
#include "systick.h"
#include "uart.h"
/* Have to include full path to the file otherwise cppcheck throws an error */
#include "../external/printf/printf.h"

int main(void)
{
	gpio_default_initialize();
	led_initialize();
	systick_initialize();
	uart2_initialize();

	while (1) {
		/* Testing MPaland's printf() implementation */
		printf("Testing two numbers %d and a longer string %d\n", 2020, 2024);
		systick_delay_ms(250);
		led_toggle(LED_GREEN);
	}
}
