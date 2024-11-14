#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
#include "systick.h"
#include "uart.h"
#include "assert_handler.h"
#include "trace.h"

int main(void)
{
	gpio_default_initialize();
	led_initialize();
	systick_initialize();
	uart2_initialize();

	while (1) {
		/* Testing MPaland's printf() implementation */
		TRACE("This is main.c");
		ASSERT(0);
		systick_delay_ms(250);
		led_toggle(LED_GREEN);
	}
}
