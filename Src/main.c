#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
#include "systick.h"
#include "uart.h"

int main(void)
{
	gpio_default_initialize();
	led_initialize();
	systick_initialize();
	uart2_initialize();

	while (1) {
		uart_send_string("HELLO WORLD THIS IS STRESS TESTING THE RING BUFFER\n");
		systick_delay_ms(250);
		led_toggle(LED_GREEN);
	}
}
