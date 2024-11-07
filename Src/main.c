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
		uart2_send_char('H');
		uart2_send_char('E');
		uart2_send_char('L');
		uart2_send_char('L');
		uart2_send_char('O');
		uart2_send_char('\n');
		systick_delay_ms(1000);
		led_toggle(LED_GREEN);
	}
}
