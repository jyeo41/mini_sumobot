#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
#include "systick.h"

int main(void)
{
	gpio_default_initialize();
	led_initialize();
	systick_initialize();

	while (1) {
		systick_delay_ms(1000);
		led_toggle(LED_GREEN);
	}
}
