#include <stm32f4xx.h>
#include "led.h"

int main(void)
{
	volatile unsigned int i;
	led_initialize();


	while (1) {
		for (i = 0; i < 1000000; i++) {}
		led_green_toggle();
	}

	
}
