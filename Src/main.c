#include <stm32f4xx.h>
#include "led.h"
#include "gpio.h"
#include "systick.h"
#include "uart.h"
#include "assert_handler.h"
#include "trace.h"
#include "ir_receiver.h"
#include "adc.h"

int main(void)
{
	led_initialize();
	systick_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	adc_initialize();

	while (1) {
		TRACE("Command: %s", ir_receiver_get_cmd());
		led_toggle(LED_GREEN);
		systick_delay_ms(150);
	}
}
