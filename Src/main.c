#include <stm32f4xx.h>
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "systick.h"
#include "qre1113.h"
#include "adc.h"

int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	qre1113_initialize();

	qre1113_voltages_t voltages;

	while (1) {
		qre1113_get_voltages(&voltages);
		qre1113_print_voltages(&voltages);
		led_toggle(LED_GREEN);
		systick_delay_ms(1000);
	}
}
