#include <stm32f4xx.h>
#include "test.h"
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "adc.h"
#include "systick.h"

int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	adc_initialize();

	while (1) {
		test_adc_polling();
		led_toggle(LED_GREEN);
	}
}
