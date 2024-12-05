#include <stm32f4xx.h>
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "systick.h"
#include "edge_detect.h"

int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	edge_detect_initialize();

	while (1) {
		edge_detect_location_print(edge_detect_lookup());
		led_toggle(LED_GREEN);
		systick_delay_ms(500);
	}
}
