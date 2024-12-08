#include <stm32f4xx.h>
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "systick.h"
#include "edge_detect.h"
#include "i2c.h"

int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	edge_detect_initialize();
	i2c_initialize();

	while (1) {
		led_toggle(LED_GREEN);
		systick_delay_ms(500);
	}
}
