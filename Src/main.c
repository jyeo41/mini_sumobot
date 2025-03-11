#include <stm32f4xx.h>
#include "systick.h"
#include "assert_handler.h"
#include "trace.h"
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "edge_detect.h"
#include "vl53l0x.h"
#include "tb6612fng.h"


int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	edge_detect_initialize();
    vl53l0x_initialize();
    tb6612fng_initialize();

	while (1) {
        TRACE("IR COMMAND: %u\n", ir_receiver_get_cmd());
		led_toggle(LED_GREEN);
        systick_delay_ms(500);
	}
}
