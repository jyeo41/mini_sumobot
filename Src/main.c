#include <stm32f4xx.h>
#include "systick.h"
#include "assert_handler.h"
#include "trace.h"
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "edge_detect.h"
#include "vl53l0x.h"
#include "pwm.h"


int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
    pwm_initialize();
	edge_detect_initialize();
    vl53l0x_initialize();

	while (1) {
        //vl53l0x_test_range(VL53L0X_INDEX_MIDDLE);
        //vl53l0x_test_range(VL53L0X_INDEX_LEFT);
        //vl53l0x_test_range(VL53L0X_INDEX_RIGHT);
        pwm_test();
		led_toggle(LED_GREEN);
        systick_delay_ms(500);
	}
}
