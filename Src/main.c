#include <stm32f4xx.h>
#include "led.h"
#include "uart.h"
#include "ir_receiver.h"
#include "systick.h"
#include "edge_detect.h"
#include "i2c.h"
#include "trace.h"
#include "assert_handler.h"
#include "gpio.h"

int main(void)
{
	systick_initialize();
	led_initialize();
	uart2_initialize();
	ir_receiver_initialize();
	edge_detect_initialize();
	i2c_initialize();

	#if 0
	gpio_configure_pin(VL53L0X_XSHUT, GPIO_MODE_OUTPUT, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL);
	ASSERT(gpio_config_compare(VL53L0X_XSHUT, GPIOA, 5, GPIO_MODE_OUTPUT, GPIO_RESISTOR_DISABLED, GPIO_OTYPE_PUSHPULL));
	gpio_data_output_set(VL53L0X_XSHUT);
	systick_delay_ms(200);
	#endif

	uint8_t read_i2c = 0;

	while (1) {
		i2c_read(0x29, 0xC0, &read_i2c, 1);
		if (read_i2c == 0xEE) {
			TRACE("Read expected VL53L0x id (0xEE)\n");
		} else {
			TRACE("Read UNexpected VL53L0x id 0x%X, expected (0xEE)\n", read_i2c);
		}
		led_toggle(LED_GREEN);
		systick_delay_ms(500);
	}
}
