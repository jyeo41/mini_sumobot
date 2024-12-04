#include <stm32f4xx.h>
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

	uint16_t adc_values[ADC_CHANNELS_USED];

	while (1) {
		adc_dma_get_values(adc_values);
		adc_dma_print_values(adc_values);
		led_toggle(LED_GREEN);
		systick_delay_ms(1000);
	}
}
