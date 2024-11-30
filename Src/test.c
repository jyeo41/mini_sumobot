#include "test.h"
#include <stm32f4xx.h>
#include "systick.h"
#include "trace.h"
#include "adc.h"
#include "ir_receiver.h"

static uint16_t test_adc_values[4] = {0, 0, 0, 0};


void test_adc_polling(void)
{
        adc_conversion_start(10);
        adc_conversion_wait();
        test_adc_values[0] = adc_value_get();

        adc_conversion_start(11);
        adc_conversion_wait();
        test_adc_values[1] = adc_value_get();

        adc_conversion_start(12);
        adc_conversion_wait();
        test_adc_values[2] = adc_value_get();

        adc_conversion_start(13);
        adc_conversion_wait();
        test_adc_values[3] = adc_value_get();

        for(uint8_t i = 0; i < 4; i++) {
                TRACE("ADC Channel %u: %u", (i + 10), test_adc_values[i]);
        }

        systick_delay_ms(1000);
}

// cppcheck-suppress unusedFunction
void test_ir_receiver(void)
{
        ir_receiver_get_cmd();
}
