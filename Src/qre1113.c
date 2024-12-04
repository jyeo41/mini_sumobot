#include "qre1113.h"
#include "adc.h"
#include "assert_handler.h"
#include "trace.h"
#include <stdint.h>
#include <stdbool.h>

static bool initialized = false;

/* Wrapper initialize function. The application shouldn't know how to call the low level ADC driver initialization. */
void qre1113_initialize(void)
{
    ASSERT(!initialized);
    adc_initialize();
    initialized = true;
}

/* Abstracting away the voltages from the raw ADC channels, to sensor placements and locations on the sumobot. 
 * NOTE: sensor locations of front_left, front_right, etc are not final as of right now. */
void qre1113_get_voltages(qre1113_voltages_t* voltages)
{
    uint16_t adc_values[ADC_CHANNELS_USED];
    adc_dma_get_values(adc_values);

    voltages->front_left = adc_values[ADC_CHANNEL10_PC0];
    voltages->front_right = adc_values[ADC_CHANNEL11_PC1];
    voltages->back_left = adc_values[ADC_CHANNEL12_PC2];
    voltages->back_right = adc_values[ADC_CHANNEL13_PC3];
}

/* Function to test voltages are being properly assigned in the qre1113_voltages struct */
void qre1113_print_voltages(const qre1113_voltages_t* voltages)
{
    TRACE("\nFront left: %u\n"
          "Front right: %u\n"
          "Back left: %u\n"
          "Back right: %u\n\n", voltages->front_left, voltages->front_right, voltages->back_left, voltages->back_right);
}
