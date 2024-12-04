#ifndef ADC_H_
#define ADC_H_
#include <stdint.h>

#define ADC_CHANNELS_USED       4

void adc_initialize(void);
void adc_dma_get_values(uint16_t adc_dma_values[]);
void adc_dma_print_values(const uint16_t adc_dma_values[]);

#endif /* ADC_H_ */
