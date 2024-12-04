#ifndef ADC_H_
#define ADC_H_
#include <stdint.h>

#define ADC_CHANNELS_USED       4

typedef enum {
	ADC_CHANNEL10_PC0,
	ADC_CHANNEL11_PC1,
	ADC_CHANNEL12_PC2,
	ADC_CHANNEL13_PC3,
}adc_dma_values_index_e;

void adc_initialize(void);
void adc_dma_get_values(uint16_t adc_dma_values[]);
void adc_dma_print_values(const uint16_t adc_dma_values[]);

#endif /* ADC_H_ */
