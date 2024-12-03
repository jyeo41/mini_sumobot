#ifndef ADC_H_
#define ADC_H_
#include <stdint.h>

void adc_initialize(void);
void adc_dma_print_values(void);
uint16_t adc_value_get(void);

#endif /* ADC_H_ */
