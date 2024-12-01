#ifndef ADC_H_
#define ADC_H_
#include <stdint.h>

void adc_initialize(void);
void adc_enable(void);
void adc_conversion_start(uint8_t channel);
uint16_t adc_value_get(void);
void adc_conversion_wait(void);
void adc_dma_initialize(void);

#endif /* ADC_H_ */
