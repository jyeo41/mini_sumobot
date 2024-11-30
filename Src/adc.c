#include "adc.h"
#include "gpio.h"
#include "assert_handler.h"
#include <stm32f4xx.h>
#include <stdbool.h>

#define ADC_CHANNELS_USED  4

static bool initialized = false;

void adc_initialize(void)
{
    ASSERT(!initialized);
    /* Enable ADC (APB2) and GPIOF (AHB1) clocks in RCC register
     * Set the prescaler for ADC clock in CCR
     * Set the scan mode and resolution in CR1
     * Set continuous conversion, EOC, data alignment in CR2
     * Set sampling time for channels in SMPRx
     * Set regular channel sequence length in SQR1
     * Set GPIO pins to analog mode
     */

    /* Use ADC3 channels 10, 11, 12, 13 which are mapped to pins PFC 0, 1, 2, 3 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* Set prescaler for ADC clock to 2, by clearing both bits to 0.
     * The max for ADCCLK is 36 MHz. */
    ADC->CCR &= ~(ADC_CCR_ADCPRE);

    /* Scan mode to continuous by setting the bit. This is because we are using multiple channels.
     * Resolution to 12 bits by clearing both bits. */
    ADC3->CR1 |= ADC_CR1_SCAN;
    ADC3->CR1 &= ~(ADC_CR1_RES);

    /* Continuous convesion mode by setting CONT bit to 1. 
     * EOC flag to set at the end of each individual regular conversion by setting EOCS bit to 1. 
     * Set right alignment by clearing ALIGN bit. */
    ADC3->CR2 |= ADC_CR2_CONT;
    ADC3->CR2 |= ADC_CR2_EOCS;
    ADC3->CR2 &= ~(ADC_CR2_ALIGN);

    /* Set sampling time to be simple at 3 cycles by clearing all 3 bits. 
     * Speed doesn't matter too much here because we're gated by the system clock of 16MHz anyways. */
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP4);
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP5);
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP6);
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP7);

    /* Regular channel sequence length, set to 4 conversions because we are using 4 channels.
     * Have to integer written to bitfield should be total number of channels used - 1*/
    ADC3->SQR1 |= ((ADC_CHANNELS_USED - 1) << ADC_SQR1_L_Pos);
    
    gpio_configure_pin(ADC123_CHANNEL10, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_configure_pin(ADC123_CHANNEL11, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_configure_pin(ADC123_CHANNEL12, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_configure_pin(ADC123_CHANNEL13, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);

    /* Sanity check to make sure the pins are initialized properly */
    ASSERT(gpio_config_compare(ADC123_CHANNEL10, GPIOC, 0, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));
    ASSERT(gpio_config_compare(ADC123_CHANNEL11, GPIOC, 1, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));
    ASSERT(gpio_config_compare(ADC123_CHANNEL12, GPIOC, 2, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));
    ASSERT(gpio_config_compare(ADC123_CHANNEL13, GPIOC, 3, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));

    initialized = true;

    adc_enable();
}

void adc_enable(void)
{
    ASSERT(initialized);
    ADC3->CR2 |= ADC_CR2_ADON;
}

void adc_conversion_start(uint8_t channel)
{
    ADC3->SQR3 = 0;
    ADC3->SQR3 |= (channel << 0);

    ADC3->SR = 0;
    ADC3->CR2 |= (1 << 30);
}

void adc_conversion_wait(void)
{
    while (!(ADC3->SR & (1 << 1))) {}
}

uint16_t adc_value_get(void)
{
    return ADC3->DR;
}
