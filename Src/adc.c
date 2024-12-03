#include "adc.h"
#include "gpio.h"
#include "assert_handler.h"
#include "trace.h"
#include <stm32f4xx.h>
#include <stdbool.h>

#define ADC_CHANNELS_USED       4
#define ADC123_CHANNEL_10_SQR  10
#define ADC123_CHANNEL_11_SQR  11
#define ADC123_CHANNEL_12_SQR  12
#define ADC123_CHANNEL_13_SQR  13

static uint16_t adc_dma_buffer[ADC_CHANNELS_USED];

static bool adc_initialized = false;
static bool dma_initialized = false;

static void adc_dma_initialize(void);
static void adc_enable(void);
static void adc_conversion_start(void);

void adc_initialize(void)
{
    ASSERT(!adc_initialized);
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
     * EOC flag to set at the end of each sequence of regular conversions by clearing this bit. 
     *  Overrun detection is enabled only if DMA = 1
     * Set right alignment by clearing ALIGN bit. */
    ADC3->CR2 |= ADC_CR2_CONT;
    ADC3->CR2 &= ~(ADC_CR2_EOCS);
    ADC3->CR2 &= ~(ADC_CR2_ALIGN);

    /* Set sampling time to be simple at 3 cycles by clearing all 3 bits. 
     * Speed doesn't matter too much here because we're gated by the system clock of 16MHz and
     *  superloop architecture of the code. */
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP4);
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP5);
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP6);
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP7);

    /* Regular channel sequence length, set to 4 conversions because we are using 4 channels.
     * Have to integer written to bitfield should be total number of channels used - 1*/
    ADC3->SQR1 |= ((ADC_CHANNELS_USED - 1) << ADC_SQR1_L_Pos);

    /* Set sequence order of channels to be sampled */
    ADC3->SQR3 |= (ADC123_CHANNEL_10_SQR << ADC_SQR3_SQ1_Pos);
    ADC3->SQR3 |= (ADC123_CHANNEL_11_SQR << ADC_SQR3_SQ2_Pos);
    ADC3->SQR3 |= (ADC123_CHANNEL_12_SQR << ADC_SQR3_SQ3_Pos);
    ADC3->SQR3 |= (ADC123_CHANNEL_13_SQR << ADC_SQR3_SQ4_Pos);
    
    gpio_configure_pin(ADC123_CHANNEL10, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_configure_pin(ADC123_CHANNEL11, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_configure_pin(ADC123_CHANNEL12, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);
    gpio_configure_pin(ADC123_CHANNEL13, GPIO_MODE_ANALOG, GPIO_AF_NONE, GPIO_RESISTOR_DISABLED);

    /* Sanity check to make sure the pins are initialized properly */
    ASSERT(gpio_config_compare(ADC123_CHANNEL10, GPIOC, 0, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));
    ASSERT(gpio_config_compare(ADC123_CHANNEL11, GPIOC, 1, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));
    ASSERT(gpio_config_compare(ADC123_CHANNEL12, GPIOC, 2, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));
    ASSERT(gpio_config_compare(ADC123_CHANNEL13, GPIOC, 3, GPIO_MODE_ANALOG, GPIO_RESISTOR_DISABLED));

    adc_initialized = true;

    /* DMA initialization */
    adc_dma_initialize();

    adc_enable();
    adc_conversion_start();
}

void adc_dma_print_values(void)
{
    for (uint8_t i = 0; i < ADC_CHANNELS_USED; i++) {
        TRACE("Channel %u Value: %u", (i + 4), adc_dma_buffer[i]);
    }
}

/* DMA configuration summary can be found in Section 10.3.17 "Stream Configuration Procedure" */
static void adc_dma_initialize(void)
{
    ASSERT(!dma_initialized);
    /* Enable DMA clock in RCC register
     * Enable DMA and continuous DMA conversion in CR2
     * Check DMA channel mapping in the reference manual and choose accordingly,
     *  ADC 3 uses DMA2 Stream 0 Channel 2 or Stream 1 Channel 2
     * Enable DMAx clock in RCC AHB1 
     *
     * Configure the stream in DMA SxCR:
     * Disable the stream first by resetting the EN Bit, then read this bit to confirm there is no ongoing
     *      stream operation. Writing this bit to 0 is not immediately effective since it is actually written
     *      to 0 once all the current transfers have finished. ONLY when it has been read as 0, the stream
     *      is ready to be configured.
     * Set source Address in SxPAR
     * Set destination Address in SxM0AR
     * NDTR set to 4(?) because we are sampling from 4 channels
     * Select the DMA channel (request) using CHSEL[2:0] in the DMA_SxCR register
     *
     * SxCR DIR set to 0b00 because we are using ADC Peripheral to memory with DMA 
     * SxCR PINC set to 0, since we are only copying from a static address ADC Data Register
     * SxCR MINC set to 1, the buffer will auto increment and next data will be saved in new location
     * SxCR CIRC set to 1, circular mode for infinite data transfer. Possibly change in the future? 
     * SxCR PSIZE/MSIZE set to 0b01 for 16 bit half word size, we are using 12 bit ADC values
     * SxFCR DMDIS bit set to 0 for direct mode, no FIFO to make it simpler
     *
     * DMA Low/High interrupt status registers and their clear flags used for interrupt implementation
     *
     * NOTE: DMA Transfer Completion. In DMA Flow controller mode (what we're using), the TCIFx bit signals
     * the transfer is complete in LISR or HISR. Afterwards, the DMA flushes any remaining data from the FIFO
     * if its being used, and disables the stream by clearing the EN bit in the SxCR register.
     *
     *
     * WARNING: To switch off a peripheral connected to a DMA stream request, it is MANDATORY
     * to first switch off the corresponding DMA stream and then wait for the EN bit = 0.
     * Afterwards the peripheral can be safely disabled.
     *
     * Only enable DMA in CR bit 0 as the final thing, because some registers turn read only when its enabled. */

    /* Enable clock*/
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* Enable DMA and and set continuous conversion for DMA by setting DDS bit in CR2 */
    ADC3->CR2 |= ADC_CR2_DMA;
    ADC3->CR2 |= ADC_CR2_DDS;

    /* Disable stream first and read the bit until its 0. */
    DMA2_Stream0->CR &= ~(DMA_SxCR_EN);
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

    /* Set Direction to peripheral-to-memory mode since we are transferring from ADC to SRAM
     * Clear both bits. */
    DMA2_Stream0->CR &= ~(DMA_SxCR_DIR);

    /* Set source (PAR) and destination addresses M0AR) */
    DMA2_Stream0->PAR = (uint32_t)(&(ADC3->DR));
    DMA2_Stream0->M0AR = (uint32_t)(&adc_dma_buffer);
    
    /* Set number of transfers */
    DMA2_Stream0->NDTR = (uint16_t)ADC_CHANNELS_USED;

    /* Set the channel to be used */
    DMA2_Stream0->CR |= (2 << DMA_SxCR_CHSEL_Pos);

    /* Set peripheral increment to 0 because ADC->DR is a static memory address */
    DMA2_Stream0->CR &= ~(DMA_SxCR_PINC);
    
    /* Set memory increment to 1 because we want to increment the index of the destination array buffer */
    DMA2_Stream0->CR |= DMA_SxCR_MINC;

    /* Set DMA Circular mode enabled */
    DMA2_Stream0->CR |= DMA_SxCR_CIRC;

    /* Set peripheral and memory size to half word byte width */
    DMA2_Stream0->CR |= (1 << DMA_SxCR_MSIZE_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_PSIZE_Pos);

    /* DMA Set to direct mode, no FIFO */
    DMA2_Stream0->FCR &= ~(DMA_SxFCR_DMDIS);

    /* Enable DMA stream as final step */
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    
    dma_initialized = true;
}

static void adc_enable(void)
{
    ASSERT(adc_initialized && dma_initialized);
    ADC3->CR2 |= ADC_CR2_ADON;
}

static void adc_conversion_start(void)
{
    ADC3->SR = 0;
    ADC3->CR2 |= ADC_CR2_SWSTART;
}
