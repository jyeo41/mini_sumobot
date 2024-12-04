#include "adc.h"
#include "gpio.h"
#include "assert_handler.h"
#include "trace.h"
#include "led.h"
#include <stm32f4xx.h>
#include <stdbool.h>

#define ADC_CHANNELS_USED       4

/* ADC Channels used for sampling */
#define ADC123_CHANNEL_10_SQR  10
#define ADC123_CHANNEL_11_SQR  11
#define ADC123_CHANNEL_12_SQR  12
#define ADC123_CHANNEL_13_SQR  13

static volatile uint16_t adc_dma_buffer[ADC_CHANNELS_USED];
static volatile uint16_t adc_dma_buffer_safe[ADC_CHANNELS_USED];

static uint16_t adc_dma_print_values[ADC_CHANNELS_USED];

static bool adc_initialized = false;
static bool dma_initialized = false;

static void adc_dma_initialize(void);
static void adc_enable(void);
static void adc_conversion_start(void);
static void adc_dma_interrupt_transfer_complete_enable(void);
static void adc_dma_trace_values(void);

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
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* Set prescaler for ADC clock to 2, by clearing both bits to 0.
     * The max for ADCCLK is 36 MHz. */
    ADC->CCR &= ~(ADC_CCR_ADCPRE);

    /* Set scan mode bit because we are using multiple channels.
     * Resolution to 12 bits by clearing both bits. */
    ADC3->CR1 |= ADC_CR1_SCAN;
    ADC3->CR1 &= ~(ADC_CR1_RES);

    /* Set single conversion mode by clearing CONT bit.
     * EOC flag to set at the end of each sequence of regular conversions by clearing this bit. 
     *  Overrun detection is enabled only if DMA = 1
     * Set right alignment by clearing ALIGN bit. */
    ADC3->CR2 &= ~(ADC_CR2_CONT);
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

/* This function is really only used for testing purposes and to check the values being read by the ADC
 * can be properly printed out into terminal over UART while using a potentiometer or jumper wire to manually 
 * pull each of the GPIO pins.
 *
 * The adc_dma_print_values[] array is specifically used to trace/print the ADC values.
 * For some reason, trying to print directly from the adc_dma_buffer_safe inside of a critical section 
 *  causes issues with printing values to a serial terminal.
 * As a workaround, copy the values from adc_dma_buffer_safe to adc_dma_print_values so the TRACE macro can print independently
 *  from an array that isn't reliant on being inside of a critical section.
 */
void adc_dma_get_values(void)
{
    /* Briefly disable global interrupts to handle race condition while reading from data buffer. 
     * For some reason, disabling ONLY the DMA transfer complete interrupt causes unknown issues.
     */
    __disable_irq();
    for (uint8_t i = 0; i < ADC_CHANNELS_USED; i++) {
        adc_dma_print_values[i] = adc_dma_buffer_safe[i];
    }
    __enable_irq();
    adc_dma_trace_values();
}

/* Function to be used with adc_dma_get_values() for debugging and testing purposes */
static void adc_dma_trace_values(void)
{
    for (uint8_t i = 0; i < ADC_CHANNELS_USED; i++) {
        TRACE("Channel %u: %u", (i + 10), adc_dma_print_values[i]);
    }
}

// cppcheck-suppress unusedFunction
void DMA2_Stream0_IRQHandler(void)
{
    //led_toggle(LED_ORANGE);
    /* Transfer complete interrupt flag. Clear the flag in software. Copy values to adc_dma_buffer_safe[] */
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
        for (uint8_t i = 0; i < ADC_CHANNELS_USED; i++) {
            adc_dma_buffer_safe[i] = adc_dma_buffer[i];
        }

        /* Reference Manual states the DMA bit in ADCx CR2 MUST be cleared and rewritten in software to start
         * a new transfer in single conversion mode. ADC Section 13.8.1 "Using the DMA" 
         * I am using direct register writes to keep the ISR decoupled from other functions as much as possible.
         *
         * First clear and write DMA bit in ADC->CR2
         * Reset the reload value in DMA_SxNDTR
         * Enable the stream in DMA_SxCR_EN
         * Start the ADC conversion by writing to ADC_CR2_SWSTART */
        ADC3->CR2 &= ~(ADC_CR2_DMA);
        ADC3->CR2 |= ADC_CR2_DMA;
        DMA2_Stream0->NDTR = ADC_CHANNELS_USED;
        DMA2_Stream0->CR |= DMA_SxCR_EN;
        ADC3->CR2 |= ADC_CR2_SWSTART;
    }
    
    /* Transfer error interrupt. */
    if (DMA2->LISR & DMA_LISR_TEIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
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
     *  I will use Stream 0 Channel 2
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

    /* Enable DMA and and set single conversion for DMA by clearing DDS bit in CR2 */
    ADC3->CR2 |= ADC_CR2_DMA;
    ADC3->CR2 &= ~(ADC_CR2_DDS);

    /* Disable stream first and read the bit until its 0. */
    DMA2_Stream0->CR &= ~(DMA_SxCR_EN);
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

    /* Set Direction to peripheral-to-memory mode since we are transferring from ADC to SRAM
     * Clear both bits. */
    DMA2_Stream0->CR &= ~(DMA_SxCR_DIR);

    /* Set source (PAR) and destination addresses M0AR) */
    DMA2_Stream0->PAR = (uint32_t)(&(ADC3->DR));
    DMA2_Stream0->M0AR = (uint32_t)adc_dma_buffer;
    
    /* Set number of transfers */
    DMA2_Stream0->NDTR = (uint16_t)ADC_CHANNELS_USED;

    /* Set the channel to be used */
    DMA2_Stream0->CR |= (2 << DMA_SxCR_CHSEL_Pos);

    /* Set peripheral increment to 0 because ADC->DR is a static memory address */
    DMA2_Stream0->CR &= ~(DMA_SxCR_PINC);
    
    /* Set memory increment to 1 because we want to increment the index of the destination array buffer */
    DMA2_Stream0->CR |= DMA_SxCR_MINC;

    /* Disable DMA Circular mode, we want non-circular mode */
    DMA2_Stream0->CR &= ~(DMA_SxCR_CIRC);

    /* Set peripheral and memory size to half word byte width */
    DMA2_Stream0->CR |= (1 << DMA_SxCR_MSIZE_Pos);
    DMA2_Stream0->CR |= (1 << DMA_SxCR_PSIZE_Pos);

    /* DMA Set to direct mode, no FIFO */
    DMA2_Stream0->FCR &= ~(DMA_SxFCR_DMDIS);

    /* Enable transfer complete interrupt. Clear corresponding event flag prior, otherwise an interrupt is generated immediately. */
    adc_dma_interrupt_transfer_complete_enable();

    /* Enable transfer error interrupt. Clear corresponding event flag prior. */
    DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
    DMA2_Stream0->CR |= DMA_SxCR_TEIE;

    /* Enable interrupts from the processor side. */
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 1);

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
    ADC3->CR2 |= ADC_CR2_SWSTART;
}

#if 0
static void adc_dma_interrupt_transfer_complete_disable(void)
{
    /* Disable transfer complete interrupt briefly when reading from array */
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
    DMA2_Stream0->CR &= ~(DMA_SxCR_TCIE);
}
#endif


static void adc_dma_interrupt_transfer_complete_enable(void)
{
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
    DMA2_Stream0->CR |= DMA_SxCR_TCIE;
}
