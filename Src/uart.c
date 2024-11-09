#include <stm32f4xx.h>
#include <stdint.h>
#include "uart.h"
#include "gpio.h"
#include "assert_handler.h"
#include "systick.h"

#define DESIRED_BAUDRATE    115200
#define SYSTEM_CLOCK        16000000
#define APB1_CLOCK          SYSTEM_CLOCK
#define RING_BUFFER_LENGTH  64

/* Variables being used inside ISRs should be declared as volatile */
static volatile uint8_t tx_buffer[RING_BUFFER_LENGTH];
static volatile uint16_t tx_put_ptr = 0;
static volatile uint16_t tx_get_ptr = 0;

bool initialized = false;

static void uart2_set_baudrate(uint32_t peripheral_clock, uint32_t baudrate);
static void uart2_interrupt_tx_enable(void);
static void uart2_interrupt_tx_disable(void);

void uart2_initialize(void)
{
    ASSERT(!initialized);
    /* Set the clock gate to enable Port B for UART1, PB6 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* Set the UART pin to alternate function, and also AF7 for UART.
     * Refer to datasheet, Chapter 3 Pinouts and Pin Description, Table 9 for AF mappings.
     * Refer to Reference Manual, Chapter 8 GPIO, 8.3.2 I/O pin multiplexer and mapping for specific AF number */
    gpio_mode_set(UART2_BOARD_TX, GPIO_MODE_ALTERNATE);
    gpio_alternate_function_set(UART2_BOARD_TX, GPIO_AF7);


    /* Enable the clock gate for UART */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    /* Delay to let the clock settle to avoid immediately sending garbage characters. */
    systick_delay_ms(50);

    /* Program the M bit in USART_CR1 to define the word length.
     * Clear it to set 1 start bit, 8 data bits, n Stop bit
     */
    USART2->CR1 &= ~(USART_CR1_M);

    /* Select the desired baud rate using USART_BRR register */
    uart2_set_baudrate(APB1_CLOCK, DESIRED_BAUDRATE);

    /* Set the Transmit Enable bit TE in USART_CR1 */
    USART2->CR1 = USART_CR1_TE;

    /* Enable the USART Module by writing to UE bit in USART_CR1 */
    USART2->CR1 |= USART_CR1_UE;

    /* Enable the USART2 which is IRQ 38 */
    NVIC_EnableIRQ(38);

    initialized = true;
}

///* Write the data to send in the USART_DR register (this clears the TXE bit) and repeat this
// * for each data to be transmitted in case of single buffer.
// *
// * After writing the last data into USART_DR, wait until TC=1 which indicates the transmission of the
// * last frame is complete. This is required for instance when USART is disabled or enters the halt mode to
// * avoid corrupting the last transmission.
// */
//// cppcheck-suppress unusedFunction
//void uart2_send_char(uint8_t c)
//{
//    /* Some terminals require a carriage return \r before a linefeed \n for proper newlining. */
//    ASSERT(initialized);
//    if (c == '\n') {
//        uart2_send_char('\r');
//    }
//
//    USART2->DR = c;
//    while (!(USART2->SR & USART_SR_TXE)) {}
//
//}

void uart_send_string(const char* string)
{
    ASSERT(initialized);
    while(*string) {
        uart2_interrupt_send_char(*string++);
    }
}

void uart2_interrupt_send_char(uint8_t c)
{
    /* Disable TX Interrupt while writing to the tx buffer.
     * Don't want an interrupt triggering while putting in data it would be a race condition. */
    uart2_interrupt_tx_disable();

    ASSERT(initialized);
    if (c == '\n') {
        uart2_interrupt_send_char('\r');
    }

    /* Put the char I want to send into the tx ring buffer, increment array pointer after */
    tx_buffer[tx_put_ptr++] = c;

    /* Instead of using expensive modulo operation, use bitwise AND to keep it within the array bounds.
     * This trick only works if the RING_BUFFER_LENGTH is a power of 2.
     */
    tx_put_ptr &= (RING_BUFFER_LENGTH - 1);

    /* Once there is data to be transmitted inside of the tx ring buffer, enable transmission interrupt */
    uart2_interrupt_tx_enable();
}

// cppcheck-suppress unusedFunction
void USART2_IRQHandler(void)
{
    /* TXE bit set means the TDR register is empty. THe next data can be written to the USART_DR register without
     * overwriting the previous data.
     * We need to first check and make sure this flag is set.*/
    if (USART2->CR1 & USART_SR_TXE) {
        /* If the two pointers do not match, that means the tx ring buffer is NOT empty and contains data that needs
         * to be sent. If the pointers are equal, that means it is empty. */
        if (tx_put_ptr != tx_get_ptr) {
            USART2->DR = tx_buffer[tx_get_ptr++];
            tx_get_ptr &= (RING_BUFFER_LENGTH - 1);
        } else {
            /* If the ring buffer is empty, then no transmission should occur.
             * Disable TX Interrupts */
            uart2_interrupt_tx_disable();
        }
    }
}

static void uart2_interrupt_tx_enable(void)
{
    USART2->CR1 |= USART_CR1_TXEIE;
}

static void uart2_interrupt_tx_disable(void)
{
    USART2->CR1 &= ~(USART_CR1_TXEIE);
}
/* Subroutine to calculate the baudrate using OVER8 = 0 from the given formula in the reference manual.
 * Pg: 519  Sec: 19.3.4 (Fractional baud rate generation)
 */
static void uart2_set_baudrate(uint32_t peripheral_clock, uint32_t baudrate)
{
    uint32_t mantissa_int;
    double mantissa_double;
    double fraction;

    uint16_t USARTDIV_MANT;
    uint16_t USARTDIV_FRAC;

    /*
     * IF we use OVER8 = 0, the formula in equation #1 reduces to the following:
     * USARTDIV = Fck / (8 * (2 - 0) * Tx/Rx Baudrate)
     * We are using 16MHz clock and desired baudrate of 115200
     *
     * USARTDIV = 16000000 / (16 * 115200)
     * USARTDIV = 8.680555...
     * Mantissa = 8
     * Fractional = .6805...
     * To turn the fractional part into a 16 bit number because DIV_fraction[3:0] is 2^4, we multiply by 16 to get
     * .6806 * 16 = 10.8896
     * The nearest whole number is 11 rounded up
     *
     * Reference Manual pg:550, sec:19.6.3 USART_BRR (Baud Rate Register)
     * The mantissa binary of 8 is 0b 0000 0000 1000 (bits 11:0 of USART_BRR)
     * The fractional binary of 11 is 0b 1011 (bits 3:0 of USART_BRR)
     * The entire 16 bit mask is 0b 0000 0000 0000 0000 0000 0000 1000 1011
     * This is the value we write to USART_BRR
     */

    mantissa_double = (double)peripheral_clock / (double)(16 * baudrate);
    mantissa_int = (uint32_t)mantissa_double;
    fraction = (mantissa_double - (uint32_t)mantissa_int);

    /* Round up if necessary */
    if ((fraction * 16) - fraction >= 0.5) {
        fraction *= 16;
        fraction += 1;
    } else {
        fraction *= 16;
    }
    USARTDIV_MANT = (uint16_t)mantissa_int;
    USARTDIV_FRAC = (uint16_t)fraction;

    USART2->BRR = (USARTDIV_MANT << 4) | (USARTDIV_FRAC << 0);
}
