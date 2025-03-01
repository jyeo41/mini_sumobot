#include "ir_receiver.h"
#include "gpio.h"
#include "assert_handler.h"
#include "trace.h"
#include "led.h"
#include <stdint.h>
#include <stdbool.h>

/* 16MHz default HSI system clock. No prescalers being used
 * to feed into the APB1 Bus Clock nor the Timer clock.
 * Desired Frequency is 10us which is 1/10us = 100000MHz
 * Prescaler formula: (f_timer / f_desired) - 1
 * 
 * Note: US in macros means microseconds
 */
#define SYSTEM_CLOCK                    16000000
#define TIMER2_FREQ_TIMER               SYSTEM_CLOCK
#define TIMER2_FREQ_DESIRED             100000
#define TIMER2_PRESCALER                (TIMER2_FREQ_TIMER / TIMER2_FREQ_DESIRED) - 1
#define TIMER2_TICK_PERIOD_US           10

/* Defines for mapping Timer CNT/CCR1 read values to NEC Protocol timing requirement values.
 * Logical 0 - 1.125ms total transmit time, 562.5us pulse burst followed by 562.5us space
 * Logical 1 - 2.250ms total transmit time, 562.5us pulse burst followed by 1.6875ms space
 *
 * When a message is transmitted
 * Leading Pulse Width - 13.5ms
 * Total message transmit time - 67.5ms
 * 
 * If the key is held down, repeat codes will be sent.
 * The first repeat code will be sent roughly 40ms after the end of the message
 *  and if the button is held down, repeat codes will continue to be sent out in 108ms intervals.
 * Leading Pulse Width - 11.25ms
 *
 * Use 562us to represent 562.5us for faster integer division rather than floating point
 */
#define TIME_562_US                     (562 / TIMER2_TICK_PERIOD_US)

#define TIME_1_125_MS                   (TIME_562_US * 2)
#define TIME_1_125_MS_DEVIATION         (TIME_1_125_MS / 6)
#define TIME_1_125_MS_LOW               (TIME_1_125_MS - TIME_1_125_MS_DEVIATION)
#define TIME_1_125_MS_HIGH              (TIME_1_125_MS + TIME_1_125_MS_DEVIATION)

#define TIME_2_25_MS                    (TIME_1_125_MS * 2)
#define TIME_2_25_MS_DEVIATION          (TIME_2_25_MS / 6)
#define TIME_2_25_MS_LOW                (TIME_2_25_MS - TIME_2_25_MS_DEVIATION)
#define TIME_2_25_MS_HIGH               (TIME_2_25_MS + TIME_2_25_MS_DEVIATION)

#define TIME_11_25_MS                   (TIME_2_25_MS * 5)
#define TIME_11_25_MS_DEVIATION         (TIME_11_25_MS / 16)
#define TIME_11_25_MS_LOW               (TIME_11_25_MS - TIME_11_25_MS_DEVIATION)
#define TIME_11_25_MS_HIGH              (TIME_11_25_MS + TIME_11_25_MS_DEVIATION)

#define TIME_13_5_MS                    (TIME_2_25_MS * 6)
#define TIME_13_5_MS_DEVIATION          (TIME_13_5_MS / 16)
#define TIME_13_5_MS_LOW                (TIME_13_5_MS - TIME_11_25_MS_DEVIATION)
#define TIME_13_5_MS_HIGH               (TIME_13_5_MS + TIME_11_25_MS_DEVIATION)

#define CMD_BUFFER_LENGTH               64
#define TIMER2_ARR_VALUE                0xFFFF

/* union to be able to access each individual byte of the message or the full 32 bit message at once */
static union {
    struct {
        // cppcheck-suppress unusedStructMember
        uint8_t command_inverted;
        uint8_t command;
        // cppcheck-suppress unusedStructMember
        uint8_t address_inverted;
        // cppcheck-suppress unusedStructMember
        uint8_t address;
    }bytes;
    uint32_t full;
}ir_message;

static bool ir_receiver_initialized = false;
static bool timer_initialized = false;
static volatile uint32_t cmd_buffer[CMD_BUFFER_LENGTH];
static volatile uint8_t cmd_put_ptr = 0;
static volatile uint8_t cmd_get_ptr = 0;

/* Variables to be used inside of timer2 ISR for decoding the NEC signal pulse width */
static uint8_t pulse_count = 0;
static uint32_t current_time = 0;
static uint32_t previous_time = 0;
static uint32_t time_diff = 0;


static void timer_initialize(void);
static void timer2_interrupt_clear(void);
static void timer2_interrupt_channel1_enable(void);
static void timer2_interrupt_channel1_disable(void);
static uint32_t timer2_count_get(void);

void ir_receiver_initialize(void)
{
    ASSERT(!ir_receiver_initialized);

    gpio_configure_pin(IR_RECEIVER, GPIO_MODE_ALTERNATE, GPIO_AF1, GPIO_RESISTOR_DISABLED,
                       GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM);
    ASSERT(gpio_config_compare(IR_RECEIVER, GPIOA, 15, GPIO_MODE_ALTERNATE, GPIO_RESISTOR_DISABLED,
                               GPIO_OTYPE_PUSHPULL, GPIO_SPEED_MEDIUM));
    timer_initialize();

    ir_receiver_initialized = true;
}

// cppcheck-suppress unusedFunction
const char* ir_receiver_get_cmd(void)
{
    /* Handle possible race condition of reading from ring buffer and ISR potentially writing to it
     * at the same time.
     */
    ir_receiver_cmd_e ir_cmd = IR_RECEIVER_CMD_NONE;

    timer2_interrupt_channel1_disable();

    /* Extract command from command buffer if the ring buffer is not empty.
     * If get and put ptr are ==, this means it is empty.
     */
    if (cmd_get_ptr != cmd_put_ptr) {
        ir_cmd = cmd_buffer[cmd_get_ptr++];
        cmd_get_ptr &= (CMD_BUFFER_LENGTH - 1);
    }
    timer2_interrupt_channel1_enable();

    /* Big switch statement to convert enum integer to string to print it out to trace. */
    switch(ir_cmd) {
        case IR_RECEIVER_CMD_0:
            return "0";
        case IR_RECEIVER_CMD_1:
            return "1";
        case IR_RECEIVER_CMD_2:
            return "2";
        case IR_RECEIVER_CMD_3:
            return "3";
        case IR_RECEIVER_CMD_4:
            return "4";
        case IR_RECEIVER_CMD_5:
            return "5";
        case IR_RECEIVER_CMD_6:
            return "6";
        case IR_RECEIVER_CMD_7:
            return "7";
        case IR_RECEIVER_CMD_8:
            return "8";
        case IR_RECEIVER_CMD_9:
            return "9";
        case IR_RECEIVER_CMD_UP:
            return "UP";
        case IR_RECEIVER_CMD_DOWN:
            return "DOWN";
        case IR_RECEIVER_CMD_LEFT:
            return "LEFT";
        case IR_RECEIVER_CMD_RIGHT:
            return "RIGHT";
        case IR_RECEIVER_CMD_VOL_DECREASE:
            return "VOL_DECREASE";
        case IR_RECEIVER_CMD_VOL_INCREASE:
            return "VOL_INCREASE";
        case IR_RECEIVER_CMD_PLAY_PAUSE:
            return "PLAY_PAUSE";
        case IR_RECEIVER_CMD_SETUP:
            return "SETUP";
        case IR_RECEIVER_CMD_STOP_OR_MODE:
            return "STOP_OR_MODE";
        case IR_RECEIVER_CMD_ENTER_OR_SAVE:
            return "ENTER_OR_SAVE";
        case IR_RECEIVER_CMD_UNDO:
            return "UNDO";
        case IR_RECEIVER_CMD_NONE:
            return "NONE";
        default:
            return "UNKNOWN";
    }
}

static void timer_initialize(void)
{
    /* Common registers to configure for GP timers.
     *  - PSC (Prescaler)
     *  - ARR (Auto Reload)
     *  - CR1 (Control Register)
     *  - SR  (Status Register)
     *  - CCR (Capture Compare)
     *  - CCMR (Capture Compare MODE Register): Configuring CC functionality, such as edge config
     *  - CCER (Capture Compare Enable Register)
     */  
    ASSERT(!timer_initialized);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* Enable clock for Timer 2 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Set as up counter */
    TIM2->CR1 &= ~(TIM_CR1_DIR);

    /* Set max value for reload register */
    TIM2->ARR = TIMER2_ARR_VALUE - 1;

    /* Prescaler set to tick every 10 microsecond, MUST SUBTRACT 1, this is done in the macro */
    TIM2->PSC = TIMER2_PRESCALER;

    /* Configure TIM2 channel 1 (PA15) for Input Capture. 
     * Write CC1S bits to 01 in the TIMx_CCMR1 register.
     */
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S);
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;

    /* Input capture filter and prescaler seems optional.
     * Clear them both for safe measure.
     */
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC1PSC);
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC1F);

    /* Set Input capture polarity to falling edge.
     * 2 bit field with a reserved bit in between.
     * 00 for rising edge
     * 01 for falling edge
     * 11 for both edges
     */
    TIM2->CCER |= TIM_CCER_CC1P;
    TIM2->CCER &= ~(TIM_CCER_CC1NP);

    /* Enable capture for channel 1*/
    TIM2->CCER |= TIM_CCER_CC1E;
    
    /* Enable interrupt request */
    timer2_interrupt_channel1_enable();

    /* Enable the Timer 2 */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* Enable TIM2 interrupt in the NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    timer_initialized = true;
}

#if 0
static uint32_t current_dump[64];
static uint8_t current_dump_index = 0;
static uint32_t time_diff_dump[64];
static uint8_t time_diff_dump_index = 0;
#endif

// cppcheck-suppress unusedFunction
void TIM2_IRQHandler(void)
{
    //led_toggle(LED_ORANGE);
    current_time = timer2_count_get();
    if (current_time > previous_time) {
        time_diff = current_time - previous_time;
    } else {
        /* Account for overflow of timer when it inevitably hits the max ARR value.
         * This is for upcounter logic.
         */
        time_diff = (TIMER2_ARR_VALUE - previous_time) + current_time;
    }
    previous_time = current_time;

    #if 0
    current_dump[current_dump_index++] = current_time;
    current_dump_index &= 64 - 1;
    time_diff_dump[time_diff_dump_index++] = time_diff;
    time_diff_dump_index &= 64 - 1;

    if (current_dump_index > 18 && current_dump_index < 35) {
        __asm("NOP");
    }
    #endif

    if ((time_diff > TIME_1_125_MS_LOW) && (time_diff < TIME_1_125_MS_HIGH)) {
        /* Logical 0 pulse width of 1.125ms. Shift in a 0 into the message. */
        pulse_count++;
        ir_message.full <<= 1;
    } else if ((time_diff > TIME_2_25_MS_LOW) && (time_diff < TIME_2_25_MS_HIGH)) {
        /* Logical 1 pulse width burst of 2.25ms. Shift in a 1 into the message. */
        pulse_count++;
        ir_message.full <<= 1;
        ir_message.full += 1;
    } else if ((time_diff > TIME_11_25_MS_LOW) && (time_diff < TIME_11_25_MS_HIGH)) {
        /* Pulse width of 11.25ms, repeat code pulse width duration.
         * Just send the current ir_message command again to cmd buffer.
         */
        cmd_buffer[cmd_put_ptr++] = ir_message.bytes.command;
        cmd_put_ptr &= (CMD_BUFFER_LENGTH - 1);
    } else if ((time_diff > TIME_13_5_MS_LOW) && (time_diff < TIME_13_5_MS_HIGH)) {
        /* Pulse burst of 13.5ms, message pulse width duration.
         * Clear the ir_message since its a new incoming message.
         */
        ir_message.full = 0;
    } 

    if (pulse_count == 32) {
        /* Full message has been decoded, add it to ring buffer so application can use it.
         * Reset the pulse count to 0 because the full message has been received.
         * Otherwise, if repeat codes are detected, duplicate commands will be added.
         */
        pulse_count = 0;
        cmd_buffer[cmd_put_ptr++] = ir_message.bytes.command;
        cmd_put_ptr &= (CMD_BUFFER_LENGTH - 1);
    }

    /* On input capture, reading the TMx_CCR1 register clears the CC1IF interrupt flag, but its still good practice
     * to explicitly clear this bit.
     */
    timer2_interrupt_clear();
}

/* Read from CCR1 register, NOT from CNT because CCR1 is a snapshot directly from the hardware */
static uint32_t timer2_count_get(void)
{
    return TIM2->CCR1;
}

static void timer2_interrupt_clear(void)
{
    TIM2->SR &= ~(TIM_SR_CC1IF);
}

static void timer2_interrupt_channel1_disable(void)
{
    TIM2->DIER &= ~(TIM_DIER_CC1IE);
}

static void timer2_interrupt_channel1_enable(void)
{

    TIM2->DIER |= TIM_DIER_CC1IE;
}
