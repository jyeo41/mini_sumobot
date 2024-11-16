#include "ir_receiver.h"
#include "gpio.h"
#include "assert_handler.h"
#include "led.h"
#include <stdbool.h>

#define SYSTEM_CLOCK            16000000
#define TIMER_TICK_EVERY_MS     SYSTEM_CLOCK/1000

static bool ir_receiver_initialized = false;
static bool timer_initialized = false;

static void timer_initialize(void);

void ir_receiver_initialize(void)
{
    ASSERT(!ir_receiver_initialized);

    #if 0 
    /* Setting the IR Receiver for pin PB0, GPIO Falling Edge Interrupt trigger */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    #endif

    gpio_configure_pin(IR_RECEIVER, GPIO_MODE_ALTERNATE, GPIO_AF1, GPIO_RESISTOR_DISABLED);
    timer_initialize();

    #if 0
    gpio_interrupt_set(IR_RECEIVER, GPIO_INTERRUPT_TRIGGER_FALLING, 6, 2);
    #endif

    ir_receiver_initialized = true;
}

ir_receiver_cmd_e ir_receiver_get_cmd(void);

#if 0
// cppcheck-suppress unusedFunction
void EXTI0_IRQHandler(void)
{
    led_toggle(LED_ORANGE);
    EXTI->PR |= (1 << 0);
}
#endif

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

    /* Prescaler set to tick every 1ms */
    TIM2->PSC = TIMER_TICK_EVERY_MS - 1;

    /* Set max value for reload register */
    TIM2->ARR = 0xFFFFFFFF;

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
    TIM2->DIER |= TIM_DIER_CC1IE;

    /* Enable the Timer 2 */
    TIM2->CR1 |= TIM_CR1_CEN;

    /* Enable TIM2 interrupt in the NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0);

    timer_initialized = true;
}

// cppcheck-suppress unusedFunction
void TIM2_IRQHandler(void)
{
    led_toggle(LED_ORANGE);
    TIM2->SR &= ~(TIM_SR_CC1IF);
}
