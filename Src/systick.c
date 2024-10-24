#include <stm32f4xx.h>
#include "systick.h"

/* Internal system clock */
#define SYSTEM_CLOCK_SPEED      16000000
#define TRIGGER_EVERY_SEC       SYSTEM_CLOCK_SPEED
#define TRIGGER_EVERY_MS        SYSTEM_CLOCK_SPEED / 1000

static uint32_t get_tick_counter(void);

uint32_t global_tick_counter;

/* Systick is a non GPIO Hardware level module so configure it using bare metal register writes.
 */
void systick_initialize(void)
{
    /* Disable the module while configuring it */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /* Set the clock source as the internal 16 MHz crystal */
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /* Load value to trigger the systick interrupt every millisecond, but subtract 1 because it includes decrementing to 0 */
    SysTick->LOAD = TRIGGER_EVERY_MS - 1U;

    /* Clear the current value to reset the timer by flagging the COUNTFLAG bit-16 in the SysTick->CTRL register*/
    SysTick->VAL = 0;

    /* Enable the systick interrupt */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    /* Enable Systick module to start it up */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/* Accurate hardware delay function to replace software for-loop delay functions.
 * Configured to take desired milliseconds as input.
 */
void systick_delay_ms(uint32_t delay)
{
    uint32_t current_tick_counter = get_tick_counter();
    while ((get_tick_counter() - current_tick_counter) <= delay) {}
}

/* Function to get the global tick counter.
 * Necessary because there could be race conditions when trying to access a global variable
 * without it being inside of a critical section.
 */
static uint32_t get_tick_counter(void)
{
    uint32_t local_tick_counter;
    __disable_irq();
    local_tick_counter = global_tick_counter;
    __enable_irq();

    return local_tick_counter;
}

// cppcheck-suppress unusedFunction
void SysTick_Handler(void)
{
    ++global_tick_counter;
}
