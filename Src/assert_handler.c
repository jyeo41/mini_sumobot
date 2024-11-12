#include <stm32f4xx.h>
#include "assert_handler.h"
#include "systick.h"
#include "trace.h"

/* The assertion should print where the assertion got triggered by showing the file and line number.
 * This functionality of tracing the code is just another tool to aid in debugging the software
 *
 * The original intention was to use the LED Drivers inside of the assert_handler, however this caused some issues.
 * The first issue is a recursive assert problem, where if I try to call led_toggle() for example and the assert fails,
 * then it would keep recursively calling the assert function since it will fail, come back to the assert_handler,
 * call the led_toggle which would fail the ASSERT again, repeatedly.
 *
 * The second issue is, what if the led_initialize() doesn't ever get called? This means the LEDs would actually never
 * blink and visually show on the board the program reached the assert handler.
 *
 * By manually enabling the clock gate, setting the mode, and toggling the blue and red LEDs inside of the assert_handler
 * bare metal, this decouples the assert_handler to work on its own if the led_driver file were to break. */
void assert_handler(const char* file, int line)
{
    trace_initialize();
    TRACE("Assertion Failed %s:%d", file, line);
    /* Toggle a software breakpoint to track in the debugger where it got triggered
     * using the intrinsic __BKPT() function from CMSIS. This breakpoint raises a SIGTRAP to GDB and completely
     * halts the program when its hit.
     *
     * The assert handler should still continue to the infinite loop of blinking the red and blue LEDs as a visual cue
     * the program failed an assert check when the debugger is not on. This is useful if after flashing the code something
     * goes on and the debugger has not been attached yet. If the debugger is on, then the breakpoint should hit and
     * automatically pause the debugger at this point since it's already known the program will enter the assert handler.
     */
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        __BKPT(0);
    }

    /* Enable the clock gate, and set both blue and red LEDs to GPIO Output in the MODE register */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= GPIO_MODER_MODER14_0;
    GPIOD->MODER |= GPIO_MODER_MODER15_0;
    while (1) {
        /* Should infinite loop while toggling LEDs to visually signal we're caught in the assert handler after flashing */
        GPIOD->ODR ^= GPIO_ODR_OD14;
        systick_delay_ms(500);
        GPIOD->ODR ^= GPIO_ODR_OD15;
        systick_delay_ms(500);
    }
}
