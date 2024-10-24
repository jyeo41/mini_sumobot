#include "assert_handler.h"
#include "led.h"
#include "gpio.h"
#include "systick.h"

/* The assertion should print where the assertion got triggered by showing the file and line number.
 * This functionality of tracing the code is just another tool to aid in debugging the software */
void assert_handler(void)
{
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

    while (1) {
        /* Should infinite loop while toggling LEDs to visually signal we're caught in the assert handler after flashing */
        led_toggle(LED_RED);
        systick_delay_ms(500);
        led_toggle(LED_BLUE);
        systick_delay_ms(500);
    }
}
