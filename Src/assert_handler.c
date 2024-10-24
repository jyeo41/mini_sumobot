#include "assert_handler.h"
#include <stm32f4xx.h>

void assert_handler(void)
{
    /* Toggle a software breakpoint to track in the debugger where it got triggered
     * using the intrinsic function from CMSIS
     */
    __BKPT(0);
    while (1) {
        /* Should infinite loop while toggling LEDs */
    }
}
