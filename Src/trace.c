#ifndef TRACE_DISABLE
#include "trace.h"
#include "uart.h"
#include "../external/printf/printf.h"
#include <stdbool.h>

void trace_initialize(void)
{
    uart2_initialize_assert();
}
/* Wrapping function of printf() to log the trace of errors in the assert handler.
 * The order of function calls goes like so:
 *  call "TRACE" macro -> trace() function -> printf -> putchar -> uart transmission
 */
void trace(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
#endif /* TRACE_DISABLE */

