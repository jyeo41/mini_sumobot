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
    /* va_list is a type defined in <stdarg.h> and is used to retrieve the addtional arguments passed to the variadic function.
     *      This holds the arguments from "..." passed into the trace() function.
     *
     * va_start is a macro that initializes "va_list args" from the previous line so it can be used to access the variadic arguments.
     *      The order of parameters (args, format) is important because va_start needs to know the LAST FIXED PARAMETER before
     *      the variadic arguments. So if trace was defined like so trace(const char *format, const int number, ...)
     *      then va_start would need to be initialized as va_start(args, number).
     *
     * vprintf() is a variation of printf() that takes in the list of arguments (va_list) instead of each variable argument directly.
     *      It uses the "va_list args" to replace the format specifiers in "format" with the corresponding values in "args". 
     *
     * va_end() is a macro that cleans up the "args" variable after the variadic arguments have been processed.
     *      This is important to do after you're done with the variadic arguments to avoid potential stack corruption or
     *      accessing invalid memory later down the line. This is to ensure the stack is correctly restored to its state before
     *      va_start was called.
     */
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
#endif /* TRACE_DISABLE */

