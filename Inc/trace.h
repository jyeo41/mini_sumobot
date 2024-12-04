#ifndef TRACE_H_
#define TRACE_H_

/* Macro to serve as a shorthand with same syntax as using standard printf() that also prints
 * the file and line numbers.
 *
 * Example:
 * TRACE("Error code: %d", error_code) would expand to -> trace("file.c:42: Error code: %d\n", "file.c", 42, error_code)
 *
 * "fmt" represents the format string for the log message.
 * "..." represents variadic arguments to hold the placeholders in the above format string.
 *
 * Inside the trace() function:
 * "%s:%d: " fmt "\n" is a combined format string using the file name/line as a prefix to the user provided fmt string.
 * __FILE__ and __LINE__ are predefined macros.
 * ##__VA_ARGS__ "variadic arguments" expands to the additional arguments provided to the TRACE macro call with "..."
 *	It represents any additional arguments after "fmt" because "fmt" represents the first argument of TRACE 
 */
#define TRACE(fmt, ...) trace("%s:%d: " fmt, __FILE__, __LINE__, ##__VA_ARGS__)

/* Additional define to easily disable and enable traces in the build.
 * Traces may affect the timing of the code so useful to turn them all off at once. 
 */
#ifndef TRACE_DISABLE
void trace_initialize(void);
void trace(const char* format, ...);
#else
#define trace_initialize() ;
#define trace(fmt, ...) ;
#endif

#endif /* TRACE_H_ */
