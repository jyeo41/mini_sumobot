#ifndef TRACE_H_
#define TRACE_H_

#define TRACE(fmt, ...) trace("%s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

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
