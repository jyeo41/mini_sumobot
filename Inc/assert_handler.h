#ifndef ASSERT_HANDLER_
#define ASSERT_HANDLER_

/* Purpose of a custom assert handler in embedded systems is to be used for error handling and debugging.
 * The idea is to be able to easily see where the assert handler got called in the function call stack while debugging,
 * and use LEDs to visually show the program is stuck in the assert handler. It's an easy and simple way to see
 * an assertion failed and also makes it easy to track it down where something went wrong in the code.
 * Assert handling can also capture information like the file name, line number, and expression that failed.
 *
 * The practical applications of using an assert_handler is for sanity checking configuration settings inside of init() functions.
 *	- Checking configurations to match expectations
 *	- Checking to make sure init() functions are called before using functions within that module such as setting output
 *
 * First wrap an assert macro in a define. This is standard industry practice to switch it on or off
 * at compile time if you choose to do so.
 *
 * The do while loop is a common idiom used in C to create a macro that behaves like a single statement. It ensures the macro
 * can be used safely with or without surrounding braces. It mainly prevents potential issues when the macro is used in if-else
 * statements.
 */
#define ASSERT(expression)	\
	do { \
		if (!(expression)) { \
			assert_handler(__FILE__, __LINE__); \
		} \
	} while(0)

void assert_handler(const char* file, int line);
#endif /* ASSERT_HANDLER_ */


