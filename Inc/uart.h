#ifndef UART_H_
#define UART_H_
#include <stdint.h>

void uart2_initialize(void);
//void uart2_send_char(uint8_t c);

/* Explicit void _putchar(char c) function name required by mpaland's printf external implementation */
void _putchar(char c);

#endif /* UART_H_ */
