#ifndef UART_H_
#define UART_H_
#include <stdint.h>

void uart2_initialize(void);
//void uart2_send_char(uint8_t c);
void uart_send_string(const char* string);
void uart2_interrupt_send_char(uint8_t c);

#endif /* UART_H_ */
