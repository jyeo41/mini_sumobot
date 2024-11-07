#ifndef UART_H_
#define UART_H_
#include <stdint.h>

void uart2_initialize(void);
void uart2_set_baudrate(uint32_t peripheral_clock, uint32_t baudrate);
void uart2_send_char(uint8_t c);
void uart_send_string(char* string);

#endif /* UART_H_ */
