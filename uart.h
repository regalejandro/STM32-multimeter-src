/*
  * uart.h
  *
  *  Created on: May 12, 2024
  *      Author: alejandroregalado
*/

#ifndef INC_UART_H_
#define INC_UART_H_
#include <stdint.h>
#include <stm32l476xx.h>

// baud rate divisor: 40 MHz / 115.2 kHz = 347.22 -> 347
#define BAUD_RATE 347
void uart_init(void);
void uart_print_char(char character);
void uart_print_message(char *message);
void uart_print_esc(char *message);
#endif /* INC_UART_H_ */
