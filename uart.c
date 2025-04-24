/*
 * uart.c
 *
 *  Created on: May 12, 2024
 *      Author: alejandroregalado
 */


#include <stm32l476xx.h>
#include <uart.h>

/* Initialze USART2 for terminal comm. */
void uart_init(void){

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable gpioa clock

	// using PA2 and PA3
	GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

	// USART2 is alternate function 7
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
	GPIOA->AFR[0] |= 7 << GPIO_AFRL_AFSEL2_Pos | 7 << GPIO_AFRL_AFSEL3_Pos;

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2_Msk | GPIO_OTYPER_OT3_Msk);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk);
	GPIOA->PUPDR  &= ~(GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk);

	// USART initialization
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	// enable transmit and receive, receive interrupt
	USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;

	// Set Baud Rate
	USART2->BRR = BAUD_RATE;

	// enable USART
	USART2->CR1 |= USART_CR1_UE;

}

/* Print a single character to terminal */
void uart_print_char(char character){
	// wait for empty transmit buffer
   while (!(USART2->ISR & USART_ISR_TXE));
   USART2->TDR = character;
}

/* Print an entire string to terminal */
void uart_print_message(char *message){
	// print out characters until null
   while(message[0] != '\0'){
		 uart_print_char(message[0]);
		 message += 1;
   }
}

/* Use escape code to modify print style */
void uart_print_esc(char *message){
   // first print out ESC then the code
   uart_print_char((char)0x1B);
   uart_print_message(message);
}



