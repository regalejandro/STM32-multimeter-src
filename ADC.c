/*
 * ADC.c
 *
 *  Created on: May 15, 2024
 *      Author: alejandroregalado
 */


#include <ADC.h>

#include <stm32l476xx.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <uart.h>

/* Configure ADC1 */
void ADC_Config(void) {

	// Initialize clock for ADC
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos);

	// Turn on ADC
	ADC1->CR &= ~(ADC_CR_DEEPPWD); 		// Disable deep power down
	ADC1->CR |= (ADC_CR_ADVREGEN); 		// ADC Voltage Regulator
	for (int i = 0; i < 1000; i++); 	// Delay for power up time


	// Calibrate
	ADC1->CR &= ~(ADC_CR_ADEN |			// Disable ADC
				ADC_CR_ADCALDIF);		// Configure calibration in single-ended in mode
	ADC1->CR |= (ADC_CR_ADCAL);			// Launch single-ended calibration
	while(ADC1->CR & ADC_CR_ADCAL);		// Wait for calibration end
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);	// Channel 5 in single-ended mode


	// Enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY); 		// Clear flag by writing '1'
	ADC1->CR |= (ADC_CR_ADEN);			// Enable
	while(!(ADC1->ISR & ADC_ISR_ADRDY));// Wait for ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY); 		// Clear flag by writing '1'


	// Configure Sampling
	ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);	// Channel 5 for 1st conv.
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP5);	// Sample time of 2.5 clocks
	ADC1->CFGR  &= ~( ADC_CFGR_CONT  |	// Single Conversion Mode
	                  ADC_CFGR_EXTEN |	// Hardware trigger disabled
	                  ADC_CFGR_RES   );	// 12-bit resolution

	// Configure + Enable Interrupts
	ADC1->IER |= ADC_IER_EOCIE;			// End of conversion interrupt en.
	NVIC->ISER[0] |= (1 << (ADC1_2_IRQn & 0x1F));	// Enable ADC ISR
	__enable_irq();						// Global interrupt en.

	// GPIO PA0
	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOAEN); // Initialize clock
	GPIOA->MODER  |= (GPIO_MODER_MODE0_Msk);// Analog Mode '11'
	GPIOA->ASCR  |= (GPIO_ASCR_ASC0);		// Connect analog switch to ADC

	// Start ADC conversion
	ADC1->CR |= ADC_CR_ADSTART;

}

/* Convert digital value to analog value in microvolts */
uint32_t ADC_digital_conv(uint16_t digital) {
	uint32_t voltage = (digital * 821);

	if (voltage < 6550)
		voltage = 0;
	else
		voltage -= 6550;

	if (voltage > 3300000)
		voltage = 3300000;

	return voltage;	// In microVolts
}

/* Print digital number to terminal as a voltage with millivolt precision */
void print_Voltage(uint16_t diginum) {
	char str[16];			// String buffer
	uint32_t voltage;		// Converted voltage
	int zeroes;				// Number of leading zeroes to print


	uart_print_esc("[16c");

	voltage = ADC_digital_conv(diginum);
	sprintf(str, "%lu", voltage);


	// print leading zeroes
	zeroes = 7 - strlen(str);
	if (zeroes > 0) {
		uart_print_char('0');
		uart_print_char('.');
		for (int j = 0; (j < zeroes - 1) && (j < 3); j++) {
			uart_print_char('0');
		}
	}
	// print remaining characters to mV precision
	if (zeroes > 0 && zeroes < 4) {	// If voltage is > 0000999uV
		for (int j = 0; zeroes < 4; j++) {
			uart_print_char(str[j]);
			zeroes++;
		}
	}

	if (zeroes == 0) {
		uart_print_char(str[0]);
		uart_print_char('.');
		while (zeroes < 3) {
			zeroes++;
			uart_print_char(str[zeroes]);
		}
	}

	uart_print_message(" V");
	uart_print_esc("[1E");

}



