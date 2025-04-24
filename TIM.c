/*
 * TIM.c
 *
 *  Created on: May 28, 2024
 *      Author: alejandroregalado
 */

#include <TIM.h>

/* Configure Timer2 */
void TIM2_Config(uint32_t ARR) {

	  // TIM2 Configuration
	  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);	// Set clock
	  TIM2->ARR = ARR;							// Fill ARR after 'ARR' clock cycles

	  // TIM2 Settings
	  TIM2->CR1 = 0;							// Default timer settings
	  TIM2->CR2 = 0;							// Default timer settings
	  TIM2->CR1 &= ~(TIM_CR1_DIR); 				// Set up counter direction (0: up-counting, 1: down-counting)
	  TIM2->CR1 |= (TIM_CR1_URS); 				// Only overflow/underflow generates an update interrupt
	  TIM2->CR1 &= ~(TIM_CR1_UDIS); 			// Enable update events

	  // Enabling events
	  TIM2->DIER |= TIM_DIER_UIE; 				// Enable update interrupt
	  NVIC->ISER[0] |= (1<<TIM2_IRQn);			// Enable interrupt in NVIC
	  TIM2->CR1 |= TIM_CR1_CEN; 				// Start the timer
}

/* Configure Timer2 */
void TIM5_Config(void) {

	// TIM5 Configuration
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;			// Set clock
	TIM5->PSC = 0;									// Prescaler not used
	TIM5->ARR = 0xFFFFFFFF;							// Fill ARR after 4 clock cycles
	TIM5->CCR1 = 100000;							// Trigger event 1 clock cycle before ARR filled

	// TIM5 Settings
	TIM5->CR1 = 0;									// Default timer settings
	TIM5->CR2 = 0;									// Default timer settings
	TIM5->CR1 &= ~(TIM_CR1_DIR); 					// Set up counter direction (0: up-counting, 1: down-counting)
	TIM5->CR1 |= (TIM_CR1_URS); 					// Only overflow/underflow generates an update interrupt
	TIM5->CR1 &= ~(TIM_CR1_UDIS); 					// Enable update events

	// Enabling events
	TIM5->DIER |= TIM_DIER_CC1IE;					// Enable CCR1
	NVIC_EnableIRQ(TIM5_IRQn);						// Enable interrupt in NVIC
	TIM5->CR1 |= TIM_CR1_CEN; 						// Start the timer
}
