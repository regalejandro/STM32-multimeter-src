/*
 * ADC.h
 *
 *  Created on: May 15, 2024
 *      Author: alejandroregalado
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_
#include <stdint.h>
#include <stm32l476xx.h>

extern uint16_t Digital_Val;

void ADC_Config(void);
uint32_t ADC_digital_conv(uint16_t digital);
void print_Voltage(uint16_t diginum);


#endif /* INC_ADC_H_ */
