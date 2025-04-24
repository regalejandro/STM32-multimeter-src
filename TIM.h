/*
 * TIM.h
 *
 *  Created on: May 28, 2024
 *      Author: alejandroregalado
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_

#include <stdint.h>
#include <stm32l476xx.h>

void TIM2_Config(uint32_t ARR);
void TIM5_Config(void);

#endif /* INC_TIM_H_ */
