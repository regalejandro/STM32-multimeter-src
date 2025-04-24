/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#define ARM_MATH_CM4

#include "main.h"
#include "uart.h"
#include "ADC.h"
#include "TIM.h"
#include "arm_math.h"
#include <stdio.h>


/* FFT Frequency Defines */
#define SAMPLE_LENGTH 2048
#define TIM_CYCLES 20000
#define SAMP_PERIOD ((float)TIM_CYCLES / 40000000.0)

/* General Defines */
#define SAMPLE_SIZE 200		// Amount of samples to process AC and DC
#define DC_SAMPLE_TIME 100	// Sample length of DC measurement in ms


/* Globals */
int32_t sample_index = 0;	// Index for FFT, and AC/DC samples
int32_t ADC_Conv_Flag = 0;	// ADC conversion done
uint32_t CCR1 = 100000;		// Initialize CCR1 incrementation variable
char CCR1_str[128];
int32_t current_sample = 0;
uint32_t Finding_Freq = 0; 	// High if FFT currently high for choosing ADC timing

void SystemClock_Config(void);

int main(void)
{

	// Initializations
	HAL_Init();
	SystemClock_Config();
	uart_init();

	uart_print_esc("[2J");
	uart_print_esc("[H");
	uart_print_esc("[0m");

	ADC_Config();
	TIM2_Config(TIM_CYCLES - 1);
	TIM5_Config();



	/* FFT Frequency Initialization */
	// in/out/magnitude buffers,
	q15_t FFT_in_buff[SAMPLE_LENGTH];
	q15_t FFT_out_buff[SAMPLE_LENGTH*2];
	q15_t FFT_mag_buff[SAMPLE_LENGTH];

	arm_status status;

	uint32_t FFT_size = SAMPLE_LENGTH;
	uint32_t iFFT_flag = 0;
	uint32_t doBitReverse = 1;
	uint32_t max_Index = 0;

	float input_frequency = 0; 	// Frequency of the input wave
	int32_t final_frequency = 0;// Frequency of the input wave as integer
	char frequency_str[32]; 	// String buffer of frequency to send over USART
	arm_rfft_instance_q15 inst;

	do {
		status = arm_rfft_init_q15(&inst, FFT_size, iFFT_flag, doBitReverse);
	}	while(status != ARM_MATH_SUCCESS); // Successful initialization



	/* General */
	uint32_t DC_buff[SAMPLE_SIZE];
	uint32_t AC_buff[SAMPLE_SIZE];

	enum State {
	    FREQ_FINDER = 1,
	    AC_FINDER = 2,
	    DC_FINDER = 3,
		PROCESS_NUMBERS = 4,
	    PRINT = 5,
		ELI_PRINT = 6
	};

	enum State step = FREQ_FINDER;


	while (1)
	{
		switch (step) {
			// First Find Frequency
			case FREQ_FINDER:
				Finding_Freq = 1;		// Enable FFT ADC timing
				if (ADC_Conv_Flag) {
					ADC_Conv_Flag = 0;	// Lower global flag

					// Store sample, increment sample index
					if (sample_index < SAMPLE_LENGTH) {
						FFT_in_buff[sample_index++] = (current_sample & ((1 << 12) - 1)) << 3;
					}

					// ADC samples collected, moving to process
					if (sample_index >= SAMPLE_LENGTH) {

						sample_index = 0; // Reset sample index

						arm_rfft_q15(&inst, FFT_in_buff, FFT_out_buff);
						arm_cmplx_mag_q15(FFT_out_buff, FFT_mag_buff, FFT_size);
						FFT_mag_buff[0] = 0;
						arm_max_q15(FFT_mag_buff, FFT_size, NULL, &max_Index);

						// Convert to frequency
						input_frequency = 1.0 * max_Index / FFT_size / SAMP_PERIOD;

						float error_offset = input_frequency < 513 ? 1 : 2;
						final_frequency = (int32_t)(input_frequency + error_offset);

						Finding_Freq = 0;	// Disable FFT ADC timing find
						
						step = AC_FINDER;
					}
				}
				break;

			// Second, find AC value
			case AC_FINDER:

				// Calculate CCR1 increment value for AC value measurements
				CCR1 = 40000000 / (final_frequency * SAMPLE_SIZE);

				sample_index = 0;
				// Measure SAMPLE_SIZE measurements and record to AC_buff
				while (sample_index < SAMPLE_SIZE) {
					if (ADC_Conv_Flag) {
						ADC_Conv_Flag = 0;
						AC_buff[sample_index] = current_sample;
						sample_index++;
					}
				}

				step = DC_FINDER;

				break;

			// Third, find DC value
			case DC_FINDER:
				// Calculate CCR1 increment value for DC value measurements
				CCR1 = round(40000000 / (SAMPLE_SIZE * 1000 / DC_SAMPLE_TIME));

				sample_index = 0;
				// Measure SAMPLE_SIZE measurements and record to DC_buff
				while (sample_index < SAMPLE_SIZE) {
					if (ADC_Conv_Flag) {
						ADC_Conv_Flag = 0;
						DC_buff[sample_index] = current_sample;
						sample_index++;
					}
				}
				step = PROCESS_NUMBERS;

				break;

			// Fourth, process buffers for printing
			case PROCESS_NUMBERS:


				uint32_t Vdc = 0;
				uint32_t Vrms = 0;
				uint32_t Vpp = 0;
				uint32_t Vmax = 0;
				uint32_t Vmin = 4095;
				uint32_t Vdc_Bar = 0;
				uint32_t Vrms_Bar = 0;

				// Iterate through and process buffers
				for (int i = 0; i < SAMPLE_SIZE; i++) {

					// Sum up DC_buff[i] for 0 < i < SAMPLE_SIZE
					Vdc += DC_buff[i];

					// Sum up AC_buff[i]^2 for 0 < i < SAMPLE_SIZE
					Vrms += (AC_buff[i] * AC_buff[i]);

					// Search for min and max values from AC_buff
					if (AC_buff[i] > Vmax)
						Vmax = AC_buff[i];
					if (AC_buff[i] < Vmin)
						Vmin = AC_buff[i];

				}

				// Divide DC value sum for average
				Vdc = round(Vdc / SAMPLE_SIZE);

				// Divide and take square root of AC squared sum for true RMS
				Vrms = round(sqrt(Vrms / SAMPLE_SIZE));

				// Calculate peak to peak value with min and max
				Vpp = Vmax - Vmin;

				// With max of 37 '#' bar characters, find number to print on scale of 0 to 3V (3723)
				/*
				Vdc_Bar = round(Vdc * 37 / 3723);
				Vrms_Bar = round(Vrms * 37 / 3723);
				*/

				Vdc_Bar = round(Vdc * 78 / 4095);
				Vrms_Bar = round(Vrms * 78 / 4095);

				step = ELI_PRINT;
				break;

			// Fifth, format printing to terminal
			case PRINT:

				// Reset and print header
				uart_print_esc("[2J");
				uart_print_esc("[0m");
				uart_print_esc("[H");
				uart_print_esc("[5m");
				uart_print_message("Digital Multimeter");
				uart_print_esc("[1E");
				uart_print_esc("[0m");

				// Frequency
				sprintf(frequency_str, "%ld", final_frequency);
				uart_print_esc("[4B");
				uart_print_esc("[15C");
				uart_print_message("Frequency: ");
				uart_print_message(frequency_str);// Send input frequency over USART
				uart_print_message(" Hz");
				uart_print_esc("[1E");

				// Vdc
				uart_print_esc("[15C");
				uart_print_message("DC Voltage: ");
				print_Voltage(Vdc);

				// Vrms
				uart_print_esc("[15C");
				uart_print_message("RMS AC Voltage: ");
				print_Voltage(Vrms);

				// Vpp
				uart_print_esc("[15C");
				uart_print_message("Peak-Peak AC Voltage: ");
				print_Voltage(Vpp);

				// Bar graphs
				uart_print_esc("[22m");
				uart_print_esc("[32m");
				uart_print_esc("[4B");
				uart_print_esc("[15C");
				for (int i = 0; i < Vdc_Bar && i < 37; i++) {
					uart_print_char('#');
				}
				uart_print_esc("[1E");
				uart_print_esc("[15C");
				uart_print_message("|-----|-----|-----|-----|-----|-----| DC Voltage");
				uart_print_esc("[1E");
				uart_print_esc("[15C");
				uart_print_message("0    0.5   1.0   1.5   2.0   2.5   3.0");
				uart_print_esc("[1E");

				uart_print_esc("[34m");
				uart_print_esc("[2B");
				uart_print_esc("[15C");
				for (int i = 0; i < Vrms_Bar && i < 37; i++) {
					uart_print_char('#');
				}
				uart_print_esc("[1E");
				uart_print_esc("[15C");
				uart_print_message("|-----|-----|-----|-----|-----|-----| AC RMS Voltage");
				uart_print_esc("[1E");
				uart_print_esc("[15C");
				uart_print_message("");
				uart_print_esc("[1E");


				step = FREQ_FINDER;
				break;
	}
}

// ADC conversion status
void ADC1_2_IRQHandler(void) {
	if (ADC1->ISR & ADC_ISR_EOC) {
		current_sample = ADC1->DR;		// Read from DR clears EOC flag
		ADC_Conv_Flag = 1;					// Raise global status flag
	}
}

// Sample rate when finding frequency
void TIM2_IRQHandler(void) {
	TIM2->SR &= ~(TIM_SR_UIF);			// Clear update interrupt flag
	if (Finding_Freq)
		ADC1->CR |= (ADC_CR_ADSTART);	// Begin next conversion
}

// Generic sample rate (CCR1 = 100000)
void TIM5_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_CC1IF) {
		TIM5->CCR1 += CCR1;
		TIM5->SR &= ~(TIM_SR_CC1IF);		// Clear Capture/Compare flag
		if (Finding_Freq == 0) {
			ADC1->CR |= (ADC_CR_ADSTART);	// Begin next conversion
		}
	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
