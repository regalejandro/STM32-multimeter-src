# Digital Multimeter Project Source Code

![DigitalMultimeterOutput](DigitalMultimeterOutput.jpg)

### Behavior Description
In this project, I am simulating the behavior of a digital multimeter, utilizing the STM32L476 MCU as well as a function generator to produce the waves measured by the DMM. The multimeter is capable of measuring a frequency and consequently finding DC average, true AC RMS, and Peak to Peak AC voltage values. By only connecting a function generator to PA0 of the MCU (and of course connecting to ground), the USART port can be accessed on a terminal as seen below to view the live measurement updates. PA0 interacts with the built-in ADC of the microcontroller to measure and convert voltages that are then processed to calculate the displayed frequency and voltages. Measurement of voltages functions with sin, square, and sawtooth waves among other sinusoids. All the user has to do after connecting to the USART port, is set the function to measure and observe.

### Software Architecture

![DigitalMultimeterFSM](DigitalMultimeterFSM.jpg)

The Digital Multimeter begins by finding the frequency utilizing an FFT. This is achieved by storing many values in an unknown period of a signal to a buffer and then processing using this buffer. After the Fourier Transform program–sourced from the CMSIS DSP library–outputs a frequency, the program moves to the AC_FINDER state. The frequency is used along with a timer to find a value that CCR1 can be incremented by each time it is reached to create a frequency that is sufficient for reading 200 values of one period of the signal. This is done with the formula CCR1 = CCR1 + 40MHz/(Frequency * 200). 200 values are then buffered at this simulated frequency created by CCR1 and the FSM goes to the DC_FINDER state. Here, CCR1 = CCR1 + 40MHz/(200/100 * 1000) where 100 is the 100ms length that the 200 samples are being collected for. The values are also buffered. In the PROCESS_NUMBERS state, the AC and DC buffers are utilized to calculate VRMS, VPP, and VDC. This is done by iterating through the 200 values collected and taking the necessary averages. The found values are then printed in the PRINT state which utilizes a USART port to transmit the information to the terminal.

### Additional Notes 
- All files were written by me excluding the additional file "arm_math.h", a library used for DFT calculation
- Not all necessary componenets to run the program exist within this repository, it is purely for holding my source code
