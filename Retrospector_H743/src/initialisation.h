#pragma once

#include "stm32h7xx.h"
#include "mpu_armv7.h"		// Memory protection unit for selectively disabling cache for DMA transfers
#include <algorithm>
#include <cstdlib>

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 10
#define SAMPLE_BUFFER_LENGTH 65536
#define CPUCLOCK 200
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];

// Define ADC array positions of various controls
enum ADC_Controls { ADC_Audio_L = 0, ADC_Audio_R = 1, ADC_Mix = 2, ADC_Delay_Pot_L = 3, ADC_Delay_Pot_R = 4, ADC_Delay_CV_L = 5, ADC_Delay_CV_R = 6,
	ADC_Feedback_Pot_L = 7, ADC_Feedback_CV_L = 8, ADC_Tone = 9};

void SystemClock_Config();
void InitCache();
void InitSysTick();
//void InitUART();
void uartSendChar(char c);
void uartSendString(const char* s);
void InitADC();
void InitDAC();
void InitI2S();
void InitTempoClock();
void InitLEDs();
