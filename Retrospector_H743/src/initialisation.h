#pragma once

#include "stm32h7xx.h"
#include <algorithm>
#include <cstdlib>

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 4
#define SAMPLE_BUFFER_LENGTH 65536
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];

// Define ADC array positions of various controls
enum ADC_Controls { ADC_AudioL = 0, ADC_AudioR = 1, ADC_DelayL= 2, ADC_Mix = 3 };

void SystemClock_Config();
void InitSysTick();
//void InitUART();
void uartSendChar(char c);
void uartSendString(const char* s);
void InitADC();
void InitDAC();
void InitI2S();


