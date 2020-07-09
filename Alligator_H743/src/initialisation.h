#pragma once

#include "stm32h7xx.h"
#include <algorithm>
#include <cstdlib>

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 3
#define SAMPLE_BUFFER_LENGTH 65536
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];

void SystemClock_Config();
void InitSysTick();
void InitUART();
void uartSendChar(char c);
void uartSendString(const char* s);
void InitADC();
void InitI2S();


