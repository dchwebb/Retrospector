#pragma once

#include "stm32h7xx.h"
#include "mpu_armv7.h"		// Memory protection unit for selectively disabling cache for DMA transfers
#include <algorithm>
#include <cstdlib>

extern volatile uint32_t SysTickVal;

#define ADC1_BUFFER_LENGTH 4
#define ADC2_BUFFER_LENGTH 7
#define SAMPLE_BUFFER_LENGTH 1048576		// Currently 2^20 (4MB of 16MB)
#define SAMPLE_RATE 48000
#define SYSTICK 1000						// Set in uS so 1000uS = 1ms
#define ADC_OFFSET_DEFAULT 33800
#define CPUCLOCK 400


extern volatile uint16_t ADC_array[ADC1_BUFFER_LENGTH + ADC2_BUFFER_LENGTH];
extern int32_t adcZeroOffset[2];

// Define ADC array positions of various controls
enum ADC_Controls {
	ADC_Mix          = 4,
	ADC_Delay_Pot_L  = 5,
	ADC_Delay_Pot_R  = 2,
	ADC_Delay_CV_L   = 6,
	ADC_Delay_CV_R   = 3,
	ADC_Feedback_Pot = 7,
	ADC_Feedback_CV  = 8,
	ADC_Filter_CV    = 9,
	ADC_Filter_Pot   = 10
};
enum channel {left = 0, right = 1};


void SystemClock_Config();
void InitCache();
void InitSysTick();
void uartSendChar(char c);
void uartSendString(const char* s);
void InitADC();
void InitADC1();
void TriggerADC1();
void InitADC2();
void InitDAC();
void InitI2S();
void suspendI2S();
void resumeI2S();
void InitTempoClock();
void InitBootloaderTimer();
void DisableBootloaderTimer();
void InitIO();
void InitDebugTimer();
void InitI2C();
void InitLEDSPI();
void Init_WS2812_SPI();
void CopyToITCMRAM();
