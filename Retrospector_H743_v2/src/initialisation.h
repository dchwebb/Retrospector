#pragma once

#include "stm32h7xx.h"
#include "mpu_armv7.h"		// Memory protection unit for selectively disabling cache for DMA transfers
#include <algorithm>
#include <cstdlib>

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 9
#define AUDIO_BUFFER_LENGTH 2
#define SAMPLE_BUFFER_LENGTH 1048576		// Currently 2^20 (4MB of 16MB)
#define SAMPLE_RATE 48000
#define SYSTICK 1000						// Set in uS so 1000uS = 1ms

#define CPUCLOCK 400


extern volatile uint16_t ADC_audio[AUDIO_BUFFER_LENGTH];
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];
extern int32_t adcZeroOffset[2];

// Define ADC array positions of various controls
enum ADC_Controls {
	ADC_Mix          = 0,
	ADC_Delay_Pot_L  = 1,
	ADC_Delay_Pot_R  = 2,
	ADC_Delay_CV_L   = 3,
	ADC_Delay_CV_R   = 4,
	ADC_Feedback_Pot = 5,
	ADC_Feedback_CV  = 6,
	ADC_Filter_CV    = 7,
	ADC_Filter_Pot   = 8
};
enum channel {left = 0, right = 1};


void SystemClock_Config();
void InitCache();
void InitSysTick();
void uartSendChar(char c);
void uartSendString(const char* s);
void InitADCAudio();
void InitADCControls();
void InitDAC();
void InitI2S();
void InitTempoClock();
void InitBootloaderTimer();
void InitIO();
void InitDebugTimer();
void InitI2C();
void InitLEDSPI();
void Init_WS2812_SPI();
