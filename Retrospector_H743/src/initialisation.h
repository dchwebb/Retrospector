#pragma once

#include "stm32h7xx.h"
#include "mpu_armv7.h"		// Memory protection unit for selectively disabling cache for DMA transfers
#include <algorithm>
#include <cstdlib>

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 8
#define SAMPLE_BUFFER_LENGTH 1048576		// Currently 2^20 (4MB of 16MB)
#define SAMPLE_RATE 48000
#define SYSTICK 1000						// Set in uS so 1000uS = 1ms

#define CPUCLOCK 400

extern volatile uint16_t ADC_audio[2];
extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH];

// Define ADC array positions of various controls
enum ADC_Controls { ADC_Mix = 0, ADC_Delay_Pot_L = 1, ADC_Delay_Pot_R = 2, ADC_Delay_CV_L = 3, ADC_Delay_CV_R = 4,
	ADC_Feedback_Pot = 5, ADC_Feedback_CV = 6, ADC_Tone = 7};
enum channel {left = 0, right = 1};

//// Available in C++ 17
//template<class T>
//constexpr const T& clamp( const T& v, const T& lo, const T& hi )
//{
//	return (v < lo) ? lo : (hi < v) ? hi : v;
//}

void SystemClock_Config();
void InitCache();
void InitSysTick();
//void InitUART();
void uartSendChar(char c);
void uartSendString(const char* s);
void InitADCAudio();
void InitADCControls();
void InitDAC();
void InitI2S();
void InitTempoClock();
void InitIO();
//void LED(channel c, bool on);
