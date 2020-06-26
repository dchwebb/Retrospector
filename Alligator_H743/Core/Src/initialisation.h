#pragma once

#include "stm32h7xx.h"

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 10
//extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH * 4];

void SystemClock_Config(void);
/*
void InitMCO2();

void InitADC(void);
void InitIO(void);
void InitSysTick();
void InitCoverageTimer();

void InitFPGAProg();
void InitSPI();
void InitSPITimer();
void sendSPIData(uint16_t data);
void clearSPI();

void InitI2S();
void sendI2SData(uint32_t data);
*/
