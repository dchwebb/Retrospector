#pragma once

#include "stm32h7xx.h"
#include <string>
#include <sstream>

extern volatile uint32_t SysTickVal;

#define ADC_BUFFER_LENGTH 10
//extern volatile uint16_t ADC_array[ADC_BUFFER_LENGTH * 4];

void SystemClock_Config();
void InitSysTick();
void InitUART();
void uartSendChar(char c);
void uartSendString(const std::string& s);
void InitADC();

/*
void InitMCO2();
void InitIO();
void InitCoverageTimer();
void InitFPGAProg();
void InitSPI();
void InitSPITimer();
void sendSPIData(uint16_t data);
void clearSPI();
void InitI2S();
void sendI2SData(uint32_t data);
*/
