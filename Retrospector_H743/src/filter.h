#pragma once

#include "initialisation.h"
#include <cmath>

#define FIRTAPS 81
#define M_PI           3.14159265358979323846

extern bool activateFilter;
extern uint8_t activeFilter;		// choose which set of coefficients to use
extern float firCoeff[2][FIRTAPS];
extern float currentCutoff;
extern int16_t filterBuffer[2][FIRTAPS];		// Ring buffer containing most recent playback samples for quicker filtering from SRAM

void InitFilter(uint16_t tone);
float Sinc(float x);

enum FilterType {LowPass, HighPass};
extern FilterType filterType;
