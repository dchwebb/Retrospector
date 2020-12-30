#pragma once

#include "initialisation.h"
#include "filter.h"

extern uint16_t adcZeroOffset[2];

enum delay_mode {modeLong = 0, modeShort = 1, modeReverse = 2};

struct digitalDelay {
public:
	int32_t readPos[2];
	int32_t writePos[2] = {1, 1};
	uint16_t delayCrossfade[2];
	uint32_t ledCounter[2];				// Counter to control timing of LED delay rate indicators
	int32_t oldReadPos[2];
	int32_t currentDelay[2];
	volatile int32_t dampedDelay[2];	// volatile required to prevent optimiser using incorrect ADC channel
	int16_t filterBuffPos[2];
	delay_mode delayMode;

	const int16_t delayHysteresis = 40;
	const int16_t crossfade = 6000;

	int32_t calcSample(channel LOrR);
	void init();
	delay_mode mode();
};
