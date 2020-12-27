#pragma once

#include "initialisation.h"
#include "filter.h"

extern uint16_t adcZeroOffset;

struct digitalDelay {
public:
	int32_t readPos[2];
	int32_t writePos[2];
	int16_t delayChanged[2];		// To implement a pause between changing delay times
	uint16_t delayCrossfade[2];
	uint32_t oldReadPos[2];
	int32_t currentDelay[2];
	int32_t dampedDelay[2];
	int16_t filterBuffPos[2];

	enum sampleLR {channelL = 0, channelR = 1};

	const int16_t delayHysteresis = 40;
	const int16_t crossFade = 6000;

	int32_t calcSample(digitalDelay::sampleLR LOrR, int16_t adcSample);
};
