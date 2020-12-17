#pragma once

#include "initialisation.h"
#include "filter.h"

extern int32_t readPos;
extern int32_t oldReadPos;
extern int32_t writePos;
extern int16_t delayChanged;
extern uint16_t delayCrossfade;
extern int32_t currentDelay;
extern int32_t dampedDelay;
extern int32_t ns, s_n, ls, rp, nrp;

extern uint16_t adcZeroOffset;

struct digitalDelay {
public:
//	int16_t samples[SAMPLE_BUFFER_LENGTH];
	int32_t readPos[2];
	//int32_t targetReadPos;
	int32_t writePos[2];
	int16_t delayChanged[2];		// To implement a pause between changing delay times
	uint16_t delayCrossfade[2];
	uint32_t oldReadPos[2];
	int32_t currentDelay[2];
	int32_t dampedDelay[2];

	enum sampleLR {channelL = 0, channelR = 1};

	int32_t lastSample[2];
	bool sampleUp;

	//bool sampleClock = false;
	const int16_t delayHysteresis = 100;
	const int16_t crossFade = 1000;

	int32_t calcSample(digitalDelay::sampleLR LOrR);
};
