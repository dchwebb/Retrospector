#pragma once

#include "initialisation.h"
#include "filter.h"
#include <map>


extern uint16_t adcZeroOffset[2];
extern uint32_t lastClock;

enum delay_mode {modeLong = 0, modeShort = 1, modeReverse = 2};

struct digitalDelay {
public:
	int32_t readPos[2];
	int32_t writePos[2] = {1, 1};
	uint16_t delayCrossfade[2];
	int32_t ledCounter[2];				// Counter to control timing of LED delay rate indicators
	int32_t oldReadPos[2];
	int32_t currentDelay[2];
	volatile int32_t calcDelay[2];	// volatile required to prevent optimiser using incorrect ADC channel
	volatile int16_t delayPotVal[2];
	volatile float delayMult[2];
	int16_t filterBuffPos[2];
	delay_mode delayMode;

	const int16_t delayHysteresis = 40;
	const int16_t crossfade = 6000;
	const int16_t tempoHysteresis = 100;
	//const std::array<std::pair<float, int16_t>, 2> tempoMap = {{0.5f, 20000}, {1.0f, 40000}};
	const std::map<uint16_t, float> tempoMap = {{13000, 1}, {26000, 2}, {39000, 4}, {52000, 8}, {65535, 16}};

	int32_t calcSample(channel LOrR);
	void init();
	delay_mode mode();
};
