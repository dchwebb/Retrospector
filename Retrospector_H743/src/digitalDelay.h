#pragma once

#include "initialisation.h"
#include <map>
#include "Filter.h"


extern uint16_t adcZeroOffset[2];
extern uint32_t lastClock;

enum delay_mode {modeLong = 0, modeShort = 1, modeReverse = 2};

struct digitalDelay {
public:
	int32_t readPos[2];
	int32_t writePos[2] = {1, 1};
	uint16_t delayCrossfade[2];
	int32_t oldReadPos[2];
	int32_t currentDelay[2];
	volatile int32_t calcDelay[2];			// volatile required to prevent optimiser using incorrect ADC channel
	volatile int16_t delayPotVal[2];
	volatile float delayMult[2];
	int16_t filterBuffPos[2];
	delay_mode delayMode;
	uint32_t ledOffTime[2];
	int32_t ledCounter[2];					// Counter to control timing of LED delay rate indicators
	int16_t ledFraction[2];					// Counter to handle tempo subdivision display locked to incoming clock
	bool pingPong = true;

	const int16_t delayHysteresis = 40;
	const int16_t crossfade = 6000;
	const int16_t tempoHysteresis = 100;
	const std::array<float, 6> tempoMult = {0.5, 1, 2, 4, 8, 16};
	const int16_t threshold = 20000;		// compression threshold
	const int16_t ratio = 10000;			// Increase for less compression: Level at which the amount over the threshold is reduced by 50%. ie at 30k input (threshold + ratio) output will be 25k (threshold + 50% of ratio)


	void calcSample(channel LOrR);
	void init();
	void updateLED(channel c);
	void reverseLED(channel c, int32_t remainingDelay);
	void ledOn(channel c);
	void ledOff(channel c);
	delay_mode mode();

};
