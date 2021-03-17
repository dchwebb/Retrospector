#pragma once

#include "initialisation.h"
//#include <map>
#include "Filter.h"

extern uint16_t adcZeroOffset[2];
extern uint32_t lastClock;
extern uint32_t clockInterval;
extern bool clockValid;
extern int32_t samples[SAMPLE_BUFFER_LENGTH];
extern uint16_t chorusSamples[2][65536];

union StereoSample {
	int32_t bothSamples;
	int16_t sample[2];
};

#define CHORUS_MIN 80.0f * 2.0f
#define CHORUS_MAX 260.0f * 2.0f
#define CHORUS_INC (CHORUS_MAX - CHORUS_MIN) / 48000

enum delay_mode {modeLong = 0, modeShort = 1, modeReverse = 2};

struct DigitalDelay {
public:
	int32_t readPos[2];
	int32_t writePos[2] = {1, 1};
	uint16_t delayCrossfade[2];
	int32_t oldReadPos[2];
	int32_t currentDelay[2];
	volatile int32_t calcDelay[2];			// volatile required to prevent optimiser using incorrect ADC channel
	volatile int16_t delayPotVal[2];
	volatile float delayMult[2];
	delay_mode delayMode;

	uint32_t delayCounter;					// Counter used to calculate clock times
	uint32_t lastClock;
	uint32_t clockInterval;
	int16_t clockError;						// Debug offset on clock calculations
	bool clockValid = false;
	bool clockHigh = false;;

	uint32_t ledOffTime[2];
	int32_t ledCounter[2];					// Counter to control timing of LED delay rate indicators
	int16_t ledFraction[2];					// Counter to handle tempo subdivision display locked to incoming clock

	bool pingPong = false;
	bool chorusMode = false;
	float chorusLFO[2] = {CHORUS_MIN, CHORUS_MAX};
	float chorusAdd[2] = {CHORUS_INC, -1 * CHORUS_INC};		// Calculated to give a variable delay between 1.7mS and 3.87mS with a 2 second LFO (Mode I = 0.5Hz, Mode II = 0.8Hz)
	uint16_t chorusWrite = 0;
	FixedFilter chorusFilter[2] = {FixedFilter(4, LowPass, 0.1f), FixedFilter(4, LowPass, 0.1f)};

	const int16_t delayHysteresis = 40;
	const int16_t crossfade = 6000;
	const int16_t tempoHysteresis = 100;
	const std::array<float, 6> tempoMult = {0.5, 1, 2, 4, 8, 16};
	const int16_t threshold = 20000;		// compression threshold
	const int16_t ratio = 10000;			// Increase for less compression: Level at which the amount over the threshold is reduced by 50%. ie at 30k input (threshold + ratio) output will be 25k (threshold + 50% of ratio)


	void CalcSample(channel LOrR);
	void Init();
	void UpdateLED(channel c);
	void ReverseLED(channel c, int32_t remainingDelay);
	void LedOn(channel c);
	void LedOff(channel c);
	delay_mode Mode();
	void ChorusMode(bool on);
	int32_t OutputMix(float drySample, float wetSample);

};
