#pragma once

#include "initialisation.h"
#include "SerialHandler.h"
#include "Filter.h"

extern int32_t samples[SAMPLE_BUFFER_LENGTH];
extern uint16_t chorusSamples[2][65536];

// For data locality reasons both L and R samples are stored as a single 32 value - this union provides easy access
union StereoSample {
	int32_t bothSamples;
	int16_t sample[2];
};

// Chorus settings to match measured Juno 106 timings
#define CHORUS_MIN 80.0f * 2.0f
#define CHORUS_MAX 260.0f * 2.0f
#define CHORUS_INC (CHORUS_MAX - CHORUS_MIN) / 48000

enum delay_mode {modeLong = 0, modeShort = 1, modeReverse = 2};

struct DigitalDelay {
	friend class SerialHandler;				// Allow the serial handler access to private data for debug printing
public:
	void CalcSample();						// Called by interrupt handler to generate next sample
	void Init();							// Initialise caches, buffers etc
	void ChorusMode(bool on);

	bool stereoWide = false;				// Feedback from one side of the stereo spectrum to the other
	bool chorusMode = false;
	bool linkLR = true;					// Makes tempo of right delay a multiple of left delay

private:
	delay_mode delayMode;					// Long/short/reverse
	channel LR = right;						// Alternates between left and right channel each time sample is calculated

	int32_t writePos = 1;					// Write position in sample buffer (same for both channels)
	int32_t readPos[2];
	int32_t oldReadPos[2];
	uint16_t delayCrossfade[2];				// Counter that ticks down during a crossfade following delay length change
	int32_t currentDelay[2];				// Used to trigger crossfade from old to new read position
	int32_t calcDelay[2];					// Delay time according to whether clocked and with multipliers applied
	int16_t delayPotVal[2];					// For hysteresis checking
	float delayMult[2];						// Multipliers for delay in clocked mode

	uint32_t delayCounter;					// Counter used to calculate clock times in sample time
	uint32_t lastClock;						// Time last clock signal received in sample time
	uint32_t clockInterval;					// Clock interval in sample time
	int16_t clockError;						// Debug offset on clock calculations
	bool clockValid = false;
	bool clockHigh = false;

	int32_t ledCounter[2];					// Counter to control timing of LED delay rate indicators
	int16_t ledFraction[2];					// Counter to handle tempo subdivision display locked to incoming clock
	int8_t ledOnTimer = 0;					// Timer to control sending an LED update

	float chorusLFO[2] = {CHORUS_MIN, CHORUS_MAX};
	float chorusAdd[2] = {CHORUS_INC, -1 * CHORUS_INC};		// Calculated to give a variable delay between 1.7mS and 3.87mS with a 2 second LFO (Mode I = 0.5Hz, Mode II = 0.8Hz)
	uint16_t chorusWrite = 0;
	FixedFilter chorusFilter[2] = {FixedFilter(4, LowPass, 0.1f), FixedFilter(4, LowPass, 0.1f)};		// Need 2 filters as uses IIR filter with feedback

	const int16_t delayHysteresis = 40;
	const int16_t crossfade = 6000;
	const int16_t tempoHysteresis = 100;
	const std::array<float, 6> tempoMult = {0.5, 1, 2, 4, 8, 16};
	const int16_t threshold = 20000;		// compression threshold
	const int16_t ratio = 10000;			// Increase for less compression: Level at which the amount over the threshold is reduced by 50%. ie at 30k input (threshold + ratio) output will be 25k (threshold + 50% of ratio)

	// Private class functions
	void UpdateLED(channel c, bool reverse, int32_t remainingDelay = 0);
	void ReverseLED(channel c, int32_t remainingDelay);
	delay_mode Mode();
	int32_t OutputMix(float drySample, float wetSample);
	void RunTest(int32_t s);

	enum class TestMode {loop, saw, none};
	TestMode testMode = TestMode::none;
};
