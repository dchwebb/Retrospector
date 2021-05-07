#pragma once

#include "initialisation.h"
#include "SerialHandler.h"
#include "Filter.h"

extern int32_t samples[SAMPLE_BUFFER_LENGTH];

// For data locality reasons both L and R samples are stored as a single 32 value - this union provides easy access
union StereoSample {
	int32_t bothSamples;
	int16_t sample[2];
};

enum delay_mode {modeLong = 0, modeShort = 1, modeReverse = 2};

struct DigitalDelay {
	friend class SerialHandler;				// Allow the serial handler access to private data for debug printing
	friend class Config;					// Allow the config access to private data to save settings
public:
	void CalcSample();						// Called by interrupt handler to generate next sample
	void Init();							// Initialise caches, buffers etc
	void CheckSwitches();

	bool stereoWide = false;				// Feedback from one side of the stereo spectrum to the other
	bool modulatedDelay = false;			// Modulated delay activated
	bool linkLR = true;						// Makes tempo of right delay a multiple of left delay
	channel LR = right;						// Alternates between left and right channel each time sample is calculated

private:
	delay_mode delayMode;					// Long/short/reverse

	int32_t writePos = 1;					// Write position in sample buffer (same for both channels)
	int32_t readPos[2];
	int32_t oldReadPos[2];
	uint16_t delayCrossfade[2];				// Counter that ticks down during a crossfade following delay length change
	int32_t currentDelay[2];				// Used to trigger crossfade from old to new read position
	int32_t calcDelay[2];					// Delay time according to whether clocked and with multipliers applied
	int16_t delayPotVal[2];					// For hysteresis checking
	float delayMult[2] = {1.0f, 1.0f};		// Multipliers for delay in clocked mode
	uint8_t longDelMult = 8;				// Delay length scaling when long or reverse delay selected

	uint32_t delayCounter;					// Counter used to calculate clock times in sample time
	uint32_t lastClock;						// Time last clock signal received in sample time
	uint32_t clockInterval;					// Clock interval in sample time
	bool clockValid = false;
	bool clockHigh = false;

	const int16_t delayHysteresis = 40;
	const int16_t crossfade = 6000;
	const int16_t tempoHysteresis = 100;
	const std::array<float, 6> tempoMult = {0.5, 1, 2, 4, 8, 16};

	int32_t ledCounter[2];					// Counter to control timing of LED delay rate indicators
	int16_t ledFraction[2];					// Counter to handle tempo subdivision display locked to incoming clock
	int8_t ledOnTimer = 0;					// Timer to control sending an LED update

	float modOffsetMax = 180.0f;			// Modulated delay maximum sample offset
	float modOffsetInc = 0.00375f;			// Amount by which offset is shifted each sample
	float modOffsetAdd[2] = {modOffsetInc, -1 * modOffsetInc};		// Calculated to give a variable delay between 1.7mS and 3.87mS with a 2 second LFO (Mode I = 0.5Hz, Mode II = 0.8Hz)
	float modOffset[2] = {modOffsetMax / 2, modOffsetMax / 2};

	enum class gateStatus {open, closed, closing} gateShut[2];
	uint16_t belowThresholdCount[2];		// Count of samples lower than gate threshold
	int32_t overThreshold[2];				// Enabled when signal exceeds gate threshold
	uint16_t gateThreshold = 100;			// Gate threshold level
	uint32_t gateHoldCount = 20000;			// Number of samples beneath threshold before applying gate
	bool gateLED = false;					// Filter LED displays gate open/closing/closed status as green/red/yellow
	bool tanhCompression = true;			// Use a fast tanh algorithm for more natural sounding compression than linear compression
	const int16_t threshold = 20000;		// linear compression threshold
	const int16_t ratio = 10000;			// Increase for less compression: Level at which the amount over the threshold is reduced by 50%. ie at 30k input (threshold + ratio) output will be 25k (threshold + 50% of ratio)


	// Private class functions
	int32_t GateSample();
	void UpdateLED(channel c, bool reverse, int32_t remainingDelay = 0);
	void ReverseLED(channel c, int32_t remainingDelay);
	delay_mode Mode();
	int32_t OutputMix(float wetSample);
	float FastTanh(float x);
	void RunTest(int32_t s);
	int32_t DelayCV(channel c);

	enum class TestMode {loop, saw, none};
	TestMode testMode = TestMode::none;
};
