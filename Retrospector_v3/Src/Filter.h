#pragma once

/*
 * Much of the filter code gratefully taken from Iowa Hills Software
 * http://www.iowahills.com/
 */

#include "initialisation.h"
#include <cmath>
#include <complex>
#include <array>
#include "LEDHandler.h"
#include "configManager.h"
#include "IIRFilter.h"

#define MAX_POLES 8		// For declaring IIR arrays
#define MAX_SECTIONS (MAX_POLES + 1) / 2
#define MAX_FIR_TAPS 93

#define M_PI           3.14159265358979323846

// For debugging
extern bool calculatingFilter;
extern LEDHandler led;

enum FilterControl {LP, HP, Both};
enum PassType {FilterOff, LowPass, HighPass};
enum FilterType {FIR, IIR};

typedef std::complex<double> complex_t;


struct Filter {
	friend class CDCHandler;				// Allow the serial handler access to private data for debug printing
	friend class Config;					// Allow access to config to store values
public:
	void Init();
	void Update(bool reset = false);
	float CalcFilter(float sample, channel c);
	void CustomiseIIR(uint8_t section, iirdouble_t damping);
	void CustomiseIIR(uint8_t sectionCount);
	void DefaultIIR();						// Reset default IIR coefficients for all IIR filters
	static void UpdateConfig();

	struct {
		float potCentre = 29000;
		uint8_t firTaps = 93;	// value must be divisble by four + 1 (eg 93 = 4*23 + 1) or will cause phase reversal when switching between LP and HP
		uint8_t iirNumPoles = 0;
		bool filter_custom_damping = false;
		float filter_damping[4] = {0.0, 0.0, 0.0, 0.0};
	} cfg;

	ConfigSaver configSaver = {
		.settingsAddress = &cfg,
		.settingsSize = sizeof(cfg),
		.validateSettings = UpdateConfig
	};

private:

	bool activateFilter = true;				// For debug
	PassType passType;
	FilterControl filterControl = Both;		// Tone control sweeps from LP to HP ('Both') or 'LP' or 'HP'
	FilterControl newFilterControl = filterControl;
	bool activeFilter = 0;					// choose which set of coefficients to use (so coefficients can be calculated without interfering with current filtering)
	float currentCutoff;
	FilterType filterType = FIR;
	FilterType newFilterType = filterType;	// Settings to enable filters to be altered without affecting ongoing calculations



	// FIR Settings
	float firCoeff[2][MAX_FIR_TAPS];
	float winCoeff[MAX_FIR_TAPS];
	float filterBuffer[2][256];				// Ring buffer containing most recent playback samples for quicker filtering from SRAM (NB using 256 to speed up ring buffer navigation)
	uint8_t filterBuffPos[2];

	// IIR settings
	bool customDamping = false;				// Set to true if using custom damping coefficients (otherwise will default to Butterworth)
	const uint8_t defaultPoles = 4;
	IIRFilter iirLPFilter[2] = {IIRFilter(defaultPoles, IIRFilter::LowPass), IIRFilter(defaultPoles, IIRFilter::LowPass)};			// Two filters for active and inactive
	IIRFilter iirHPFilter[2] = {IIRFilter(defaultPoles, IIRFilter::HighPass), IIRFilter(defaultPoles, IIRFilter::HighPass)};
	IIRRegisters iirLPReg[2];				// Two channels (left and right)
	IIRRegisters iirHPReg[2];				// Store separate shift registers for high and low pass to allow smooth transition

	float dampedADC, previousADC;			// ADC readings governing damped cut off level (and previous for hysteresis)
	FixedFilter filterADC = FixedFilter(2, IIRFilter::LowPass, 0.002f);
	static constexpr uint16_t hysteresis = 30;

	uint16_t softSwitchTime = 0;			// Amount of time remaining for soft switch cross-fading
	const uint16_t softSwitchDefault = 500;	// Total amount of time for soft switch cross-fading

	iirdouble_t CalcIIRFilter(iirdouble_t sample, channel c);
	float CalcFIRFilter(float sample, channel c);
	void InitFIRFilter(float tone);
	void InitIIRFilter(iirdouble_t tone);
	float Sinc(float x);
	void FIRFilterWindow();
	float Bessel(float x);
	void SwitchFilter();
};


extern Filter filter;


