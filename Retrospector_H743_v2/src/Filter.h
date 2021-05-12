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
enum IIRType {Butterworth, Custom};


typedef double iirdouble_t;			// to allow easy testing with floats or doubles
typedef std::complex<double> complex_t;



struct IIRCoeff {
	iirdouble_t a0[MAX_SECTIONS];
	iirdouble_t a1[MAX_SECTIONS];
	iirdouble_t a2[MAX_SECTIONS];
	iirdouble_t b0[MAX_SECTIONS];
	iirdouble_t b1[MAX_SECTIONS];
	iirdouble_t b2[MAX_SECTIONS];
};

// These coeff form H(s) = 1 / (D2*s^2 + D1*s + D0)
struct SPlaneCoeff {
	iirdouble_t D2[MAX_POLES];
	iirdouble_t D1[MAX_POLES];
	iirdouble_t D0[MAX_POLES];
};

struct IIRRegisters {
	iirdouble_t X1[MAX_SECTIONS];
	iirdouble_t X2[MAX_SECTIONS];
	iirdouble_t Y1[MAX_SECTIONS];
	iirdouble_t Y2[MAX_SECTIONS];

	IIRRegisters() {
		for (uint8_t i = 0; i < MAX_SECTIONS; ++i) {
			X1[i] = 0.0; X2[i] = 0.0; Y1[i] = 0.0; Y2[i] = 0.0;
		}
	}
	void Init() {
		for (uint8_t i = 0; i < MAX_SECTIONS; ++i) {
			X1[i] = 0.0; X2[i] = 0.0; Y1[i] = 0.0; Y2[i] = 0.0;
		}
	}
};

class IIRPrototype {
public:
	IIRPrototype(uint8_t poles) {
		numPoles = poles;
		DefaultProtoCoeff();
	}
	IIRPrototype() {};

	SPlaneCoeff Coeff;
	uint8_t numPoles = 0;

	void DefaultProtoCoeff();
private:
	void ButterworthPoly(std::array<std::complex<double>, MAX_POLES> &Roots);
	void GetFilterCoeff(std::array<std::complex<double>, MAX_POLES> &Roots);
};


class IIRFilter {
	friend class SerialHandler;									// Allow the serial handler access to private data for debug printing
	friend class Config;										// Allow access to config to store values
private:
	uint8_t numPoles = 1;
	uint8_t numSections = 0;
	PassType passType = LowPass;
	iirdouble_t cutoffFreq = 0.0f;
	bool customDamping = false;									// Set to true if using custom damping coefficients (otherwise will default to Butterworth)
	iirdouble_t damping[MAX_SECTIONS] = {0.923879, 0.382684};	// Damping factor for custom IIR filter - default to Butterworth values
	IIRPrototype iirProto;										// Standard Butterworth is default
	IIRCoeff iirCoeff;

	iirdouble_t CalcSection(int k, iirdouble_t x, IIRRegisters& registers);

public:
	// constructors
	IIRFilter(uint8_t poles, PassType pass) : numPoles{poles}, passType{pass}, iirProto(IIRPrototype(poles)) {};
	IIRFilter() {};

	void UpdateProto(uint8_t section, iirdouble_t damping);		// Allows custom damping values to be used in place of Butterworth defaults
	void UpdateProto(uint8_t poles);							// Edit number of poles
	void DefaultProto();										// Resets prototype to Butterworth defaults
	void CalcCoeff(iirdouble_t omega);
	void CalcCustomLowPass(iirdouble_t omega);
	iirdouble_t FilterSample(iirdouble_t sample, IIRRegisters& registers);
};


// Filter with fixed cut off (eg control smoothing)
class FixedFilter {
private:
	IIRFilter filter;
	IIRRegisters iirReg;
public:
	FixedFilter(uint8_t poles, PassType pass, iirdouble_t frequency) : filter{poles, pass} {
		filter.CalcCoeff(frequency);
	}
	iirdouble_t FilterSample(iirdouble_t sample);
};




struct Filter {
	friend class SerialHandler;				// Allow the serial handler access to private data for debug printing
	friend class Config;					// Allow access to config to store values
public:
//	enum class FilterSwitching {Changed, Switch, None} filterSwitching;


	void Init();
	void Update(bool reset = false);
	float CalcFilter(float sample, channel c);
	void CustomiseIIR(uint8_t section, iirdouble_t damping);
	void CustomiseIIR(uint8_t sectionCount);
	void DefaultIIR();						// Reset default IIR coefficients for all IIR filters
private:
	uint8_t firTaps = 93;	// value must be divisble by four + 1 (eg 93 = 4*23 + 1) or will cause phase reversal when switching between LP and HP
	float potCentre = 29000;				// Configurable in calibration

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
	IIRFilter iirLPFilter[2] = {IIRFilter(defaultPoles, LowPass), IIRFilter(defaultPoles, LowPass)};			// Two filters for active and inactive
	IIRFilter iirHPFilter[2] = {IIRFilter(defaultPoles, HighPass), IIRFilter(defaultPoles, HighPass)};
	IIRRegisters iirLPReg[2];				// Two channels (left and right)
	IIRRegisters iirHPReg[2];				// Store separate shift registers for high and low pass to allow smooth transition

	float dampedADC, previousADC;			// ADC readings governing damped cut off level (and previous for hysteresis)
	FixedFilter filterADC = FixedFilter(2, LowPass, 0.002f);
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


