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

// These coeff form H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
struct SPlaneCoeff {
	iirdouble_t N2[MAX_POLES];
	iirdouble_t N1[MAX_POLES];
	iirdouble_t N0[MAX_POLES];
	iirdouble_t D2[MAX_POLES];
	iirdouble_t D1[MAX_POLES];
	iirdouble_t D0[MAX_POLES];
};

struct IIRRegisters {
	iirdouble_t X1[MAX_SECTIONS];
	iirdouble_t X2[MAX_SECTIONS];
	iirdouble_t Y1[MAX_SECTIONS];
	iirdouble_t Y2[MAX_SECTIONS];
};

class IIRPrototype {
public:
	IIRPrototype(uint8_t poles) {
		numPoles = poles;
		CalcLowPassProtoCoeff();
	}
	IIRPrototype() {};

	SPlaneCoeff Coeff;

private:
	uint8_t numPoles = 0;
	uint8_t numSections = 0;

	void CalcLowPassProtoCoeff();
	void ButterworthPoly(std::array<std::complex<double>, MAX_POLES> &Roots);
	void GetFilterCoeff(std::array<std::complex<double>, MAX_POLES> &Roots);
};


class IIRFilter {
	friend class SerialHandler;				// Allow the serial handler access to private data for debug printing
private:
	uint8_t numPoles = 1;
	uint8_t numSections = 0;
	PassType passType = LowPass;
	iirdouble_t cutoffFreq = 0.0f;
	IIRPrototype iirProto;
	IIRCoeff iirCoeff;

	iirdouble_t CalcSection(int k, iirdouble_t x, IIRRegisters& registers);

public:
	// constructors
	IIRFilter(uint8_t poles, PassType pass) : numPoles{poles}, passType{pass}, iirProto(IIRPrototype(poles)) {}
	IIRFilter() {};

	void CalcCoeff(iirdouble_t omega);
	iirdouble_t FilterSample(iirdouble_t sample, IIRRegisters& registers);
};


// Filter with fixed cut off (eg chorus filter, control smoothing)
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
public:
	uint16_t potCentre = 29000;		// FIXME - make this configurable in calibration

	void Init();
	void Update(bool reset = false);
	float CalcFilter(float sample, channel c);

private:
	uint8_t firTaps = 93;	// value must be divisble by four + 1 (eg 93 = 4*23 + 1) or will cause phase reversal when switching between LP and HP

	bool activateFilter = true;				// For debug
	FilterType filterType = FIR;
	PassType passType;
	FilterControl filterControl = Both;		// Tone control sweeps from LP to HP ('Both') or 'LP' or 'HP'
	uint8_t activeFilter = 0;				// choose which set of coefficients to use (so coefficients can be calculated without interfering with current filtering)
	float currentCutoff;

	// FIR Settings
	float firCoeff[2][MAX_FIR_TAPS];
	float winCoeff[MAX_FIR_TAPS];
	float filterBuffer[2][256];				// Ring buffer containing most recent playback samples for quicker filtering from SRAM (NB using 256 to speed up ring buffer navigation)
	uint8_t filterBuffPos[2];

	// IIR settings
	const uint8_t polesLP = 4;
	const uint8_t polesHP = 4;
	IIRFilter iirLPFilter[2] = {IIRFilter(polesLP, LowPass), IIRFilter(polesLP, LowPass)};			// Two filters for active and inactive
	IIRFilter iirHPFilter[2] = {IIRFilter(polesHP, HighPass), IIRFilter(polesHP, HighPass)};
	IIRRegisters iirLPReg[2];				// Two channels (left and right)
	IIRRegisters iirHPReg[2];				// Store separate shift registers for high and low pass to allow smooth transition

	float dampedADC, previousADC;		// ADC readings governing damped cut off level (and previous for hysteresis)
	FixedFilter filterADC = FixedFilter(2, LowPass, 0.002f);
	static constexpr uint16_t hysteresis = 30;


	iirdouble_t CalcIIRFilter(iirdouble_t sample, channel c);
	float CalcFIRFilter(float sample, channel c);
	void InitFIRFilter(float tone);
	void InitIIRFilter(iirdouble_t tone);
	float Sinc(float x);
	void FIRFilterWindow();
	float Bessel(float x);
};


extern Filter filter;


