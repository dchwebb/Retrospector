#pragma once

#include "initialisation.h"
#include <cmath>
#include <complex>
#include <array>


#define MAX_POLES 8		// For declaring IIR arrays
#define MAX_SECTIONS (MAX_POLES + 1) / 2

#define M_PI           3.14159265358979323846


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
	friend class CDCHandler;									// Allow the serial handler access to private data for debug printing
	friend class Config;										// Allow access to config to store values

public:
	enum PassType {FilterOff, LowPass, HighPass};

	// constructors
	IIRFilter(uint8_t poles, PassType pass) : numPoles{poles}, passType{pass}, iirProto(IIRPrototype(poles)) {};
	IIRFilter() {};

	void UpdateProto(uint8_t section, iirdouble_t damping);		// Allows custom damping values to be used in place of Butterworth defaults
	void UpdateProto(uint8_t poles);							// Edit number of poles
	void DefaultProto();										// Resets prototype to Butterworth defaults
	void CalcCoeff(iirdouble_t omega);
	void CalcCustomLowPass(iirdouble_t omega);
	iirdouble_t FilterSample(iirdouble_t sample, IIRRegisters& registers);
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

};


// Filter with fixed cut off (eg control smoothing)
class FixedFilter {
private:
	IIRFilter filter;
	IIRRegisters iirReg;
public:
	FixedFilter(uint8_t poles, IIRFilter::PassType pass, iirdouble_t frequency) : filter{poles, pass} {
		filter.CalcCoeff(frequency);
	}
	iirdouble_t FilterSample(iirdouble_t sample);
};





