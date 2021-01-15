/*
 * Much of the filter code gratefully taken from Iowa Hills Software
 * http://www.iowahills.com/
 */
#pragma once

#include "initialisation.h"
#include <cmath>
#include <complex>
#include <map>

// Debug
#include "USB.h"
extern USB usb;

#define FIRTAPS 101
#define MAX_POLES 8		// For declaring IIR arrays
#define MAX_SECTIONS (MAX_POLES + 1) / 2
#define M_PI           3.14159265358979323846

extern bool activateFilter, activateWindow;
extern float currentCutoff;
extern int16_t filterBuffer[2][FIRTAPS];	// Ring buffer containing most recent playback samples for quicker filtering from SRAM

extern bool calculatingFilter;				// For debugging
extern bool debugSort;

enum FilterControl {LP, HP, Both};
enum PassType {FilterOff, LowPass, HighPass, BandPass};
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

class IIRPrototype
{
public:
	uint8_t numPoles;
	uint8_t numSections;
	SPlaneCoeff Coeff;

	IIRPrototype(uint8_t poles) {
		numPoles = poles;
		CalcLowPassProtoCoeff();
	}
	IIRPrototype() {};
	void CalcLowPassProtoCoeff();
	void ButterworthPoly(std::array<std::complex<double>, MAX_POLES> &Roots);
	void GetFilterCoeff(std::array<std::complex<double>, MAX_POLES> &Roots);
};


struct IIRFilter {
public:
	uint8_t numPoles = 1;
	uint8_t numSections;
	PassType passType;
	iirdouble_t cutoffFreq;
	IIRPrototype iirProto;
	IIRCoeff iirCoeff;

	// constructors
	IIRFilter(uint8_t poles, PassType pass) {
		iirProto = IIRPrototype(poles);
		numPoles = poles;
		passType = pass;
	}
	IIRFilter() {};

	void CalcCoeff(iirdouble_t omega);
	iirdouble_t FilterSample(iirdouble_t sample, IIRRegisters& registers);

private:
	iirdouble_t CalcSection(int k, iirdouble_t x, IIRRegisters& registers);
};


struct Filter {
	uint16_t filterPotCentre = 29000;		// FIXME - make this configurable in calibration
	FilterType filterType = IIR;
	PassType passType;
	FilterControl filterControl = Both;		// Tone control sweeps from LP to HP (or choose just LP or HP)
	uint8_t activeFilter = 0;				// choose which set of coefficients to use (so coefficients can be calculated without interfering with current filtering)

	// FIR Settings
	float firCoeff[2][FIRTAPS];
	float winCoeff[FIRTAPS];

	// IIR settings
	const uint8_t polesLP = 4;
	const uint8_t polesHP = 4;
	IIRFilter iirLPFilter[2] = {IIRFilter(polesLP, LowPass), IIRFilter(polesLP, LowPass)};			// Two filters for active and inactive
	IIRFilter iirHPFilter[2] = {IIRFilter(polesHP, HighPass), IIRFilter(polesHP, HighPass)};
	IIRRegisters iirLPReg[2];				// Two channels (left and right)
	IIRRegisters iirHPReg[2];				// STore separate shift registers for high and low pass to allow smooth transition

	void InitFIRFilter(uint16_t tone);
	void InitIIRFilter(uint16_t tone);
	iirdouble_t CalcIIRFilter(iirdouble_t sample, channel c);
	float Sinc(float x);
	void FIRFilterWindow(float beta);
	float Bessel(float x);
};


extern Filter filter;
