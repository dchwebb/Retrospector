#pragma once

#include "initialisation.h"
#include <cmath>
#include <complex>
#include <map>

// Debug
#include "USB.h"
extern USB usb;

#define FIRTAPS 101
#define M_PI           3.14159265358979323846

extern bool activateFilter, activateWindow;
extern float firCoeff[2][FIRTAPS];
extern float winCoeff[FIRTAPS];
extern float currentCutoff;
extern int16_t filterBuffer[2][FIRTAPS];	// Ring buffer containing most recent playback samples for quicker filtering from SRAM

extern bool calculatingFilter;				// For debugging
extern bool debugSort;

enum FilterControl {LP, HP, Both};
enum PassType {FilterOff, LowPass, HighPass};
enum FilterType {FIR, IIR};
enum FilterPoly {BUTTERWORTH, GAUSSIAN, BESSEL, ADJUSTABLE, CHEBYSHEV,	INVERSE_CHEBY, PAPOULIS, ELLIPTIC, NOT_IIR};

typedef double iirdouble;

//#define MAX_POLE_COUNT 8
#define MAX_POLES 8 				// This MUST be at least 2*MAX_POLE_COUNT because some filter polys are defined in terms of 2 * NumPoles


struct IIRCoeff {
	iirdouble a0[MAX_POLES];
	iirdouble a1[MAX_POLES];
	iirdouble a2[MAX_POLES];
	iirdouble b0[MAX_POLES];
	iirdouble b1[MAX_POLES];
	iirdouble b2[MAX_POLES];
	int NumSections;
};

// These coeff form H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
// NumSections is the number of 1st and 2nd order polynomial factors .
struct SPlaneCoeff {
	iirdouble N2[MAX_POLES];
	iirdouble N1[MAX_POLES];
	iirdouble N0[MAX_POLES];
	iirdouble D2[MAX_POLES];
	iirdouble D1[MAX_POLES];
	iirdouble D0[MAX_POLES];
};

struct IIRRegisters {
	iirdouble RegX1[2][MAX_POLES];
	iirdouble RegX2[2][MAX_POLES];
	iirdouble RegY1[2][MAX_POLES];
	iirdouble RegY2[2][MAX_POLES];
};

class IIRPrototype
{
public:
	/*
	 * 	CalcLowPassProtoCoeff
	 *  	ButterworthPoly
	 *  	GetFilterCoeff
	 */
	int NumPoles;
	int NumSections;
	SPlaneCoeff Coeff;

	IIRPrototype(uint8_t numPoles) {
		NumPoles = numPoles;
		CalcLowPassProtoCoeff();
	}
	IIRPrototype() {};
	void CalcLowPassProtoCoeff();
	void ButterworthPoly(std::array<std::complex<double>, MAX_POLES> &Roots);
	void GetFilterCoeff(std::array<std::complex<double>, MAX_POLES> &Roots);
};


struct IIRFilter
{
	IIRPrototype iirProto;
	IIRCoeff iirCoeff;
	int numPoles = 1;

	IIRFilter(uint8_t poles) {
		iirProto = IIRPrototype(poles);
		numPoles = poles;
	}
	IIRFilter() {};

	void CalcIIRFilterCoeff(iirdouble OmegaC, PassType PassType);
//	iirdouble IIRFilter(iirdouble sample, channel c);
//	iirdouble SectCalc(int k, iirdouble x, channel c);

};

struct Filter
{
	uint16_t filterPotCentre = 29000;		//32768	FIXME - make this configurable

//	struct {
//		FilterPoly ProtoType = BUTTERWORTH;
//		int NumPoles = 1;				// 1 <= NumPoles <= 12, 15, 20 Depending on the filter.
//		iirdouble BW = 0.1;				// 0.0 < BandWidth < 1.0       3 dB bandwidth for bandpass and notch filters
//	} iirSettings;

	FilterType filterType = IIR;
	PassType passType;
	FilterControl filterControl = LP;		// Tone control sweeps from LP to HP (or choose just LP or HP)
	uint8_t activeFilter = 0;				// choose which set of coefficients to use (so coefficients can be calculated without interfering with current filtering)

	const uint8_t polesLP = 8;
	const uint8_t polesHP = 4;
	IIRFilter iirLPFilter[2] = {IIRFilter(polesLP), IIRFilter(polesLP)};			// Two filters for active and inactive
	IIRFilter iirHPFilter[2] = {IIRFilter(polesHP), IIRFilter(polesHP)};

	void InitFIRFilter(uint16_t tone);
	void InitIIRFilter(uint16_t tone);
	iirdouble CalcIIRFilter(iirdouble sample, channel c);
	iirdouble SectCalc(int k, iirdouble x, channel c, IIRFilter& currentFilter);
	float Sinc(float x);
	void FIRFilterWindow(float beta);
	float Bessel(float x);
};



extern Filter filter;
