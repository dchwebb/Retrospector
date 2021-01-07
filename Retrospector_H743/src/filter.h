#pragma once

#include "initialisation.h"
#include <cmath>
#include "CPLXD.hpp"
#include <math.h>

// Debug
#include "USB.h"
extern USB usb;

#define FIRTAPS 101
#define M_PI           3.14159265358979323846

extern bool activateFilter, activateWindow;
extern bool iirFilter;
extern uint8_t activeFilter;				// choose which set of coefficients to use
extern float firCoeff[2][FIRTAPS];
extern float winCoeff[FIRTAPS];
extern float currentCutoff;
extern int16_t filterBuffer[2][FIRTAPS];	// Ring buffer containing most recent playback samples for quicker filtering from SRAM

extern bool calculatingFilter;				// For debugging
extern bool debugSort;


enum FilterType {FilterOff, LowPass, HighPass};
extern FilterType filterType;


void InitFilter(uint16_t tone);
float Sinc(float x);
void FIRFilterWindow(float beta);
float Bessel(float x);


typedef float iirdouble;

#define MAX_POLE_COUNT 20
#define ARRAY_DIM 50      // This MUST be at least 2*MAX_POLE_COUNT because some filter polys are defined in terms of 2 * NumPoles
#define OVERFLOW_LIMIT  1.0E5
#define IIRSAMPLECOUNT 1024
#define P51_MAXDEGREE   100       // The max poly order allowed. Used at the top of P51. This was set arbitrarily.
#define P51_ARRAY_SIZE  102       // P51 uses the new operator. P51 arrays must be MaxDegree + 2
#define MAX_ELLIP_ITER 15
#define ELLIPARRAYSIZE 20  // needs to be > 10 and >= Max Num Poles + 1


// These are the available filter polynomials. NOT_IIR is for code testing.
enum TFilterPoly {BUTTERWORTH, GAUSSIAN, BESSEL, ADJUSTABLE, CHEBYSHEV,	INVERSE_CHEBY, PAPOULIS, ELLIPTIC, NOT_IIR};

//enum TIIRPassTypes {iirLPF, iirHPF, iirBPF, iirNOTCH, iirALLPASS};

struct TIIRCoeff {
	iirdouble a0[ARRAY_DIM];
	iirdouble a1[ARRAY_DIM];
	iirdouble a2[ARRAY_DIM];
	iirdouble a3[ARRAY_DIM];
	iirdouble a4[ARRAY_DIM];
	iirdouble b0[ARRAY_DIM];
	iirdouble b1[ARRAY_DIM];
	iirdouble b2[ARRAY_DIM];
	iirdouble b3[ARRAY_DIM];
	iirdouble b4[ARRAY_DIM];
	int NumSections;
};

enum TOurSortTypes{stMax, stMin};

// These coeff form H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
// NumSections is the number of 1st and 2nd order polynomial factors .
struct TSPlaneCoeff {
	iirdouble N2[ARRAY_DIM];
	iirdouble N1[ARRAY_DIM];
	iirdouble N0[ARRAY_DIM];
	iirdouble D2[ARRAY_DIM];
	iirdouble D1[ARRAY_DIM];
	iirdouble D0[ARRAY_DIM];
	int NumSections;
};

struct filter
{
	struct {
		TFilterPoly ProtoType = BUTTERWORTH;
		int NumPoles = 1;				// 1 <= NumPoles <= 12, 15, 20 Depending on the filter.
		iirdouble Ripple = 0.1;			// 0.0 <= Ripple <= 1.0 dB     Chebyshev and Elliptic (less for high order Chebyshev).
		iirdouble StopBanddB = 60.0;		// 20 <= StopBand <= 120 dB    Inv Cheby and Elliptic
		iirdouble BW = 0.1;				// 0.0 < BandWidth < 1.0       3 dB bandwidth for bandpass and notch filters
		iirdouble dBGain = 0.0;			// -60.0 < dBGain < 60.0       All filters
	} iirSettings;

	TIIRCoeff IIRCoeff[2];
	FilterType filterType;
	void InitFIRFilter(uint16_t tone);
	void InitIIRFilter(uint16_t tone);
	void CalcIIRFilterCoeff(iirdouble OmegaC, FilterType PassType, TIIRCoeff &iirCoeff);
	TSPlaneCoeff CalcLowPassProtoCoeff();
	iirdouble IIRFilter(iirdouble sample, channel c);
	iirdouble SectCalc(int k, iirdouble x, channel c);
};

extern filter Filter;

void InitIIRFilter(uint16_t tone);
void FilterWithIIR(TIIRCoeff IIRCoeff, iirdouble *Signal, iirdouble *FilteredSignal, int NumSigPts);
iirdouble IIRFilter(iirdouble sample, channel c);
iirdouble SectCalc(int k, iirdouble x, channel c);
