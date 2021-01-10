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
extern float firCoeff[2][FIRTAPS];
extern float winCoeff[FIRTAPS];
extern float currentCutoff;
extern int16_t filterBuffer[2][FIRTAPS];	// Ring buffer containing most recent playback samples for quicker filtering from SRAM

extern bool calculatingFilter;				// For debugging
extern bool debugSort;

enum PassType {FilterOff, LowPass, HighPass};
enum FilterType {FIR, IIR};
enum TFilterPoly {BUTTERWORTH, GAUSSIAN, BESSEL, ADJUSTABLE, CHEBYSHEV,	INVERSE_CHEBY, PAPOULIS, ELLIPTIC, NOT_IIR};

typedef float iirdouble;

#define MAX_POLE_COUNT 20
#define ARRAY_DIM 50 				// This MUST be at least 2*MAX_POLE_COUNT because some filter polys are defined in terms of 2 * NumPoles
#define OVERFLOW_LIMIT  1.0E5
#define IIRSAMPLECOUNT 1024
#define P51_MAXDEGREE   100			// The max poly order allowed. Used at the top of P51. This was set arbitrarily.
#define P51_ARRAY_SIZE  102			// P51 uses the new operator. P51 arrays must be MaxDegree + 2
#define MAX_ELLIP_ITER 15
#define ELLIPARRAYSIZE 20			// needs to be > 10 and >= Max Num Poles + 1


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
	uint16_t filterPotCentre = 29000;		//32768	FIXME - make this configurable

	struct {
		TFilterPoly ProtoType = ELLIPTIC;
		int NumPoles = 1;				// 1 <= NumPoles <= 12, 15, 20 Depending on the filter.
		iirdouble Ripple = 0.1;			// 0.0 <= Ripple <= 1.0 dB     Chebyshev and Elliptic (less for high order Chebyshev).
		iirdouble StopBanddB = 60.0;	// 20 <= StopBand <= 120 dB    Inv Cheby and Elliptic
		iirdouble BW = 0.1;				// 0.0 < BandWidth < 1.0       3 dB bandwidth for bandpass and notch filters
		iirdouble dBGain = 0.0;			// -60.0 < dBGain < 60.0       All filters
	} iirSettings;

	TIIRCoeff IIRCoeff[2];
	FilterType filterType = IIR;
	PassType passType;
	uint8_t activeFilter = 0;				// choose which set of coefficients to use (so coefficients can be calculated without interfering with current filtering)

	void InitFIRFilter(uint16_t tone);
	void InitIIRFilter(uint16_t tone);
	void CalcIIRFilterCoeff(iirdouble OmegaC, PassType PassType, TIIRCoeff &iirCoeff);
	TSPlaneCoeff CalcLowPassProtoCoeff();
	iirdouble IIRFilter(iirdouble sample, channel c);
	iirdouble SectCalc(int k, iirdouble x, channel c);
	float Sinc(float x);
	void FIRFilterWindow(float beta);
	float Bessel(float x);
	int GetFilterCoeff(int RootCount, CplxD *Roots, iirdouble *A2, iirdouble *A1, iirdouble *A0);
	void SetCornerFreq(int PolyCount, iirdouble *D2, iirdouble *D1, iirdouble *D0, iirdouble *N2, iirdouble *N1, iirdouble *N0);

};

extern filter Filter;
