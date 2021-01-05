#pragma once

#include "initialisation.h"
#include <cmath>
#include "CPLXD.hpp"
#include <math.h>

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

enum FilterType {FilterOff, LowPass, HighPass};
extern FilterType filterType;


void InitFilter(uint16_t tone);
float Sinc(float x);
void FIRFilterWindow(float beta);
float Bessel(float x);




#define MAX_POLE_COUNT 20
#define ARRAY_DIM 50      // This MUST be at least 2*MAX_POLE_COUNT because some filter polys are defined in terms of 2 * NumPoles
#define OVERFLOW_LIMIT  1.0E5
#define IIRSAMPLECOUNT 1024
#define P51_MAXDEGREE   100       // The max poly order allowed. Used at the top of P51. This was set arbitrarily.
#define P51_ARRAY_SIZE  102       // P51 uses the new operator. P51 arrays must be MaxDegree + 2



// These are the available filter polynomials. NOT_IIR is for code testing.
enum TFilterPoly {BUTTERWORTH, GAUSSIAN, BESSEL, ADJUSTABLE, CHEBYSHEV,	INVERSE_CHEBY, PAPOULIS, ELLIPTIC, NOT_IIR};
enum TIIRPassTypes {iirLPF, iirHPF, iirBPF, iirNOTCH, iirALLPASS};






struct TIIRCoeff {
	double a0[ARRAY_DIM];
	double a1[ARRAY_DIM];
	double a2[ARRAY_DIM];
	double a3[ARRAY_DIM];
	double a4[ARRAY_DIM];
	double b0[ARRAY_DIM];
	double b1[ARRAY_DIM];
	double b2[ARRAY_DIM];
	double b3[ARRAY_DIM];
	double b4[ARRAY_DIM];
	int NumSections;
};

//struct TIIRFilterParams {
//	TIIRPassTypes IIRPassType;    // Defined above: Low pass, High Pass, etc.
//	double OmegaC;                 // The IIR filter's 3 dB corner freq for low pass and high pass, the center freq for band pass and notch.
//	double BW;                     // The IIR filter's 3 dB bandwidth for band pass and notch filters.
//	double dBGain;                 // Sets the Gain of the filter
//
//	// These define the low pass prototype to be used
//	TFilterPoly ProtoType; // Butterworth, Cheby, etc.
//	int NumPoles;          // Pole count
//	double Ripple;          // Passband Ripple for the Elliptic and Chebyshev
//	double StopBanddB;      // Stop Band Attenuation in dB for the Elliptic and Inverse Chebyshev
//	double Gamma;           // Controls the transition bandwidth on the Adjustable Gauss. -1 <= Gamma <= 1
//};

enum TOurSortTypes{stMax, stMin};

// These coeff form H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
// NumSections is the number of 1st and 2nd order polynomial factors .
struct TSPlaneCoeff {
	double N2[ARRAY_DIM];
	double N1[ARRAY_DIM];
	double N0[ARRAY_DIM];
	double D2[ARRAY_DIM];
	double D1[ARRAY_DIM];
	double D0[ARRAY_DIM];
	int NumSections;
};

// This structure defines the low pass filter prototype.
// The 3 dB corner frequency is 1 rad/sec for all filters.
//struct TLowPassParams {
//	TFilterPoly ProtoType; // Butterworth, Cheby, etc.
//	int NumPoles;          // Pole count
//	double Ripple;          // Passband Ripple for the Elliptic and Chebyshev
//	double StopBanddB;      // Stop Band Attenuation in dB for the Elliptic and Inverse Cheby
//	double Gamma;           // Controls the transition bandwidth on the Adjustable Gauss. -1 <= Gamma <= 1
//};

//extern TIIRCoeff IIRCoeff;

struct filter
{
	struct {
		TFilterPoly ProtoType = BUTTERWORTH;
		int NumPoles = 6;				// 1 <= NumPoles <= 12, 15, 20 Depending on the filter.
		double Ripple = 0.1;			// 0.0 <= Ripple <= 1.0 dB     Chebyshev and Elliptic (less for high order Chebyshev).
		double StopBanddB = 60.0;		// 20 <= StopBand <= 120 dB    Inv Cheby and Elliptic
		double Gamma = 0.0;				// -1.0 <= Gamma <= 1.0        Adjustable Gauss  Controls the transition BW
		double BW = 0.1;				// 0.0 < BandWidth < 1.0       3 dB bandwidth for bandpass and notch filters
		double dBGain = 0.0;			// -60.0 < dBGain < 60.0       All filters
	} iirSettings;

	TIIRCoeff IIRCoeff[2];

	void InitIIRFilter(uint16_t tone);
	void CalcIIRFilterCoeff(double OmegaC, TIIRPassTypes PassType, TIIRCoeff &iirCoeff);
	TSPlaneCoeff CalcLowPassProtoCoeff();
	double IIRFilter(double sample, channel c);
	double SectCalc(int k, double x, channel c);
};

extern filter Filter;

void InitIIRFilter(uint16_t tone);
void FilterWithIIR(TIIRCoeff IIRCoeff, double *Signal, double *FilteredSignal, int NumSigPts);
double IIRFilter(double sample, channel c);
double SectCalc(int k, double x, channel c);
