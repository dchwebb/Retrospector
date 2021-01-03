#include "filter.h"

// FIR data
float firCoeff[2][FIRTAPS];
float winCoeff[FIRTAPS];
int16_t filterBuffer[2][FIRTAPS];		// Ring buffer containing most recent playback samples for quicker filtering from SRAM
uint8_t activeFilter = 0;				// choose which set of coefficients to use (so coefficients can be calculated without interfering with current filtering)

// Debug
bool calculatingFilter = false;
bool activateFilter = true;
bool activateWindow = true;
float currentCutoff;
uint16_t filterPotCentre = 29000;		//32768	FIXME - make this configurable

// Rectangular FIR
void InitFilter(uint16_t tone)
{
	float arg, omegaC;
	FilterType filterType;

	calculatingFilter = true;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass (settings optimised for 81 filter taps)
	if (tone < filterPotCentre - 1000) {		// Low Pass
		filterType = LowPass;
		omegaC = 1.0f - std::pow(((float)filterPotCentre - tone) / 34000.0f, 0.2f);
	} else if (tone > filterPotCentre + 1000) {
		filterType = HighPass;
		omegaC = 1.0f - std::pow(((float)tone - filterPotCentre)  / 75000.0f, 3.0f);
	} else {
		filterType = FilterOff;
		omegaC = 1.0f;
	}

	// cycle between two sets of coefficients so one can be changed without affecting the other
	//uint8_t inactiveFilter = activeFilter;
	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	if (filterType == LowPass) {
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;
			firCoeff[inactiveFilter][j] = omegaC * Sinc(omegaC * arg * M_PI) * (activateWindow ? winCoeff[j] : 1.0);
		}
	} else if (filterType == HighPass)  {
		int8_t sign = 1;
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;

			firCoeff[inactiveFilter][j] = sign * omegaC * Sinc(omegaC * arg * M_PI) * (activateWindow ? winCoeff[j] : 1.0);
			sign = sign * -1;
		}
	}
	activeFilter = inactiveFilter;
	currentCutoff = omegaC;

	calculatingFilter = false;
}


void FIRFilterWindow(float beta)		// example has beta = 4.0 (value between 0.0 and 10.0)
{
	float arg;

	// Kaiser window
	for (uint8_t j = 0; j < FIRTAPS; j++) {
		arg = beta * sqrt(1.0 - pow( ((float)(2 * j) + 1 - FIRTAPS) / (FIRTAPS + 1), 2.0) );
		winCoeff[j] = Bessel(arg) / Bessel(beta);
	}

}

float Sinc(float x)
{
	if (x > -1.0E-5 && x < 1.0E-5)
		return(1.0);
	return (std::sin(x) / x);
}

// Used for Kaiser window calculations
float Bessel(float x)
{
	float Sum = 0.0, XtoIpower;
	int Factorial;
	for (uint8_t i = 1; i < 10; ++i) {
		XtoIpower = pow(x/2.0, (float)i);
		Factorial = 1;
		for (uint8_t j = 1; j <= i; ++j) {
			Factorial *= j;
		}
		Sum += pow(XtoIpower / (float)Factorial, 2.0);
	}
	return(1.0 + Sum);
}


#if false
#define MAX_POLE_COUNT 20
#define ARRAY_DIM 50      // This MUST be at least 2*MAX_POLE_COUNT because some filter polys are defined in terms of 2 * NumPoles

// These are the available filter polynomials. NOT_IIR is for code testing.
enum TFilterPoly {BUTTERWORTH, GAUSSIAN, BESSEL, ADJUSTABLE, CHEBYSHEV,
                  INVERSE_CHEBY, PAPOULIS, ELLIPTIC, NOT_IIR};
enum TIIRPassTypes {iirLPF, iirHPF, iirBPF, iirNOTCH, iirALLPASS};

struct TIIRCoeff {double a0[ARRAY_DIM]; double a1[ARRAY_DIM]; double a2[ARRAY_DIM]; double a3[ARRAY_DIM]; double a4[ARRAY_DIM];
				   double b0[ARRAY_DIM]; double b1[ARRAY_DIM]; double b2[ARRAY_DIM]; double b3[ARRAY_DIM]; double b4[ARRAY_DIM];
                  int NumSections; };

struct TIIRFilterParams { TIIRPassTypes IIRPassType;     // Defined above: Low pass, High Pass, etc.
                          double OmegaC;                 // The IIR filter's 3 dB corner freq for low pass and high pass, the center freq for band pass and notch.
                          double BW;                     // The IIR filter's 3 dB bandwidth for band pass and notch filters.
                          double dBGain;                 // Sets the Gain of the filter

                          // These define the low pass prototype to be used
                          TFilterPoly ProtoType;  // Butterworth, Cheby, etc.
                          int NumPoles;           // Pole count
                          double Ripple;          // Passband Ripple for the Elliptic and Chebyshev
                          double StopBanddB;      // Stop Band Attenuation in dB for the Elliptic and Inverse Chebyshev
                          double Gamma;           // Controls the transition bandwidth on the Adjustable Gauss. -1 <= Gamma <= 1
                        };

//---------------------------------------------------------------------------
/*
 This calculates the coefficients for IIR filters from a set of 2nd order s plane coefficients
 which are obtained by calling CalcLowPassProtoCoeff() in LowPassPrototypes.cpp.
 The s plane filters are frequency scaled so their 3 dB frequency is at s = omega = 1 rad/sec.
 The poles and zeros are also ordered in a manner appropriate for IIR filters.
 For a derivation of the formulas used here, see the IIREquationDerivations.pdf
 This shows how the various poly coefficients are defined.
 H(s) = ( Ds^2 + Es + F ) / ( As^2 + Bs + C )
 H(z) = ( b2z^2 + b1z + b0 ) / ( a2z^2 + a1z + a0 )
*/
TIIRCoeff CalcIIRFilterCoeff(TIIRFilterParams IIRFilt)
{
	int j, k;
	double Scalar, SectionGain, Coeff[5];
	double A, B, C, D, E, F, T, Q, Arg;
	double a2[ARRAY_DIM], a1[ARRAY_DIM], a0[ARRAY_DIM];
	double b2[ARRAY_DIM], b1[ARRAY_DIM], b0[ARRAY_DIM];
	CplxD Roots[5];

	TIIRCoeff IIR;                // Gets returned by this function.
	TLowPassParams LowPassFilt;   // Passed to the CalcLowPassProtoCoeff() function.
	TSPlaneCoeff SPlaneCoeff;     // Filled by the CalcLowPassProtoCoeff() function.


	// We can set the TLowPassParams variables directly from the TIIRFilterParams variables.
	LowPassFilt.ProtoType = IIRFilt.ProtoType;
	LowPassFilt.NumPoles = IIRFilt.NumPoles;
	LowPassFilt.Ripple = IIRFilt.Ripple;
	LowPassFilt.Gamma = IIRFilt.Gamma;
	LowPassFilt.StopBanddB = IIRFilt.StopBanddB;

	// Get the low pass prototype 2nd order s plane coefficients.
	SPlaneCoeff = CalcLowPassProtoCoeff(LowPassFilt);


	// Init the IIR structure.
	for (j=0; j<ARRAY_DIM; j++)	{
		IIR.a0[j] = 0.0;  IIR.b0[j] = 0.0;
		IIR.a1[j] = 0.0;  IIR.b1[j] = 0.0;
		IIR.a2[j] = 0.0;  IIR.b2[j] = 0.0;
		IIR.a3[j] = 0.0;  IIR.b3[j] = 0.0;
		IIR.a4[j] = 0.0;  IIR.b4[j] = 0.0;
	}

	// Set the number of IIR filter sections we will be generating.
	IIR.NumSections = (IIRFilt.NumPoles + 1) / 2;
	if(IIRFilt.IIRPassType == iirBPF || IIRFilt.IIRPassType == iirNOTCH)IIR.NumSections = IIRFilt.NumPoles;



	// T sets the IIR filter's corner frequency, or center freqency.
	// The Bilinear transform is defined as:  s = 2/T * tan(Omega/2) = 2/T * (1 - z)/(1 + z)
	T = 2.0 * tan(IIRFilt.OmegaC * M_PI_2);
	Q = 1.0 + IIRFilt.OmegaC;             // Q is used for band pass and notch filters.
	if (Q > 1.95)
		Q = 1.95;
	Q = 0.8 * tan(Q * M_PI_4);            // This is a correction factor for Q.
	Q = IIRFilt.OmegaC / IIRFilt.BW / Q;  // This is the corrected Q.


	// Calc the IIR coefficients.
	// SPlaneCoeff.NumSections is the number of 1st and 2nd order s plane factors.
	k = 0;
	for(j=0; j<SPlaneCoeff.NumSections; j++)
	{
		A = SPlaneCoeff.D2[j]; // We use A - F to make the code easier to read.
		B = SPlaneCoeff.D1[j];
		C = SPlaneCoeff.D0[j];
		D = SPlaneCoeff.N2[j];
		E = SPlaneCoeff.N1[j]; // N1 is always zero, except for the all pass. Consequently, the equations below can be simplified a bit by removing E.
		F = SPlaneCoeff.N0[j];

		// b's are the numerator  a's are the denominator
		if (IIRFilt.IIRPassType == iirLPF || IIRFilt.IIRPassType == iirALLPASS) // Low Pass and All Pass
		{
			if (A == 0.0 && D == 0.0) // 1 pole case
			{
				Arg = (2.0*B + C*T);
				IIR.a2[j] = 0.0;
				IIR.a1[j] = (-2.0*B + C*T) / Arg;
				IIR.a0[j] = 1.0;

				IIR.b2[j] = 0.0;
				IIR.b1[j] = (-2.0*E + F*T) / Arg * C/F;
				IIR.b0[j] = ( 2.0*E + F*T) / Arg * C/F;
			} else // 2 poles
			{
				Arg = (4.0*A + 2.0*B*T + C*T*T);
				IIR.a2[j] = (4.0*A - 2.0*B*T + C*T*T) / Arg;
				IIR.a1[j] = (2.0*C*T*T - 8.0*A) / Arg;
				IIR.a0[j] = 1.0;

				// With all pole filters, our LPF numerator is (z+1)^2, so all our Z Plane zeros are at -1
				IIR.b2[j] = (4.0*D - 2.0*E*T + F*T*T) / Arg * C/F;
				IIR.b1[j] = (2.0*F*T*T - 8.0*D) / Arg * C/F;
				IIR.b0[j] = (4*D + F*T*T + 2.0*E*T) / Arg * C/F;
			}
		}

		if (IIRFilt.IIRPassType == iirHPF) // High Pass
		{
			if (A == 0.0 && D == 0.0) // 1 pole
			{
				Arg = 2.0*C + B*T;
				IIR.a2[j] = 0.0;
				IIR.a1[j] = (B*T - 2.0*C) / Arg;
				IIR.a0[j] = 1.0;

				IIR.b2[j] = 0.0;
				IIR.b1[j] = (E*T - 2.0*F) / Arg * C/F;
				IIR.b0[j] = (E*T + 2.0*F) / Arg * C/F;
			} else  // 2 poles
			{
				Arg = A*T*T + 4.0*C + 2.0*B*T;
				IIR.a2[j] = (A*T*T + 4.0*C - 2.0*B*T) / Arg;
				IIR.a1[j] = (2.0*A*T*T - 8.0*C) / Arg;
				IIR.a0[j] = 1.0;

				// With all pole filters, our HPF numerator is (z-1)^2, so all our Z Plane zeros are at 1
				IIR.b2[j] = (D*T*T - 2.0*E*T + 4.0*F) / Arg * C/F;
				IIR.b1[j] = (2.0*D*T*T - 8.0*F) / Arg * C/F;
				IIR.b0[j] = (D*T*T + 4.0*F + 2.0*E*T) / Arg * C/F;
			}
		}



	// Adjust the b's or a0 for the desired Gain.
	SectionGain = pow(10.0, IIRFilt.dBGain/20.0);
	SectionGain = pow(SectionGain, 1.0/(double)IIR.NumSections);
	for (j=0; j<IIR.NumSections; j++) {
		IIR.b0[j] *= SectionGain;
		IIR.b1[j] *= SectionGain;
		IIR.b2[j] *= SectionGain;
		// This is an alternative to adjusting the b's
		// IIR.a0[j] = SectionGain;
	}

	return(IIR);
}
#endif
