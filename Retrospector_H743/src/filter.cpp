#include "filter.h"

bool calculatingFilter = false;

// Rectangular FIR
void InitFilter(uint16_t tone)
{
	float arg, omegaC;

	calculatingFilter = true;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass (settings optimised for 81 filter taps)
	if (tone < 32768) {		// Low Pass
		filterType = LowPass;
		omegaC = 1.0f - std::pow((32768.0f - tone) / 34000.0f, 0.2f);
	} else {
		filterType = HighPass;
		omegaC = 1.0f - std::pow(((float)tone - 32768.0f)  / 55000.0f, 3.0f);
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

float winCoeff[FIRTAPS];

void FIRFilterWindow(float beta)		// example has beta = 4.0 (value between 0.0 and 10.0)
{
	float dN, arg;

	// Calculate the window for N/2 points, then fold the window over (at the bottom).
	dN = FIRTAPS + 1; // a double

	// Kaiser
	for (uint8_t j = 0; j < FIRTAPS; j++) {
		arg = beta * sqrt(1.0 - pow( ((double)(2 * j + 2) - dN) / dN, 2.0) );
		winCoeff[j] = Bessel(arg) / Bessel(beta);
	}

	// Fold the coefficients over
	for (uint8_t j = 0; j < FIRTAPS / 2; j++)
		winCoeff[FIRTAPS - j - 1] = winCoeff[j];

	// Apply the window to the FIR coefficients.
	//for(j=0; j<N; j++)FIRCoeff[j] *= WinCoeff[j];

}

float Sinc(float x)
{
	if (x > -1.0E-5 && x < 1.0E-5)
		return(1.0);
	return (std::sin(x) / x);
}

// This gets used with the Kaiser window.
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
