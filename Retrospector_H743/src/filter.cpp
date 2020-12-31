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
