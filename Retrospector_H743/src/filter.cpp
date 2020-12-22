#include "filter.h"

// Rectangular FIR
void InitFilter(uint16_t tone)
{
	float arg, omegaC;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass (settings optimised for 81 filter taps)
	if (tone < 32768) {		// Low Pass
		filterType = LowPass;
		omegaC = 1.0f - std::pow((32768.0f - tone) / 33000.0f, 0.2f);
	} else {
		filterType = HighPass;
		omegaC = 1.0f - std::pow(((float)tone - 32768.0f)  / 40000.0f, 7.0f);
	}

	// cycle between two sets of coefficients so one can be changed without affecting the other
	//uint8_t inactiveFilter = activeFilter;
	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	if (filterType == LowPass) {
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;
			firCoeff[inactiveFilter][j] = omegaC * Sinc(omegaC * arg * M_PI);
		}
	} else if (filterType == HighPass)  {
		int8_t sign = 1;
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;

			firCoeff[inactiveFilter][j] = sign * omegaC * Sinc(omegaC * arg * M_PI);
			sign = sign * -1;
		}
	}
	activeFilter = inactiveFilter;
	currentCutoff = omegaC;
}

float Sinc(float x)
{
	if (x > -1.0E-5 && x < 1.0E-5)
		return(1.0);
	return (std::sin(x) / x);
}
