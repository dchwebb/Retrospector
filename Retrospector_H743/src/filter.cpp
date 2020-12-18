#include "filter.h"

// Rectangular FIR
void InitFilter(float omegaC)
{
	float arg;
	// cycle between two sets of coefficients so one can be changed without affecting the other
	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	for(uint16_t j = 0; j < FIRTAPS; j++) {
		arg = (float)j - (float)(FIRTAPS - 1) / 2.0;
		firCoeff[inactiveFilter][j] = omegaC * Sinc(omegaC * arg * M_PI);
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
