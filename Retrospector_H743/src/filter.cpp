#include "Filter.h"

/*
 * CalcIIRFilterCoeff
 * 	CalcLowPassProtoCoeff
 *  	ButterworthPoly
 *  	GetFilterCoeff
 *  		SortRootsByZeta
 *  			HeapIndexSort
 */

// FIR data
float firCoeff[2][FIRTAPS];
float winCoeff[FIRTAPS];
int16_t filterBuffer[2][FIRTAPS];		// Ring buffer containing most recent playback samples for quicker filtering from SRAM

// Debug
bool calculatingFilter = false;
bool activateFilter = true;
bool activateWindow = true;
float currentCutoff;

bool debugSort = false;

// Rectangular FIR
void Filter::InitFIRFilter(uint16_t tone)
{
	float arg, omegaC;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass (settings optimised for 81 filter taps)
	if (tone < filterPotCentre - 1000) {		// Low Pass
		passType = LowPass;
		omegaC = 1.0f - std::pow(((float)filterPotCentre - tone) / 34000.0f, 0.2f);
	} else if (tone > filterPotCentre + 1000) {
		passType = HighPass;
		omegaC = 1.0f - std::pow(((float)tone - filterPotCentre)  / 75000.0f, 3.0f);
	} else {
		passType = FilterOff;
		omegaC = 1.0f;
	}

	// cycle between two sets of coefficients so one can be changed without affecting the other
	//uint8_t inactiveFilter = activeFilter;
	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	if (passType == LowPass) {
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;
			firCoeff[inactiveFilter][j] = omegaC * Sinc(omegaC * arg * M_PI) * (activateWindow ? winCoeff[j] : 1.0);
		}
	} else if (passType == HighPass)  {
		int8_t sign = 1;
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;

			firCoeff[inactiveFilter][j] = sign * omegaC * Sinc(omegaC * arg * M_PI) * (activateWindow ? winCoeff[j] : 1.0);
			sign = sign * -1;
		}
	}
	activeFilter = inactiveFilter;
	currentCutoff = omegaC;
}


void Filter::FIRFilterWindow(float beta)		// example has beta = 4.0 (value between 0.0 and 10.0)
{
	float arg;

	// Kaiser window
	for (uint8_t j = 0; j < FIRTAPS; j++) {
		arg = beta * sqrt(1.0 - pow( ((float)(2 * j) + 1 - FIRTAPS) / (FIRTAPS + 1), 2.0) );
		winCoeff[j] = Bessel(arg) / Bessel(beta);
	}
}

float Filter::Sinc(float x)
{
	if (x > -1.0E-5 && x < 1.0E-5)
		return(1.0);
	return (std::sin(x) / x);
}

// Used for Kaiser window calculations
float Filter::Bessel(float x)
{
	float Sum = 0.0, XtoIpower;
	int Factorial;
	for (uint8_t i = 1; i < 10; ++i) {
		XtoIpower = pow(x / 2.0, (float)i);
		Factorial = 1;
		for (uint8_t j = 1; j <= i; ++j) {
			Factorial *= j;
		}
		Sum += pow(XtoIpower / (float)Factorial, 2.0);
	}
	return(1.0 + Sum);
}




void Filter::InitIIRFilter(uint16_t tone)
{
	iirdouble cutoff;

	if (filterControl == LP) {			// Want a sweep from 0.03 to 0.999 with most travel at low end
		passType = LowPass;
		cutoff = std::min(0.03 + pow((iirdouble)tone / 65536.0, 2.0), 0.999);
	} else if (filterControl == HP) {		// Want a sweep from 0.001 to 0.2-0.3
		passType = HighPass;
		cutoff = 0.001 + pow(((iirdouble)tone / 100000.0), 3.0);
	} else {
		if (tone <= filterPotCentre) {		// Low Pass
			passType = LowPass;
			cutoff = std::min(pow(((iirdouble)tone + 15000) / 32000.0, 2.0), 0.999);
		} else if (tone > filterPotCentre) {
			passType = HighPass;
			cutoff = 0.001 + pow((((iirdouble)tone - filterPotCentre) / 50000.0), 3.0);
		}
	}

	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	// store reference to active filter (LP or HP and by activeFilter)
	IIRFilter& currentFilter = (passType == LowPass) ? iirLPFilter[inactiveFilter] : iirHPFilter[inactiveFilter];

	currentFilter.CalcIIRFilterCoeff(cutoff, passType);

	activeFilter = inactiveFilter;
	currentCutoff = cutoff;
}


//	Take a new sample and return filtered value
iirdouble Filter::CalcIIRFilter(iirdouble sample, channel c)
{
	// store reference to active filter (LP or HP and by activeFilter)
	IIRFilter& currentFilter = (passType == LowPass) ? iirLPFilter[activeFilter] : iirHPFilter[activeFilter];

	iirdouble y = SectCalc(0, sample, c, currentFilter);
	for (uint8_t k = 1; k < currentFilter.iirCoeff.NumSections; k++) {
		y = SectCalc(k, y, c, currentFilter);
	}
	return y;
}


// Calculates each stage of a multi-section IIR filter (eg 8 pole is constructed from four 2-pole filters)
iirdouble Filter::SectCalc(int k, iirdouble x, channel c, IIRFilter& currentFilter)
{
	iirdouble y, CenterTap;
	static iirdouble RegX1[2][MAX_POLES], RegX2[2][MAX_POLES], RegY1[2][MAX_POLES], RegY2[2][MAX_POLES];
	static iirdouble MaxRegVal = 1.0E-12;

	// Zero the registers on an overflow condition
	if (MaxRegVal > 1.0E5) {

		MaxRegVal = 1.0E-12;
		for (uint8_t i = 0; i < MAX_POLES; i++) {
			RegX1[c][i] = 0.0;
			RegX2[c][i] = 0.0;
			RegY1[c][i] = 0.0;
			RegY2[c][i] = 0.0;
		}
	}

	CenterTap = x * currentFilter.iirCoeff.b0[k] + currentFilter.iirCoeff.b1[k] * RegX1[c][k] + currentFilter.iirCoeff.b2[k] * RegX2[c][k];
	y = currentFilter.iirCoeff.a0[k] * CenterTap - currentFilter.iirCoeff.a1[k] * RegY1[c][k] - currentFilter.iirCoeff.a2[k] * RegY2[c][k];

	RegX2[c][k] = RegX1[c][k];
	RegX1[c][k] = x;
	RegY2[c][k] = RegY1[c][k];
	RegY1[c][k] = y;

	// MaxRegVal is used to prevent overflow.  Overflow seldom occurs, but will
	// if the filter has faulty coefficients. MaxRegVal is usually less than 100.0
	if (std::abs(CenterTap) > MaxRegVal)
		MaxRegVal = std::abs(CenterTap);
	if (std::abs(y) > MaxRegVal)
		MaxRegVal = std::abs(y);
	return(y);
}



/*
 Calculate the z-plane coefficients for IIR filters from 2nd order S-plane coefficients
 H(s) = ( Ds^2 + Es + F ) / ( As^2 + Bs + C )
 H(z) = ( b2z^2 + b1z + b0 ) / ( a2z^2 + a1z + a0 )
 */
void IIRFilter::CalcIIRFilterCoeff(iirdouble omega, PassType passType)
{
	int j;

	iirdouble A, B, C, D, E, F, T, Arg;

	// Init the IIR structure.
	for (j = 0; j < MAX_POLES; j++)	{
		iirCoeff.a0[j] = 0.0;  iirCoeff.b0[j] = 0.0;
		iirCoeff.a1[j] = 0.0;  iirCoeff.b1[j] = 0.0;
		iirCoeff.a2[j] = 0.0;  iirCoeff.b2[j] = 0.0;
	}

	// Set the number of IIR filter sections we will be generating.
	iirCoeff.NumSections = (numPoles + 1) / 2;

	// T sets the IIR filter's corner frequency, or center freqency.
	// The Bilinear transform is defined as:  s = 2/T * tan(Omega/2) = 2/T * (1 - z)/(1 + z)
	T = 2.0 * tan(omega * M_PI / 2);

	// Calc the IIR coefficients. SPlaneCoeff.NumSections is the number of 1st and 2nd order s plane factors.
	for (j = 0; j < iirProto.NumSections; j++) {
		A = iirProto.Coeff.D2[j];			// We use A - F to make the code easier to read.
		B = iirProto.Coeff.D1[j];
		C = iirProto.Coeff.D0[j];
		D = iirProto.Coeff.N2[j];
		E = iirProto.Coeff.N1[j];			// N1 is always zero, except for the all pass. Consequently, the equations below can be simplified a bit by removing E.
		F = iirProto.Coeff.N0[j];

		// b's are the numerator  a's are the denominator
		if (passType == LowPass) {
			if (A == 0.0 && D == 0.0) {					// 1 pole case
				Arg = (2.0 * B + C * T);
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (-2.0 * B + C * T) / Arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = (-2.0 * E + F * T) / Arg * C/F;
				iirCoeff.b0[j] = ( 2.0 * E + F * T) / Arg * C/F;
			} else {									// 2 poles

				Arg = (4.0 * A + 2.0 * B * T + C * T * T);
				iirCoeff.a2[j] = (4.0 * A - 2.0 * B * T + C * T * T) / Arg;
				iirCoeff.a1[j] = (2.0 * C * T * T - 8.0 * A) / Arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, our LPF numerator is (z+1)^2, so all our Z Plane zeros are at -1
				iirCoeff.b2[j] = (4.0 * D - 2.0 * E * T + F * T * T) / Arg * C/F;
				iirCoeff.b1[j] = (2.0 * F * T * T - 8.0 * D) / Arg * C/F;
				iirCoeff.b0[j] = (4*D + F * T * T + 2.0 * E * T) / Arg * C/F;
			}
		}

		if (passType == HighPass) { // High Pass
			if (A == 0.0 && D == 0.0) { // 1 pole
				Arg = 2.0 * C + B * T;
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (B * T - 2.0 * C) / Arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = (E * T - 2.0 * F) / Arg * C/F;
				iirCoeff.b0[j] = (E * T + 2.0 * F) / Arg * C/F;
			} else {  // 2 poles
				Arg = A * T * T + 4.0 * C + 2.0 * B * T;
				iirCoeff.a2[j] = (A * T * T + 4.0 * C - 2.0 * B * T) / Arg;
				iirCoeff.a1[j] = (2.0 * A * T * T - 8.0 * C) / Arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, our HPF numerator is (z-1)^2, so all our Z Plane zeros are at 1
				iirCoeff.b2[j] = (D * T * T - 2.0 * E * T + 4.0 * F) / Arg * C/F;
				iirCoeff.b1[j] = (2.0 * D * T * T - 8.0 * F) / Arg * C/F;
				iirCoeff.b0[j] = (D * T * T + 4.0 * F + 2.0 * E * T) / Arg * C/F;
			}
		}
	}

}


void IIRPrototype::CalcLowPassProtoCoeff()
{
	std::array<std::complex<double>, MAX_POLES> Poles;

	// Init the S Plane Coeff. H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
	for (uint8_t j = 0; j < MAX_POLES; j++) {
		Coeff.N2[j] = 0.0;
		Coeff.N1[j] = 0.0;
		Coeff.N0[j] = 1.0;
		Coeff.D2[j] = 0.0;
		Coeff.D1[j] = 0.0;
		Coeff.D0[j] = 1.0;
	}

	// A one pole filter is simply 1/(s+1)
	if (NumPoles == 1) {
		Coeff.D1[0] = 1.0;
	} else {				// Always use Butterworth
		Poles[0] =  std::complex<double>(0.0, 0.0) ;
		ButterworthPoly(Poles);
		GetFilterCoeff(Poles);
	}

	NumSections = NumPoles;
}


// Calculate the roots for a Butterworth filter: fill the array Roots[]
//void iirPrototype::ButterworthPoly(std::complex<double> *Roots)
void IIRPrototype::ButterworthPoly(std::array<std::complex<double>, MAX_POLES> &Roots)
{
	int n = 0;
	iirdouble theta;

	for (uint8_t j = 0; j < NumPoles / 2; j++) {
		theta = M_PI * (iirdouble)(2 * j + NumPoles + 1) / (iirdouble)(2 * NumPoles);
		Roots[n++] = std::complex<double>(cos(theta), sin(theta));
		Roots[n++] = std::complex<double>(cos(theta), -sin(theta));
	}
	if (NumPoles % 2 == 1)
		Roots[n++] = std::complex<double>(-1.0, 0.0);		// The real root for odd pole counts

}


// Create the 2nd order polynomials with coefficients A2, A1, A0.
void IIRPrototype::GetFilterCoeff(std::array<std::complex<double>, MAX_POLES> &Roots)
{
	int polyCount, j;

	// Sort the roots with the most negative real part first
	std::sort(Roots.begin(), Roots.end(), [](const std::complex<double> &lhs, const std::complex<double> &rhs) {
		return lhs.real() < rhs.real();
	});

	// This forms the 2nd order coefficients from the root value. Ignore roots in the Right Hand Plane.
	polyCount = 0;
	for (j = 0; j < NumPoles; j++) {
		if (Roots[j].real() > 0.0)
			continue;							// Right Hand Plane
		if (Roots[j].real() == 0.0 && Roots[j].imag() == 0.0)
			continue;							// At the origin.  This should never happen.

		if (Roots[j].real() == 0.0) {			// Imag Root (A poly zero)
			Coeff.D2[polyCount] = 1.0;
			Coeff.D1[polyCount] = 0.0;
			Coeff.D0[polyCount] = Roots[j].imag() * Roots[j].imag();
			j++;
			polyCount++;
		} else if (Roots[j].imag() == 0.0) {	// Real Pole
			Coeff.D2[polyCount] = 0.0;
			Coeff.D1[polyCount] = 1.0;
			Coeff.D0[polyCount] = -Roots[j].real();
			polyCount++;
		} else { 								// Complex Pole
			Coeff.D2[polyCount] = 1.0;
			Coeff.D1[polyCount] = -2.0 * Roots[j].real();
			Coeff.D0[polyCount] = Roots[j].real() * Roots[j].real() + Roots[j].imag() * Roots[j].imag();
			j++;
			polyCount++;
		}
	}

}
